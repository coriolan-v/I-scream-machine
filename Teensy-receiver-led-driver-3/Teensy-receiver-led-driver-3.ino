/*
  Teensy 4.1 LED renderer (Dual Mic Audio Tunnel - Mirrored Double-Sided Tube)

  FESTIVAL QUICK-CONFIGURATION SKETCH (DECIBEL CALIBRATION VERSION)
  - Modify parameters in the first block below using standard audio dB scales.
  - 0.0f dB is absolute digital maximum (clipping). 
  - Negative numbers represent headroom below max (e.g., -3.0f dB is roughly half amplitude).
*/

#include <Arduino.h>
#include <OctoWS2811.h>

// ============================================================================
// ====== FESTIVAL ON-SITE TWEAKING PARAMETERS (DECIBEL LOG SCALE) ============
// ============================================================================

// --- Scream Overdrive Thresholds (In dB) ---
// 0.0f dB = Max possible volume before sensor clips.
const float   SCREAM_DB_LIMIT       = -1.5f;   // Lower this (e.g., -4.0f or -6.0f) if people scream but it won't go red. Raise closer to 0.0f if it goes red too easily.
const float   SCREAM_WHITE_HOT_DB   = -0.2f;   // Headroom threshold to trigger the white-hot entry blast. Must be higher than SCREAM_DB_LIMIT (closer to 0.0f).
const uint8_t SCREAM_OVERDRIVE_HUE  = 0;       // 0 = Crimson Red, 32 = Orange, 96 = Green, 160 = Blue.

// --- System Sensitivity & Tuning ---
static constexpr float    SCREAM_SMOOTH_LIMIT   = 0.85f;  // Percentage-based trigger for sustained volume peaks (0.0 to 1.0).
static constexpr float    SCREAM_WHITE_HOT_BOOST = 120.0f; // Intensity of the white flash core (0.0 to 255.0).
static constexpr uint32_t IDLE_TIMEOUT_MS       = 1000;   // Milliseconds of silence before idle kicks in.
static constexpr float    IDLE_SWEEP_SPEED      = 0.02f;  // Speed of the ambient idle ring sweep.

// --- Global Hardware Layout Geometry ---
static constexpr float    OUTSIDE_BRIGHTNESS    = 1.0f;   // Exterior tube brightness scalar (0.0 to 1.0)
static constexpr float    INSIDE_BRIGHTNESS     = 0.5f;   // Interior mirror tube brightness scalar (0.0 to 1.0)
static constexpr int      IDLE_PIXEL_LIMIT      = 12;     // How many pixels deep the idle animation travels.

// ============================================================================
// ============================================================================

// --- Dynamically Calculated Raw Thresholds (Computed on Boot) ---
uint8_t rawScreamLimit    = 235;
uint8_t rawWhiteHotLimit  = 252;

// ---------- Performance & Timing Configuration ----------
static constexpr uint32_t TARGET_FPS = 55;        
static constexpr uint32_t FRAME_PERIOD_MS = 1000 / TARGET_FPS;

// ---------- Dual UART Configuration ----------
#define MIC1_SERIAL Serial7   
#define MIC2_SERIAL Serial1   
#define MIC1_SERIAL_RTS 27
#define MIC2_SERIAL_RTS 2
#define LED_DEBUG 13

#define VERBOSE true
#define PACKET_DEBUG_INTERVAL_MS 200

// ---------- LEDs Physical Geometry ----------
static constexpr int NUM_STRIPS = 12;
static constexpr int LEDS_PER_STRIP = 384;            
static constexpr int SIDE_LEDS = LEDS_PER_STRIP / 2;  
static constexpr int NUM_LEDS = NUM_STRIPS * LEDS_PER_STRIP;
static constexpr uint32_t UART_BAUD = 460800;

byte pinList[NUM_STRIPS] = { 21, 18, 19, 20, 36, 38, 33, 37, 34, 35, 40, 39 };
const int bytesPerLED = 3;

DMAMEM int displayMemory[LEDS_PER_STRIP * NUM_STRIPS * bytesPerLED / 4];
int drawingMemory[LEDS_PER_STRIP * NUM_STRIPS * bytesPerLED / 4];

const int config = WS2811_GRB | WS2811_800kHz;
OctoWS2811 leds(LEDS_PER_STRIP, displayMemory, drawingMemory, config, NUM_STRIPS, pinList);

struct AudioFrame {
  uint8_t volume = 0, bass = 0, mid = 0, treble = 0, noteHue = 0, beat = 0;
};

struct MicEngine {
  AudioFrame audio;
  uint32_t lastFrameMs = 0, frameCounter = 0, badChecksumCounter = 0, rawByteCounter = 0;
  uint8_t state = 0, buf[8], idx = 0;
  float smVol = 0, smBass = 0, smMid = 0, smTreble = 0, smNoteHue = 0, beatPulse = 0;
};

MicEngine mic1; MicEngine mic2;
enum RenderMode { TUNNEL_MODE, CLUSTER_MODE };
RenderMode currentMode = TUNNEL_MODE;

float flow = 0, clockTrackAngle = 0.0f, targetClockTrack = 0.0f;     
uint32_t lastRandomHopMs = 0, lastFrameRenderMs = 0, lastAudioActivityMs = 0;   
float idleTracker = 0.0f, idleFade = 0.0f;              

// ---------- Helpers ----------
uint8_t checksumFrame(uint8_t *f, int n) {
  uint8_t c = 0; for (int i = 0; i < n; i++) c ^= f[i]; return c;
}

uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

int ledIndex(int strip, int pixel) { return strip * LEDS_PER_STRIP + pixel; }

void fillAll(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) leds.setPixel(i, color); leds.show();
}

void startupLedTest() {
  fillAll(rgb(50, 0, 0)); delay(1000);
  fillAll(rgb(0, 50, 0)); delay(1000);
  fillAll(rgb(0, 0, 50)); delay(1000);
  fillAll(rgb(0, 0, 0));   delay(1000);
}

bool readAudioFrame(HardwareSerial &serialPort, MicEngine &mic) {
  while (serialPort.available()) {
    uint8_t b = serialPort.read(); mic.rawByteCounter++;
    switch (mic.state) {
      case 0: if (b == 0xAA) mic.state = 1; break;
      case 1: if (b == 0x55) { mic.buf[0] = 0xAA; mic.buf[1] = 0x55; mic.idx = 2; mic.state = 2; } else { mic.state = 0; } break;
      case 2: mic.buf[mic.idx++] = b; if (mic.idx >= 8) mic.state = 3; break;
      case 3: {
        if (b == checksumFrame(mic.buf, 8)) {
          mic.audio.volume = mic.buf[2]; mic.audio.bass = mic.buf[3]; mic.audio.mid = mic.buf[4];
          mic.audio.treble = mic.buf[5]; mic.audio.noteHue = mic.buf[6]; mic.audio.beat = mic.buf[7];
          mic.lastFrameMs = millis(); mic.frameCounter++; mic.state = 0; return true;
        } else { mic.badChecksumCounter++; mic.state = 0; }
        break;
      }
    }
  }
  return false;
}

uint32_t hsvToRgb(uint8_t h, uint8_t s, uint8_t v) {
  uint8_t region = h / 43, remainder = (h - region * 43) * 6;
  uint8_t p = (v * (255 - s)) >> 8, q = (v * (255 - ((s * remainder) >> 8))) >> 8, t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
  switch (region) {
    case 0: return rgb(v, t, p); case 1: return rgb(q, v, p); case 2: return rgb(p, v, t);
    case 3: return rgb(p, q, v); case 4: return rgb(t, p, v); default: return rgb(v, p, q);
  }
}

uint32_t scaleColor(uint32_t c, float scale) {
  scale = constrain(scale, 0.0f, 1.0f);
  return rgb(((c >> 16) & 0xFF) * scale, ((c >> 8) & 0xFF) * scale, (c & 0xFF) * scale);
}

uint32_t addColor(uint32_t a, uint32_t b) {
  return rgb(min(255, ((a >> 16) & 0xFF) + ((b >> 16) & 0xFF)), min(255, ((a >> 8) & 0xFF) + ((b >> 8) & 0xFF)), min(255, (a & 0xFF) + (b & 0xFF)));
}

float getWaveIntensity(int strip, int targetPixel, MicEngine &mic, float speedModifier, uint32_t &outColor) {
  float vol = mic.smVol / 255.0f; if (vol < 0.03f) { outColor = 0; return 0.0f; }
  float bass = mic.smBass / 255.0f, mid = mic.smMid / 255.0f, treble = mic.smTreble / 255.0f, x = (float)targetPixel;
  
  float phase = (flow * speedModifier) - (x * 0.45f) + ((float)strip * (3.5f + (treble * 4.0f)));
  float fireWaves = powf((sinf(phase * 0.15f) * cosf(phase * 0.08f + strip) * 0.5f) + 0.5f, 2.0f);     
  float beatWave = max(0.0f, sinf(((flow * 2.5f) - (x * 0.60f) + (strip * 1.2f)) * 0.1f)) * mic.beatPulse * 1.5f;

  float structuralCooling = constrain(1.0f - (x / (float)SIDE_LEDS), 0.0f, 1.0f);
  uint32_t hash = (uint32_t)(targetPixel * 1103515245UL + strip * 76543UL + (uint32_t)(flow * 0.5f));
  float sparkle = (((hash >> 24) & 0xFF) / 255.0f) > (0.988f - treble * 0.05f) ? treble * structuralCooling * 0.8f : 0.0f;

  float intensity = powf(constrain((fireWaves * (0.35f + mid * 0.5f) + beatWave) * vol * 1.5f * structuralCooling + sparkle, 0.0f, 1.0f), 1.3f);
  if (intensity < 0.01f) { outColor = 0; return 0.0f; }

  // --- Scream Overdrive Intercept Logic (Utilizes Pre-Calculated dB Boundaries) ---
  if (mic.audio.volume > rawScreamLimit || vol > SCREAM_SMOOTH_LIMIT) {
    outColor = hsvToRgb(SCREAM_OVERDRIVE_HUE, 255, (uint8_t)(intensity * 255.0f));
    if (mic.audio.volume >= rawWhiteHotLimit) {
      uint8_t coreBoost = (uint8_t)(structuralCooling * SCREAM_WHITE_HOT_BOOST);
      outColor = addColor(outColor, rgb(coreBoost, coreBoost / 2, coreBoost / 2)); 
    }
  } else {
    float totalEnergy = bass + mid + treble; uint8_t targetHue = (uint8_t)mic.smNoteHue;
    if (totalEnergy > 0.05f) targetHue = (uint8_t)(targetHue * 0.4f + ((bass * 0.0f + mid * 80.0f + treble * 175.0f) / totalEnergy) * 0.6f);
    outColor = hsvToRgb(targetHue + (strip * 2), 255 - (bass * 30), (uint8_t)(intensity * 255.0f));
  }

  if (x < 24.0f) {
    float injectionGlow = (1.0f - (x / 24.0f)) * (bass * 0.6f + vol * 0.4f);
    uint8_t injHue = (mic.audio.volume > rawScreamLimit) ? SCREAM_OVERDRIVE_HUE : (mic.smNoteHue - 8);
    outColor = addColor(outColor, hsvToRgb(injHue, 255, (uint8_t)(injectionGlow * 150)));
  }
  return intensity;
}

void applyIdleAnimation(float alpha) {
  float speedNormalization = 60.0f / (float)TARGET_FPS;
  idleTracker += IDLE_SWEEP_SPEED * speedNormalization;
  if (idleTracker >= 12.0f) idleTracker -= 12.0f;
  uint8_t cyclicHue = (uint8_t)((millis() / 60) % 256);

  for (int s = 0; s < NUM_STRIPS; s++) {
    float angularDiff = fabsf((float)s - idleTracker); if (angularDiff > 6.0f) angularDiff = 12.0f - angularDiff;
    float stripGlow = powf(max(0.0f, 1.0f - (angularDiff / 2.0f)), 2.0f); 

    for (int p = 0; p < IDLE_PIXEL_LIMIT; p++) {
      float combinedScalar = stripGlow * (1.0f - ((float)p / (float)IDLE_PIXEL_LIMIT)) * 0.6f * alpha; 
      if (combinedScalar > 0.005f) {
        uint32_t idleColor = hsvToRgb(cyclicHue + (s * 4), 220, (uint8_t)(combinedScalar * 255.0f));
        int out1 = ledIndex(s, p), in1 = ledIndex(s, SIDE_LEDS + (SIDE_LEDS - 1 - p));
        leds.setPixel(out1, addColor(leds.getPixel(out1), scaleColor(idleColor, OUTSIDE_BRIGHTNESS)));
        leds.setPixel(in1,  addColor(leds.getPixel(in1),  scaleColor(idleColor, INSIDE_BRIGHTNESS)));
        int out2 = ledIndex(s, (SIDE_LEDS - 1) - p), in2 = ledIndex(s, SIDE_LEDS + p);
        leds.setPixel(out2, addColor(leds.getPixel(out2), scaleColor(idleColor, OUTSIDE_BRIGHTNESS)));
        leds.setPixel(in2,  addColor(leds.getPixel(in2),  scaleColor(idleColor, INSIDE_BRIGHTNESS)));
      }
    }
  }
}

void renderAudioTunnel() {
  float v1 = mic1.smVol / 255.0f, v2 = mic2.smVol / 255.0f, volAvg = (v1 + v2) * 0.5f, speedNorm = 60.0f / (float)TARGET_FPS;
  flow += (1.2f + ((mic1.smMid + mic2.smMid) / 510.0f) * 4.0f + ((mic1.smBass + mic2.smBass) / 510.0f) * 2.0f) * speedNorm;
  if (flow > 200000.0f) flow = 0;

  uint32_t currentMs = millis();
  bool beatDetected = (mic1.audio.beat > 0 || mic2.audio.beat > 0), quietTimeout = (currentMs - lastRandomHopMs > 1200);
  if (beatDetected || quietTimeout) { targetClockTrack = (float)random(0, NUM_STRIPS); lastRandomHopMs = currentMs; if (volAvg < 0.04f && beatDetected) clockTrackAngle = targetClockTrack; }

  float approachDiff = targetClockTrack - clockTrackAngle;
  if (approachDiff > 6.0f) approachDiff -= 12.0f; if (approachDiff < -6.0f) approachDiff += 12.0f;
  clockTrackAngle += approachDiff * (0.18f * speedNorm);
  if (clockTrackAngle >= 12.0f) clockTrackAngle -= 12.0f; if (clockTrackAngle < 0.0f) clockTrackAngle += 12.0f;

  float decayFactor = powf(0.85f, speedNorm);
  mic1.beatPulse *= decayFactor; if (mic1.audio.beat > 0) mic1.beatPulse = 1.0f;
  mic2.beatPulse *= decayFactor; if (mic2.audio.beat > 0) mic2.beatPulse = 1.0f;

  if (currentMs - mic1.lastFrameMs > 500) { mic1.smVol *= 0.85f; mic1.smBass *= 0.85f; mic1.smMid *= 0.85f; mic1.smTreble *= 0.85f; }
  if (currentMs - mic2.lastFrameMs > 500) { mic2.smVol *= 0.85f; mic2.smBass *= 0.85f; mic2.smMid *= 0.85f; mic2.smTreble *= 0.85f; }

  float m1Dom = 0.0f, m2Dom = 0.0f, volDelta = fabsf(v1 - v2), crossover = 0.15f; 
  if (v1 > 0.02f || v2 > 0.02f) {
    if (v1 > v2) { m1Dom = 1.0f; m2Dom = (volDelta > crossover) ? 0.0f : (1.0f - (volDelta / crossover)); }
    else { m2Dom = 1.0f; m1Dom = (volDelta > crossover) ? 0.0f : (1.0f - (volDelta / crossover)); }
  }

  for (int s = 0; s < NUM_STRIPS; s++) {
    for (int p = 0; p < SIDE_LEDS; p++) {
      uint32_t c1 = 0, c2 = 0;
      float i1 = getWaveIntensity(s, p, mic1, 1.0f, c1);
      float i2 = getWaveIntensity(s, (SIDE_LEDS - 1) - p, mic2, 0.95f, c2);

      uint32_t finalOutsideColor = addColor(scaleColor(c1, m1Dom), scaleColor(c2, m2Dom));
      if (m1Dom > 0.0f && m2Dom > 0.0f && i1 > 0.15f && i2 > 0.15f) {
        uint32_t h = (uint32_t)(p * 16777619UL + s * 31UL + (uint32_t)(flow * 2.0f));
        if (((h >> 20) & 0xFF) > 160) { uint8_t f = (uint8_t)((i1 + i2) * 0.5f * 255.0f); finalOutsideColor = addColor(finalOutsideColor, rgb(f, f, f)); }
      }
      leds.setPixel(ledIndex(s, p), scaleColor(finalOutsideColor, OUTSIDE_BRIGHTNESS));
      leds.setPixel(ledIndex(s, SIDE_LEDS + (SIDE_LEDS - 1 - p)), scaleColor(finalOutsideColor, INSIDE_BRIGHTNESS));
    }
  }
}

void printDualPacketDebug() {
  if (!VERBOSE) return; static uint32_t lastDebug = 0; uint32_t now = millis();
  if (now - lastDebug < PACKET_DEBUG_INTERVAL_MS) return; lastDebug = now;
  Serial.print("[M1 dB]: "); Serial.print(20.0f * log10f(max(0.01f, mic1.smVol / 255.0f)), 1);
  Serial.print(" | [M2 dB]: "); Serial.print(20.0f * log10f(max(0.01f, mic2.smVol / 255.0f)), 1);
  Serial.print(" | Target Scream Raw: "); Serial.println(rawScreamLimit);
}

void applySmoothingFilters(MicEngine &mic) {
  float speedNorm = 60.0f / (float)TARGET_FPS;
  float vSmooth = constrain(1.0f - (0.18f * speedNorm), 0.1f, 0.95f), bmtSmooth = constrain(1.0f - (0.22f * speedNorm), 0.1f, 0.95f), hSmooth = constrain(1.0f - (0.15f * speedNorm), 0.1f, 0.95f);
  mic.smVol = mic.smVol * vSmooth + mic.audio.volume * (1.0f - vSmooth);
  mic.smBass = mic.smBass * bmtSmooth + mic.audio.bass * (1.0f - bmtSmooth);
  mic.smMid = mic.smMid * bmtSmooth + mic.audio.mid * (1.0f - bmtSmooth);
  mic.smTreble = mic.smTreble * bmtSmooth + mic.audio.treble * (1.0f - bmtSmooth);
  mic.smNoteHue = mic.smNoteHue * hSmooth + mic.audio.noteHue * (1.0f - hSmooth);

  if (mic.audio.volume == 0) { mic.smVol *= 0.50f; mic.smBass *= 0.50f; mic.smMid *= 0.50f; mic.smTreble *= 0.50f; if (mic.smVol < 3.0f) { mic.smVol = 0.0f; mic.smBass = 0.0f; mic.smMid = 0.0f; mic.smTreble = 0.0f; } }
}

void setup() {
  pinMode(LED_DEBUG, OUTPUT); digitalWrite(LED_DEBUG, HIGH);
  Serial.begin(115200); delay(500);
  pinMode(MIC1_SERIAL_RTS, OUTPUT); digitalWrite(MIC1_SERIAL_RTS, LOW);
  pinMode(MIC2_SERIAL_RTS, OUTPUT); digitalWrite(MIC2_SERIAL_RTS, LOW);
  MIC1_SERIAL.begin(UART_BAUD); MIC2_SERIAL.begin(UART_BAUD);
  leds.begin(); leds.show();
  randomSeed(analogRead(38) + analogRead(39));
  
  // --- CALCULATE LOGARITHMIC TRANSLATIONS ONCE ---
  rawScreamLimit   = (uint8_t)constrain(powf(10.0f, SCREAM_DB_LIMIT / 20.0f) * 255.0f, 0.0f, 255.0f);
  rawWhiteHotLimit = (uint8_t)constrain(powf(10.0f, SCREAM_WHITE_HOT_DB / 20.0f) * 255.0f, 0.0f, 255.0f);
  
  startupLedTest();
  lastFrameRenderMs = millis(); lastAudioActivityMs = millis(); 
}

void loop() {
  while (readAudioFrame(MIC1_SERIAL, mic1)) {} while (readAudioFrame(MIC2_SERIAL, mic2)) {}
  uint32_t currentMs = millis(); if (mic1.audio.volume > 0 || mic2.audio.volume > 0) lastAudioActivityMs = currentMs;

  if (currentMs - lastFrameRenderMs >= FRAME_PERIOD_MS) {
    lastFrameRenderMs = currentMs;
    applySmoothingFilters(mic1); applySmoothingFilters(mic2);

    bool wantIdle = (currentMs - lastAudioActivityMs >= IDLE_TIMEOUT_MS);
    float alphaStep = 1.0f / (TARGET_FPS * (wantIdle ? 1.0f : 0.5f));
    if (wantIdle) { idleFade += alphaStep; if (idleFade > 1.0f) idleFade = 1.0f; } 
    else { idleFade -= alphaStep; if (idleFade < 0.0f) idleFade = 0.0f; }

    renderAudioTunnel();
    if (idleFade > 0.0f) applyIdleAnimation(idleFade);
    leds.show();
  }
  printDualPacketDebug();
}