
/*
  Teensy 4.1 LED renderer (Dual Mic Audio Tunnel - Mirrored Double-Sided Tube)
  
  ANTI-TRANSIENT UPDATE:
  - Added Vocal Confirmation Windowing to ignore short clicks/door slams.
  - Added asymmetric slew rate limits to prevent immediate flash spikes on sudden room noise.
*/

#include <Arduino.h>
#include <OctoWS2811.h>

// ---------- Performance & Timing Configuration ----------
static constexpr uint32_t TARGET_FPS = 55;        
static constexpr uint32_t FRAME_PERIOD_MS = 1000 / TARGET_FPS;

// ---------- Dual Brightness Configuration ----------
static constexpr float OUTSIDE_BRIGHTNESS = 1.0f; 
static constexpr float INSIDE_BRIGHTNESS  = 0.5f; 

// ---------- Dual UART Configuration ----------
#define MIC1_SERIAL Serial7   
#define MIC2_SERIAL Serial1   

#define MIC1_SERIAL_RTS 27
#define MIC2_SERIAL_RTS 2

#define LED_DEBUG 13

// ---------- Debug ----------
#define VERBOSE true
#define DEBUG_INTERVAL_MS 200
#define PACKET_DEBUG_INTERVAL_MS 200

// ---------- LEDs Physical Geometry ----------
static constexpr int NUM_STRIPS = 12;
static constexpr int LEDS_PER_STRIP = 384;            
static constexpr int SIDE_LEDS = LEDS_PER_STRIP / 2;  // 192 LEDs per side (2 meters)
static constexpr int NUM_LEDS = NUM_STRIPS * LEDS_PER_STRIP;

static constexpr uint32_t UART_BAUD = 460800;

// Teensy LED pins
byte pinList[NUM_STRIPS] = {
  21, 18, 19, 20, 36, 38, 33, 37, 34, 35, 40, 39  
};
const int bytesPerLED = 3;

DMAMEM int displayMemory[LEDS_PER_STRIP * NUM_STRIPS * bytesPerLED / 4];
int drawingMemory[LEDS_PER_STRIP * NUM_STRIPS * bytesPerLED / 4];

const int config = WS2811_GRB | WS2811_800kHz;

OctoWS2811 leds(
  LEDS_PER_STRIP, displayMemory, drawingMemory, config, NUM_STRIPS, pinList
);

struct AudioFrame {
  uint8_t volume = 0;
  uint8_t bass = 0;
  uint8_t mid = 0;
  uint8_t treble = 0;
  uint8_t noteHue = 0;
  uint8_t beat = 0;
};

struct MicEngine {
  AudioFrame audio;
  uint32_t lastFrameMs = 0;
  uint32_t frameCounter = 0;
  uint32_t badChecksumCounter = 0;
  uint32_t rawByteCounter = 0;

  uint8_t state = 0;
  uint8_t buf[8];
  uint8_t idx = 0;

  float smVol = 0;
  float smBass = 0;
  float smMid = 0;
  float smTreble = 0;
  float smNoteHue = 0;
  float beatPulse = 0;

  // --- Anti-Transient Tracker Variables ---
  uint16_t sustainedFrames = 0; 
  float targetVolProcessed = 0;
};

MicEngine mic1;
MicEngine mic2;

// --- Anti-Transient Config Tuning Parameters ---
static constexpr uint16_t DEBOUNCE_CONFIRM_FRAMES = 3; // Must be loud for ~50ms to count as vocal
static constexpr float MAX_VOL_ATTACK_STEP = 35.0f;     // Maximum volume delta increase allowed per frame

enum RenderMode {
  TUNNEL_MODE,    
  CLUSTER_MODE    
};

RenderMode currentMode = TUNNEL_MODE;
RenderMode pendingMode = TUNNEL_MODE; 
bool modeChangePending = false;

uint32_t lastModeSwitchMs = 0;
uint32_t modeDurationMs = 5000;  

float flow = 0;
float clockTrackAngle = 0.0f;     
float targetClockTrack = 0.0f;     
uint32_t lastRandomHopMs = 0;      
uint32_t lastFrameRenderMs = 0;

// ---------- Helpers ----------
uint8_t checksumFrame(uint8_t *f, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; i++) c ^= f[i];
  return c;
}

uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

int ledIndex(int strip, int pixel) {
  return strip * LEDS_PER_STRIP + pixel;
}

void fillAll(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) leds.setPixel(i, color);
  leds.show();
}

void startupLedTest() {
  fillAll(rgb(120, 0, 0)); delay(500);
  fillAll(rgb(0, 120, 0)); delay(500);
  fillAll(rgb(0, 0, 120)); delay(500);
  fillAll(rgb(0, 0, 0));   delay(100);
}

bool readAudioFrame(HardwareSerial &serialPort, MicEngine &mic) {
  while (serialPort.available()) {
    uint8_t b = serialPort.read();
    mic.rawByteCounter++;
    switch (mic.state) {
      case 0: if (b == 0xAA) mic.state = 1; break;
      case 1:
        if (b == 0x55) {
          mic.buf[0] = 0xAA; mic.buf[1] = 0x55; mic.idx = 2; mic.state = 2;
        } else {
          mic.state = 0;
        }
        break;
      case 2:
        mic.buf[mic.idx++] = b;
        if (mic.idx >= 8) mic.state = 3;
        break;
      case 3: {
        uint8_t expected = checksumFrame(mic.buf, 8);
        if (b == expected) {
          mic.audio.volume  = mic.buf[2]; mic.audio.bass    = mic.buf[3];
          mic.audio.mid     = mic.buf[4]; mic.audio.treble  = mic.buf[5];
          mic.audio.noteHue = mic.buf[6]; mic.audio.beat    = mic.buf[7];
          mic.lastFrameMs = millis(); mic.frameCounter++; mic.state = 0;
          return true;
        } else {
          mic.badChecksumCounter++; mic.state = 0;
        }
        break;
      }
    }
  }
  return false;
}

uint32_t hsvToRgb(uint8_t h, uint8_t s, uint8_t v) {
  uint8_t region = h / 43;
  uint8_t remainder = (h - region * 43) * 6;
  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
  switch (region) {
    case 0:  return rgb(v, t, p); case 1:  return rgb(q, v, p);
    case 2:  return rgb(p, v, t); case 3:  return rgb(p, q, v);
    case 4:  return rgb(t, p, v); default: return rgb(v, p, q);
  }
}

uint32_t scaleColor(uint32_t c, float scale) {
  scale = constrain(scale, 0.0f, 1.0f);
  uint8_t r = ((c >> 16) & 0xFF) * scale;
  uint8_t g = ((c >> 8) & 0xFF) * scale;
  uint8_t b = (c & 0xFF) * scale;
  return rgb(r, g, b);
}

uint32_t addColor(uint32_t a, uint32_t b) {
  uint16_t ar = (a >> 16) & 0xFF; uint16_t ag = (a >> 8) & 0xFF; uint16_t ab = a & 0xFF;
  uint16_t br = (b >> 16) & 0xFF; uint16_t bg = (b >> 8) & 0xFF; uint16_t bb = b & 0xFF;
  return rgb(min(255, ar + br), min(255, ag + bg), min(255, ab + bb));
}

float getWaveIntensity(int strip, int targetPixel, MicEngine &mic, float speedModifier, uint32_t &outColor) {
  float vol = mic.smVol / 255.0f;
  if (vol < 0.03f) { outColor = 0; return 0.0f; }

  float bass = mic.smBass / 255.0f;
  float mid = mic.smMid / 255.0f;
  float treble = mic.smTreble / 255.0f;

  float x = (float)targetPixel;
  float stripOffset = (float)strip * (3.5f + (treble * 4.0f));
  float phase = (flow * speedModifier) - (x * 0.45f) + stripOffset;

  float fireWaves = sinf(phase * 0.15f) * cosf(phase * 0.08f + strip);
  fireWaves = powf((fireWaves * 0.5f) + 0.5f, 2.0f);     

  float beatSamplePoint = (flow * 2.5f) - (x * 0.60f) + (strip * 1.2f);
  float beatWave = max(0.0f, sinf(beatSamplePoint * 0.1f)) * mic.beatPulse * 1.5f;

  float structuralCooling = constrain(1.0f - (x / (float)SIDE_LEDS), 0.0f, 1.0f);

  uint32_t hash = (uint32_t)(targetPixel * 1103515245UL + strip * 76543UL + (uint32_t)(flow * 0.5f));
  float sparkle = (((hash >> 24) & 0xFF) / 255.0f) > (0.988f - treble * 0.05f) ? treble * structuralCooling * 0.8f : 0.0f;

  float intensity = (fireWaves * (0.35f + mid * 0.5f) + beatWave) * vol * 1.5f * structuralCooling;
  intensity = powf(constrain(intensity + sparkle, 0.0f, 1.0f), 1.3f);

  if (intensity < 0.01f) { outColor = 0; return 0.0f; }

  float totalEnergy = bass + mid + treble;
  uint8_t targetHue = (uint8_t)mic.smNoteHue;
  if (totalEnergy > 0.05f) {
    float calculatedHue = (bass * 0.0f + mid * 80.0f + treble * 175.0f) / totalEnergy;
    targetHue = (uint8_t)(targetHue * 0.4f + calculatedHue * 0.6f);
  }

  outColor = hsvToRgb(targetHue + (strip * 2), 255 - (bass * 30), (uint8_t)(intensity * 255.0f));

  if (x < 24.0f) {
    float injectionGlow = (1.0f - (x / 24.0f)) * (bass * 0.6f + vol * 0.4f);
    outColor = addColor(outColor, hsvToRgb(targetHue - 8, 255, (uint8_t)(injectionGlow * 150)));
  }

  return intensity;
}

void renderAudioTunnel() {
  float v1 = mic1.smVol / 255.0f;
  float v2 = mic2.smVol / 255.0f;
  float volAvg = (v1 + v2) * 0.5f;
  
  float speedNormalization = 60.0f / (float)TARGET_FPS;
  
  flow += (1.2f + ((mic1.smMid + mic2.smMid) / 510.0f) * 4.0f + ((mic1.smBass + mic2.smBass) / 510.0f) * 2.0f) * speedNormalization;
  if (flow > 200000.0f) flow = 0;

  uint32_t currentMs = millis();

  bool beatDetected = (mic1.audio.beat > 0 || mic2.audio.beat > 0);
  bool quietTimeout = (currentMs - lastRandomHopMs > 1200);

  if (beatDetected || quietTimeout) {
    targetClockTrack = (float)random(0, NUM_STRIPS);
    lastRandomHopMs = currentMs;
    if (volAvg < 0.04f && beatDetected) clockTrackAngle = targetClockTrack;
  }

  float approachDiff = targetClockTrack - clockTrackAngle;
  if (approachDiff > 6.0f) approachDiff -= 12.0f;
  if (approachDiff < -6.0f) approachDiff += 12.0f;
  clockTrackAngle += approachDiff * (0.18f * speedNormalization);

  if (clockTrackAngle >= 12.0f) clockTrackAngle -= 12.0f;
  if (clockTrackAngle < 0.0f) clockTrackAngle += 12.0f;

  float decayFactor = powf(0.85f, speedNormalization);
  mic1.beatPulse *= decayFactor; if (mic1.audio.beat > 0) mic1.beatPulse = 1.0f;
  mic2.beatPulse *= decayFactor; if (mic2.audio.beat > 0) mic2.beatPulse = 1.0f;

  if (currentMs - mic1.lastFrameMs > 500) {
    mic1.smVol *= 0.85f; mic1.smBass *= 0.85f; mic1.smMid *= 0.85f; mic1.smTreble *= 0.85f;
  }
  if (currentMs - mic2.lastFrameMs > 500) {
    mic2.smVol *= 0.85f; mic2.smBass *= 0.85f; mic2.smMid *= 0.85f; mic2.smTreble *= 0.85f;
  }

  // --- HARD ACOUSTIC DOMINANCE & DIRECTION ENGINE ---
  float mic1Dominance = 0.0f;
  float mic2Dominance = 0.0f;
  
  float volumeDelta = fabsf(v1 - v2);
  float crossoverThreshold = 0.15f; 

  if (v1 > 0.02f || v2 > 0.02f) {
    if (v1 > v2) {
      mic1Dominance = 1.0f;
      if (volumeDelta > crossoverThreshold) {
        mic2Dominance = 0.0f;
      } else {
        mic2Dominance = 1.0f - (volumeDelta / crossoverThreshold);
      }
    } else {
      mic2Dominance = 1.0f;
      if (volumeDelta > crossoverThreshold) {
        mic1Dominance = 0.0f;
      } else {
        mic1Dominance = 1.0f - (volumeDelta / crossoverThreshold);
      }
    }
  }

  // ---------- LED Renderer Pass ----------
  for (int s = 0; s < NUM_STRIPS; s++) {
    float windowFilter = 1.0f;
    if (currentMode == CLUSTER_MODE) {
      float angularDiff = fabsf((float)s - clockTrackAngle);
      if (angularDiff > 6.0f) angularDiff = 12.0f - angularDiff;
      float allowedWidth = 1.2f + (((mic1.smTreble + mic2.smTreble) / 510.0f) * 0.8f);
      if (angularDiff > allowedWidth) {
        for (int p = 0; p < LEDS_PER_STRIP; p++) leds.setPixel(ledIndex(s, p), 0);
        continue;
      }
      windowFilter = constrain(powf(1.0f - (angularDiff / allowedWidth), 1.4f), 0.0f, 1.0f);
    }

    for (int p = 0; p < SIDE_LEDS; p++) {
      uint32_t color1 = 0;
      uint32_t color2 = 0;

      float intensity1 = getWaveIntensity(s, p, mic1, 1.0f, color1);
      int m2Distance = (SIDE_LEDS - 1) - p; 
      float intensity2 = getWaveIntensity(s, m2Distance, mic2, 0.95f, color2);

      color1 = scaleColor(color1, mic1Dominance * windowFilter);
      color2 = scaleColor(color2, mic2Dominance * windowFilter);

      uint32_t finalOutsideColor = addColor(color1, color2);

      if (mic1Dominance > 0.0f && mic2Dominance > 0.0f && intensity1 > 0.15f && intensity2 > 0.15f) {
        float collisionForce = (intensity1 + intensity2) * 0.5f;
        uint32_t hash = (uint32_t)(p * 16777619UL + s * 31UL + (uint32_t)(flow * 2.0f));
        if (((hash >> 20) & 0xFF) > 160) { 
          uint8_t flashVal = (uint8_t)(collisionForce * 255.0f);
          uint32_t whiteFlash = rgb(flashVal, flashVal, flashVal);
          finalOutsideColor = addColor(finalOutsideColor, whiteFlash);
        }
      }

      leds.setPixel(ledIndex(s, p), scaleColor(finalOutsideColor, OUTSIDE_BRIGHTNESS));
      int insidePixel = SIDE_LEDS + (SIDE_LEDS - 1 - p);
      leds.setPixel(ledIndex(s, insidePixel), scaleColor(finalOutsideColor, INSIDE_BRIGHTNESS));
    }
  }

  leds.show();
}

void printDualPacketDebug() {
  if (!VERBOSE) return;
  static uint32_t lastPacketDebugMs = 0;
  uint32_t now = millis();
  if (now - lastPacketDebugMs < PACKET_DEBUG_INTERVAL_MS) return;
  lastPacketDebugMs = now;

  Serial.print("[M1] Vol:"); Serial.print(mic1.audio.volume);
  Serial.print(" [M2] Vol:"); Serial.print(mic2.audio.volume);
  Serial.print(" | Active Mode: ");
  Serial.print(currentMode == TUNNEL_MODE ? "TUNNEL" : "CLUSTER");
  Serial.print(" | Track Center: ");
  Serial.println(clockTrackAngle, 1);
}

// ---------- Smart Anti-Transient Smoothing Engine ----------
void applySmoothingFilters(MicEngine &mic) {
  float speedNormalization = 60.0f / (float)TARGET_FPS;
  
  // 1. VOCAL CONFIRMATION GATE (Debouncer)
  // If incoming raw volume crosses an operational threshold, test it over consecutive frames.
  if (mic.audio.volume > 35) {
    mic.sustainedFrames++;
  } else {
    if (mic.sustainedFrames > 0) mic.sustainedFrames--;
  }

  // Assign internal processed target based on whether noise has outlasted transient limits
  if (mic.sustainedFrames >= DEBOUNCE_CONFIRM_FRAMES) {
    mic.targetVolProcessed = (float)mic.audio.volume;
  } else {
    mic.targetVolProcessed = 0.0f; // Discard short clicks completely
  }

  // 2. ASYMMETRIC SLEW LIMITER (Limits rate of lighting explosion)
  float volDelta = mic.targetVolProcessed - mic.smVol;
  if (volDelta > 0) {
    // Going up: Clamp max step size to suppress fast room slaps
    if (volDelta > MAX_VOL_ATTACK_STEP) volDelta = MAX_VOL_ATTACK_STEP;
    mic.smVol += volDelta;
  } else {
    // Going down: Fall away at standard responsive exponential rate
    float vSmooth = 1.0f - (0.24f * speedNormalization);
    mic.smVol = mic.smVol * vSmooth + mic.targetVolProcessed * (1.0f - vSmooth);
  }

  // Standard Exponential Filter for Frequency Sub-bands
  float bmtSmooth = 1.0f - (0.22f * speedNormalization);
  float hSmooth = 1.0f - (0.15f * speedNormalization);

  mic.smBass   = mic.smBass   * bmtSmooth + mic.audio.bass   * (1.0f - bmtSmooth);
  mic.smMid    = mic.smMid    * bmtSmooth + mic.audio.mid    * (1.0f - bmtSmooth);
  mic.smTreble = mic.smTreble * bmtSmooth + mic.audio.treble * (1.0f - bmtSmooth);
  mic.smNoteHue = mic.smNoteHue * hSmooth + mic.audio.noteHue * (1.0f - hSmooth);

  // Complete blackout handling
  if (mic.targetVolProcessed == 0.0f) {
    mic.smVol *= 0.60f; 
    mic.smBass *= 0.60f; 
    mic.smMid *= 0.60f; 
    mic.smTreble *= 0.60f;
    if (mic.smVol < 2.0f) {
      mic.smVol = 0.0f; mic.smBass = 0.0f; mic.smMid = 0.0f; mic.smTreble = 0.0f;
    }
  }
}

void setup() {
  pinMode(LED_DEBUG, OUTPUT);
  digitalWrite(LED_DEBUG, HIGH);

  Serial.begin(115200);
  delay(500);

  pinMode(MIC1_SERIAL_RTS, OUTPUT); digitalWrite(MIC1_SERIAL_RTS, LOW);
  pinMode(MIC2_SERIAL_RTS, OUTPUT); digitalWrite(MIC2_SERIAL_RTS, LOW);

  MIC1_SERIAL.begin(UART_BAUD);
  MIC2_SERIAL.begin(UART_BAUD);

  leds.begin();
  leds.show();

  randomSeed(analogRead(38) + analogRead(39));
  startupLedTest();
  
  lastFrameRenderMs = millis();
  lastModeSwitchMs = millis();
}

void loop() {
  while (readAudioFrame(MIC1_SERIAL, mic1)) {}
  while (readAudioFrame(MIC2_SERIAL, mic2)) {}

  uint32_t currentMs = millis();
  if (currentMs - lastFrameRenderMs >= FRAME_PERIOD_MS) {
    lastFrameRenderMs = currentMs;
    applySmoothingFilters(mic1);
    applySmoothingFilters(mic2);
    renderAudioTunnel();
  }

  printDualPacketDebug();
}

