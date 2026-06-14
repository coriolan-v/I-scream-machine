/*
  Teensy 4.1 LED renderer (Dual Mic Audio Tunnel - Mirrored Double-Sided Tube)

  Receives compact audio frames from TWO XIAO RP2040s over designated UARTs.
  Maps them to 12 double-sided strips arranged like clock numbers.
  
  INTELLIGENT LATCH ENGINE:
  Alternates dynamically between Global Tunnel Mode and Randomized Cluster Mode,
  as well as jumping between clock strips, BUT locks execution changes until 
  the current energy packet clears the tube length to prevent mid-travel shearing.
*/

#include <Arduino.h>
#include <OctoWS2811.h>

// ---------- Performance & Timing Configuration ----------
static constexpr uint32_t TARGET_FPS = 90;        // Target frames per second (e.g., 60, 90, 120)
static constexpr uint32_t FRAME_PERIOD_MS = 1000 / TARGET_FPS;

// ---------- Dual Brightness Configuration ----------
static constexpr float OUTSIDE_BRIGHTNESS = 1.0f; // Brightness scaling for LEDs 0 -> 191 (0.0 to 1.0)
static constexpr float INSIDE_BRIGHTNESS  = 0.5f; // Brightness scaling for LEDs 192 -> 383 (0.0 to 1.0)

// ---------- Dual UART Configuration ----------
#define MIC1_SERIAL Serial7   // Choose UART for Mic 1
#define MIC2_SERIAL Serial1   // Choose UART for Mic 2

#define MIC1_SERIAL_RTS 27
#define MIC2_SERIAL_RTS 2

#define LED_DEBUG 13

// ---------- Debug ----------
#define VERBOSE true
#define DEBUG_INTERVAL_MS 1000
#define PACKET_DEBUG_INTERVAL_MS 500

// ---------- LEDs Physical Geometry ----------
static constexpr int NUM_STRIPS = 12;
static constexpr int LEDS_PER_STRIP = 384;            // Total LEDs per strip (Out + In)
static constexpr int SIDE_LEDS = LEDS_PER_STRIP / 2;  // 192 LEDs on Outside (2 meters), 192 on Inside
static constexpr int NUM_LEDS = NUM_STRIPS * LEDS_PER_STRIP;

static constexpr uint32_t UART_BAUD = 460800;

// Teensy LED pins (Preserved custom layout sequence mapping)
byte pinList[NUM_STRIPS] = {
  21, // Strip 1  (was 12)
  18, // Strip 2  (was 9)
  19, // Strip 3  (was 10)
  20, // Strip 4  (was 11)
  36, // Strip 5  (was 4)
  38, // Strip 6  (was 6)
  33, // Strip 7  (was 1)
  37, // Strip 8  (was 5)
  34, // Strip 9  (was 2)
  35, // Strip 10 (was 3)
  40, // Strip 11 (was 8)
  39  // Strip 12 (was 7)
};
const int bytesPerLED = 3;

DMAMEM int displayMemory[LEDS_PER_STRIP * NUM_STRIPS * bytesPerLED / 4];
int drawingMemory[LEDS_PER_STRIP * NUM_STRIPS * bytesPerLED / 4];

const int config = WS2811_GRB | WS2811_800kHz;

OctoWS2811 leds(
  LEDS_PER_STRIP,
  displayMemory,
  drawingMemory,
  config,
  NUM_STRIPS,
  pinList
);

// ---------- Audio Frame Struct ----------
struct AudioFrame {
  uint8_t volume = 0;
  uint8_t bass = 0;
  uint8_t mid = 0;
  uint8_t treble = 0;
  uint8_t noteHue = 0;
  uint8_t beat = 0;
};

// ---------- Dual Engine Processing State ----------
struct MicEngine {
  AudioFrame audio;
  uint32_t lastFrameMs = 0;
  uint32_t frameCounter = 0;
  uint32_t badChecksumCounter = 0;
  uint32_t rawByteCounter = 0;

  // UART parser state machine tracking
  uint8_t state = 0;
  uint8_t buf[8];
  uint8_t idx = 0;

  // Visual Smoothing
  float smVol = 0;
  float smBass = 0;
  float smMid = 0;
  float smTreble = 0;
  float smNoteHue = 0;
  float beatPulse = 0;
};

MicEngine mic1;
MicEngine mic2;

// ---------- Global Layout Architecture Modes ----------
enum RenderMode {
  TUNNEL_MODE,    // All 12 strips active
  CLUSTER_MODE    // Isolated 1, 2, or 3 strips active
};

RenderMode currentMode = TUNNEL_MODE;
RenderMode pendingMode = TUNNEL_MODE; // Latched state holder
bool modeChangePending = false;

uint32_t lastModeSwitchMs = 0;
uint32_t modeDurationMs = 5000;  // Random window boundary tracking holder (30-50s)

// ---------- Global Visual State ----------
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
  for (int i = 0; i < NUM_LEDS; i++) {
    leds.setPixel(i, color);
  }
  leds.show();
}

// ---------- Startup LED test ----------
void startupLedTest() {
  if (VERBOSE) Serial.println("Startup LED test: all red");
  fillAll(rgb(255, 0, 0));
  delay(500);

  if (VERBOSE) Serial.println("Startup LED test: all green");
  fillAll(rgb(0, 255, 0));
  delay(500);

  if (VERBOSE) Serial.println("Startup LED test: all blue");
  fillAll(rgb(0, 0, 255));
  delay(500);

  fillAll(rgb(0, 0, 0));
  delay(100);
}

// ---------- Dual UART Parsers ----------
bool readAudioFrame(HardwareSerial &serialPort, MicEngine &mic) {
  while (serialPort.available()) {
    uint8_t b = serialPort.read();
    mic.rawByteCounter++;

    switch (mic.state) {
      case 0:
        if (b == 0xAA) {
          mic.state = 1;
        }
        break;

      case 1:
        if (b == 0x55) {
          mic.buf[0] = 0xAA;
          mic.buf[1] = 0x55;
          mic.idx = 2;
          mic.state = 2;
        } else {
          mic.state = 0;
        }
        break;

      case 2:
        mic.buf[mic.idx++] = b;
        if (mic.idx >= 8) {
          mic.state = 3;
        }
        break;

      case 3: {
        uint8_t expected = checksumFrame(mic.buf, 8);

        if (b == expected) {
          mic.audio.volume  = mic.buf[2];
          mic.audio.bass    = mic.buf[3];
          mic.audio.mid     = mic.buf[4];
          mic.audio.treble  = mic.buf[5];
          mic.audio.noteHue = mic.buf[6];
          mic.audio.beat    = mic.buf[7];

          mic.lastFrameMs = millis();
          mic.frameCounter++;

          mic.state = 0;
          return true;
        } else {
          mic.badChecksumCounter++;
          mic.state = 0;
        }
        break;
      }
    }
  }
  return false;
}

// ---------- Colour Math Utilities ----------
uint32_t hsvToRgb(uint8_t h, uint8_t s, uint8_t v) {
  uint8_t region = h / 43;
  uint8_t remainder = (h - region * 43) * 6;

  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  switch (region) {
    case 0:  return rgb(v, t, p);
    case 1:  return rgb(q, v, p);
    case 2:  return rgb(p, v, t);
    case 3:  return rgb(p, q, v);
    case 4:  return rgb(t, p, v);
    default: return rgb(v, p, q);
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

// ---------- Render Math Engine ----------
uint32_t computePatternColor(int strip, int targetPixel, MicEngine &mic, float sideBias, float speedModifier) {
  float vol = mic.smVol / 255.0f;
  if (vol < 0.03f) return 0; // Absolute noise floor gate

  float bass = mic.smBass / 255.0f;
  float mid = mic.smMid / 255.0f;
  float treble = mic.smTreble / 255.0f;

  // 1. CONDITIONAL MODE OVERRIDE FILTER
  float windowFilter = 1.0f;

  if (currentMode == CLUSTER_MODE) {
    float angularDiff = fabsf((float)strip - clockTrackAngle);
    if (angularDiff > 6.0f) angularDiff = 12.0f - angularDiff;

    float allowedWidth = 1.2f + (treble * 0.8f);
    if (angularDiff > allowedWidth) return 0;

    windowFilter = 1.0f - (angularDiff / allowedWidth);
    windowFilter = constrain(powf(windowFilter, 1.4f), 0.0f, 1.0f);
  }

  // 2. Dynamic Spectrum Cross-Fader calculation
  float totalEnergy = bass + mid + treble;
  uint8_t targetHue = (uint8_t)mic.smNoteHue;
  
  if (totalEnergy > 0.05f) {
    float calculatedHue = (bass * 0.0f + mid * 80.0f + treble * 175.0f) / totalEnergy;
    targetHue = (uint8_t)(targetHue * 0.4f + calculatedHue * 0.6f);
  }

  float x = (float)targetPixel;

  // Helix Offset Multiplier
  float helixIntensity = 3.5f + (treble * 4.0f); 
  float stripOffset = (float)strip * helixIntensity;

  // One-way dynamic flow field phase logic
  float phase = (flow * speedModifier) - (x * 0.45f) + stripOffset;

  // Continuous Emission Wave Layer (Fire movement simulation)
  float fireWaves = sinf(phase * 0.15f) * cosf(phase * 0.08f + strip);
  fireWaves = (fireWaves * 0.5f) + 0.5f; 
  fireWaves = powf(fireWaves, 2.0f);     

  // One-way Beat Impact Fronts
  float beatVelocity = 2.5f; 
  float beatSamplePoint = (flow * beatVelocity) - (x * 0.60f) + (strip * 1.2f);
  float beatWave = sinf(beatSamplePoint * 0.1f);
  beatWave = max(0.0f, beatWave) * mic.beatPulse * 1.5f;

  // Natural Tail Fade dissipation across the 2-meter physical strip length
  float structuralCooling = 1.0f - (x / (float)SIDE_LEDS);
  structuralCooling = constrain(structuralCooling, 0.0f, 1.0f);

  // Treble Sparkle Dust 
  uint32_t hash = (uint32_t)(targetPixel * 1103515245UL + strip * 76543UL + (uint32_t)(flow * 0.5f));
  float sparkle = ((hash >> 24) & 0xFF) / 255.0f;
  if (sparkle > (0.988f - treble * 0.05f)) {
    sparkle = treble * structuralCooling * 0.8f;
  } else {
    sparkle = 0.0f;
  }

  // Combine layers into a fluid driving coefficient multiplied by mode architecture filter
  float intensity = (fireWaves * (0.35f + mid * 0.5f) + beatWave) * vol * 1.5f * structuralCooling * windowFilter;
  intensity += (sparkle * windowFilter);

  intensity = constrain(intensity, 0.0f, 1.0f);
  intensity = powf(intensity, 1.3f); 

  if (intensity < 0.01f) return 0;

  // Lock final hue directly into the pitch mapping loop variables
  uint8_t finalHue = targetHue + (strip * 2);
  uint8_t sat = 255 - (bass * 30); 
  uint8_t val = (uint8_t)(intensity * 255.0f);

  uint32_t c = hsvToRgb(finalHue, sat, val);

  // Direct injection point flare
  float adjustedX = x + (strip * 0.5f);
  if (adjustedX < 24.0f) {
    float injectionGlow = (1.0f - (adjustedX / 24.0f)) * (bass * 0.6f + vol * 0.4f) * windowFilter;
    uint32_t plasmaCore = hsvToRgb(targetHue - 8, 255, (uint8_t)(injectionGlow * 150));
    c = addColor(c, plasmaCore);
  }

  return c;
}

// ---------- Render Audio Tunnel Loops ----------
void renderAudioTunnel() {
  float midAvg = (mic1.smMid + mic2.smMid) / 510.0f;
  float bassAvg = (mic1.smBass + mic2.smBass) / 510.0f;
  float volAvg = (mic1.smVol + mic2.smVol) / 510.0f;
  
  float speedNormalization = 60.0f / (float)TARGET_FPS;
  
  // Continuous progression accumulation variable
  flow += (1.2f + midAvg * 4.0f + bassAvg * 2.0f) * speedNormalization;
  if (flow > 200000.0f) flow = 0;

  uint32_t currentMs = millis();

  // !!! INTERLOCKING TIMING SCHEDULER MATRIX !!!
  if (!modeChangePending && (currentMs - lastModeSwitchMs >= modeDurationMs)) {
    // Flag that we want to swap modes, but do not update currentMode yet
    modeChangePending = true;
    pendingMode = (currentMode == TUNNEL_MODE) ? CLUSTER_MODE : TUNNEL_MODE;
  }

  // Safe Latch Verification: Only execute pending swaps during audio pauses/zero-energy crossings
  if (modeChangePending && (volAvg < 0.05f)) {
    currentMode = pendingMode;
    modeChangePending = false;
    lastModeSwitchMs = currentMs;
    modeDurationMs = (uint32_t)random(30000, 50001); // 30 to 50 seconds next sequence frame
    
    if (VERBOSE) {
      if (currentMode == CLUSTER_MODE) Serial.println("\n[LATCHED] Swapped safely to CLUSTER MODE");
      else Serial.println("\n[LATCHED] Swapped safely to ALL-STRIP TUNNEL MODE");
    }
  }

  // !!! PROTECTED SPATIAL RANDOM TRACK JUMPING !!!
  bool beatDetected = (mic1.audio.beat > 0 || mic2.audio.beat > 0);
  bool quietTimeout = (currentMs - lastRandomHopMs > 1200);

  if (beatDetected || quietTimeout) {
    // Sharp beat triggers or quiet timeouts flag a new target location, but do not snap instantly mid-wave
    float proposedTarget = (float)random(0, NUM_STRIPS);
    
    // Only snap or shift targets if current pipeline calculations are quiet
    if (volAvg < 0.08f || quietTimeout) {
      targetClockTrack = proposedTarget;
      lastRandomHopMs = currentMs;
      
      // If a major drum beat hits during a quiet pocket, snap the tracker instantly
      if (beatDetected && volAvg < 0.04f) {
        clockTrackAngle = targetClockTrack;
      }
    }
  }

  // Smooth architectural approach calculations
  float approachDiff = targetClockTrack - clockTrackAngle;
  if (approachDiff > 6.0f) approachDiff -= 12.0f;
  if (approachDiff < -6.0f) approachDiff += 12.0f;

  clockTrackAngle += approachDiff * (0.18f * speedNormalization);

  if (clockTrackAngle >= 12.0f) clockTrackAngle -= 12.0f;
  if (clockTrackAngle < 0.0f) clockTrackAngle += 12.0f;

  // Handle pulse dampening
  float decayFactor = powf(0.85f, speedNormalization);
  mic1.beatPulse *= decayFactor;
  if (mic1.audio.beat > 0) mic1.beatPulse = 1.0f;

  mic2.beatPulse *= decayFactor;
  if (mic2.audio.beat > 0) mic2.beatPulse = 1.0f;

  if (currentMs - mic1.lastFrameMs > 500) {
    mic1.smVol *= 0.85f; mic1.smBass *= 0.85f; mic1.smMid *= 0.85f; mic1.smTreble *= 0.85f;
  }
  if (currentMs - mic2.lastFrameMs > 500) {
    mic2.smVol *= 0.85f; mic2.smBass *= 0.85f; mic2.smMid *= 0.85f; mic2.smTreble *= 0.85f;
  }

  for (int s = 0; s < NUM_STRIPS; s++) {
    float stripPhase = (float)s / NUM_STRIPS;
    float sideBias = sinf(stripPhase * TWO_PI + flow * 0.01f) * 0.5f + 0.5f;
    
    for (int p = 0; p < SIDE_LEDS; p++) {
      uint32_t c1 = computePatternColor(s, p, mic1, sideBias, 1.0f);
      
      int m2Distance = (SIDE_LEDS - 1) - p; 
      uint32_t c2 = computePatternColor(s, m2Distance, mic2, sideBias, 0.95f);

      uint32_t finalOutsideColor = addColor(c1, c2);

      leds.setPixel(ledIndex(s, p), scaleColor(finalOutsideColor, OUTSIDE_BRIGHTNESS));

      int insidePixel = SIDE_LEDS + (SIDE_LEDS - 1 - p);
      leds.setPixel(ledIndex(s, insidePixel), scaleColor(finalOutsideColor, INSIDE_BRIGHTNESS));
    }
  }

  leds.show();
}

// ---------- Debug Out Functions ----------
void printDualPacketDebug() {
  if (!VERBOSE) return;
  static uint32_t lastPacketDebugMs = 0;
  uint32_t now = millis();
  if (now - lastPacketDebugMs < PACKET_DEBUG_INTERVAL_MS) return;
  lastPacketDebugMs = now;

  Serial.print("[M1] Pkts:"); Serial.print(mic1.frameCounter);
  Serial.print(" Vol:"); Serial.print(mic1.audio.volume);
  Serial.print(" | [M2] Pkts:"); Serial.print(mic2.frameCounter);
  Serial.print(" Vol:"); Serial.print(mic2.audio.volume);
  
  Serial.print(" | Mode: ");
  if (modeChangePending) Serial.print("[PENDING SWAP]");
  else if (currentMode == TUNNEL_MODE) Serial.print("GLOBAL TUNNEL");
  else Serial.print("SPATIAL CLUSTER");
  
  Serial.print(" | Target Track: ");
  Serial.println(targetClockTrack, 1);
}

// ---------- Smoothing Filters Variables Matrix ----------
void applySmoothingFilters(MicEngine &mic) {
  float speedNormalization = 60.0f / (float)TARGET_FPS;
  float vSmooth = 1.0f - (0.18f * speedNormalization);
  float bmtSmooth = 1.0f - (0.22f * speedNormalization);
  float hSmooth = 1.0f - (0.15f * speedNormalization);

  mic.smVol = mic.smVol * constrain(vSmooth, 0.1f, 0.95f) + mic.audio.volume * (1.0f - constrain(vSmooth, 0.1f, 0.95f));
  mic.smBass = mic.smBass * constrain(bmtSmooth, 0.1f, 0.95f) + mic.audio.bass * (1.0f - constrain(bmtSmooth, 0.1f, 0.95f));
  mic.smMid = mic.smMid * constrain(bmtSmooth, 0.1f, 0.95f) + mic.audio.mid * (1.0f - constrain(bmtSmooth, 0.1f, 0.95f));
  mic.smTreble = mic.smTreble * constrain(bmtSmooth, 0.1f, 0.95f) + mic.audio.treble * (1.0f - constrain(bmtSmooth, 0.1f, 0.95f));
  mic.smNoteHue = mic.smNoteHue * constrain(hSmooth, 0.1f, 0.95f) + mic.audio.noteHue * (1.0f - constrain(hSmooth, 0.1f, 0.95f));

  if (mic.audio.volume == 0) {
    mic.smVol *= 0.50f; mic.smBass *= 0.50f; mic.smMid *= 0.50f; mic.smTreble *= 0.50f;
    if (mic.smVol < 3.0f) {
      mic.smVol = 0.0f; mic.smBass = 0.0f; mic.smMid = 0.0f; mic.smTreble = 0.0f;
    }
  }
}

// ---------- System Architecture Entry Setup ----------
void setup() {
  pinMode(LED_DEBUG, OUTPUT);
  digitalWrite(LED_DEBUG, HIGH);

  Serial.begin(115200);
  delay(500);

  pinMode(MIC1_SERIAL_RTS, OUTPUT);
  digitalWrite(MIC1_SERIAL_RTS, LOW);

  pinMode(MIC2_SERIAL_RTS, OUTPUT);
  digitalWrite(MIC2_SERIAL_RTS, LOW);

  MIC1_SERIAL.begin(UART_BAUD);
  MIC2_SERIAL.begin(UART_BAUD);

  leds.begin();
  leds.show();

  randomSeed(analogRead(38) + analogRead(39));

  if (VERBOSE) {
    Serial.print("\nTeensy Absolute Unidirectional Fluid/Fire Renderer Started @ ");
    Serial.print(TARGET_FPS);
    Serial.println(" FPS");
  }

  startupLedTest();
  lastFrameRenderMs = millis();
  lastModeSwitchMs = millis();
}

// ---------- Main Execution Processing Loop ----------
void loop() {
  while (readAudioFrame(MIC1_SERIAL, mic1)) {}
  while (readAudioFrame(MIC2_SERIAL, mic2)) {}

  uint32_t currentMs = millis();
  if (currentMs - lastFrameRenderMs >= FRAME_PERIOD_MS) {
    lastFrameRenderMs = currentMs;

    applySmoothingFilters(mic1);
    applySmoothingFilters(mic2);

    renderAudioTunnel();
    printDualPacketDebug();
  }
}

