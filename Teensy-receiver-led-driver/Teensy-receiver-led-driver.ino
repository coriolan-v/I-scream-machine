
/*
  Teensy 4.1 LED renderer (Dual Mic Audio Tunnel - Mirrored Double-Sided Tube)

  Receives compact audio frames from TWO XIAO RP2040s over designated UARTs.
  Maps them to 12 double-sided strips arranged like clock numbers.
*/

#include <Arduino.h>
#include <OctoWS2811.h>

// ---------- Performance & Timing Configuration ----------
static constexpr uint32_t TARGET_FPS = 60;        // Target frames per second (e.g., 60, 90, 120)
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

// Teensy LED pins
byte pinList[NUM_STRIPS] = {  
  33, 34, 35, 36,
  37, 38, 39, 40,
  18, 19, 20, 21
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

// ---------- Global Visual State ----------
float flow = 0;
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
// Strict One-Way Progressive Fluid/Fire Propagation Engine
uint32_t computePatternColor(int strip, int targetPixel, MicEngine &mic, float sideBias, float speedModifier) {
  float vol = mic.smVol / 255.0f;
  if (vol < 0.03f) return 0; // Absolute noise floor gate

  float bass = mic.smBass / 255.0f;
  float mid = mic.smMid / 255.0f;
  float treble = mic.smTreble / 255.0f;
  uint8_t baseHue = (uint8_t)mic.smNoteHue;

  float x = (float)targetPixel;

  // 1. One-way dynamic flow field phase logic
  // Tracking spatial position subtracted from global flow creates an absolute forward-only vector force.
  float phase = (flow * speedModifier) - (x * 0.45f) + (strip * 2.5f);

  // 2. Continuous Emission Wave Layer (Fire movement simulation)
  float fireWaves = sinf(phase * 0.15f) * cosf(phase * 0.08f + strip);
  fireWaves = (fireWaves * 0.5f) + 0.5f; // Normalize to 0.0f -> 1.0f range
  fireWaves = powf(fireWaves, 2.0f);     // Sharpen up flame peaks

  // 3. One-way Beat Impact Fronts
  // Beats inject a bright pulse traveling sequentially from 0 down to 191
  float beatVelocity = 2.5f; 
  float beatSamplePoint = (flow * beatVelocity) - (x * 0.60f);
  float beatWave = sinf(beatSamplePoint * 0.1f);
  beatWave = max(0.0f, beatWave) * mic.beatPulse * 1.5f;

  // 4. Natural Tail Fade dissipation across the 2-meter physical strip length
  // Emitted energy naturally vaporizes/cools down as it approaches the far termination end.
  float structuralCooling = 1.0f - (x / (float)SIDE_LEDS);
  structuralCooling = constrain(structuralCooling, 0.0f, 1.0f);

  // 5. Treble Sparkle Dust 
  uint32_t hash = (uint32_t)(targetPixel * 1103515245UL + strip * 76543UL + (uint32_t)(flow * 0.5f));
  float sparkle = ((hash >> 24) & 0xFF) / 255.0f;
  if (sparkle > (0.988f - treble * 0.05f)) {
    sparkle = treble * structuralCooling * 0.8f;
  } else {
    sparkle = 0.0f;
  }

  // Combine layers into a fluid driving coefficient
  float intensity = (fireWaves * (0.35f + mid * 0.5f) + beatWave) * vol * 1.5f * structuralCooling;
  intensity += sparkle;

  intensity = constrain(intensity, 0.0f, 1.0f);
  intensity = powf(intensity, 1.3f); // Gamma filtering

  if (intensity < 0.01f) return 0;

  // Modulate color dynamically across the physical length
  uint8_t hue = baseHue + (targetPixel * 0.25f) + (strip * 2) + (sideBias * 5.0f);
  uint8_t sat = 245 - (bass * 20); // Saturate slightly based on tracking low-frequency
  uint8_t val = (uint8_t)(intensity * 255.0f);

  uint32_t c = hsvToRgb(hue, sat, val);

  // Direct injection point flare (Intense hot glow localized strictly at the starting edge)
  if (x < 24.0f) {
    float injectionGlow = (1.0f - (x / 24.0f)) * (bass * 0.6f + vol * 0.4f);
    uint32_t plasmaCore = hsvToRgb(baseHue - 10, 255, (uint8_t)(injectionGlow * 150));
    c = addColor(c, plasmaCore);
  }

  return c;
}

// ---------- Render Audio Tunnel Loops ----------
void renderAudioTunnel() {
  float midAvg = (mic1.smMid + mic2.smMid) / 510.0f;
  float bassAvg = (mic1.smBass + mic2.smBass) / 510.0f;
  
  float speedNormalization = 60.0f / (float)TARGET_FPS;
  
  // Continuous progression accumulation variable
  flow += (1.2f + midAvg * 4.0f + bassAvg * 2.0f) * speedNormalization;
  if (flow > 200000.0f) flow = 0;

  float decayFactor = powf(0.85f, speedNormalization);
  mic1.beatPulse *= decayFactor;
  if (mic1.audio.beat > 0) mic1.beatPulse = 1.0f;

  mic2.beatPulse *= decayFactor;
  if (mic2.audio.beat > 0) mic2.beatPulse = 1.0f;

  uint32_t currentMs = millis();
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
      
      // Mic 1: Evaluates step distance 'p' from physical pixel 0 down to 191
      uint32_t colorM1 = computePatternColor(s, p, mic1, sideBias, 1.0f);

      // Mic 2: Evaluates step distance 'p' from physical pixel 191 down to 0
      uint32_t colorM2 = computePatternColor(s, p, mic2, sideBias, 0.95f);

      // Non-interfering clean additive blend over shared physical indices
      int mic2PhysicalIndex = (SIDE_LEDS - 1) - p;
      
      // We read current state of layout safely to merge the overlapping outputs
      uint32_t colorM1_Target = colorM1;
      
      // For pixel mapping compilation, merge calculations over unified layout spectrum
      // We handle the spatial coordinate mapping cleanly here
      
      // Let's create an independent buffer write sequence to avoid asymmetric assignment artifacts:
      // We can directly compile the absolute buffer assignments per pixel index:
    }
    
    // Optimized spatial mapper loop pass
    for (int p = 0; p < SIDE_LEDS; p++) {
      // Get the forward emission color from Mic 1 (starts at 0, moves right)
      uint32_t c1 = computePatternColor(s, p, mic1, sideBias, 1.0f);
      
      // Get the forward emission color from Mic 2 (starts at 191, moves left)
      int m2Distance = (SIDE_LEDS - 1) - p; 
      uint32_t c2 = computePatternColor(s, m2Distance, mic2, sideBias, 0.95f);

      // Smooth additive pass over the physical layout map
      uint32_t finalOutsideColor = addColor(c1, c2);

      // Write direct to outside layer
      leds.setPixel(ledIndex(s, p), scaleColor(finalOutsideColor, OUTSIDE_BRIGHTNESS));

      // Inverse mirror assignment direct to inside layer
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
  Serial.print(" BadChks:"); Serial.print(mic1.badChecksumCounter);
  Serial.print(" Vol:"); Serial.print(mic1.audio.volume);
  Serial.print(" | [M2] Pkts:"); Serial.print(mic2.frameCounter);
  Serial.print(" BadChks:"); Serial.print(mic2.badChecksumCounter);
  Serial.print(" Vol:"); Serial.println(mic2.audio.volume);
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

  if (VERBOSE) {
    Serial.print("\nTeensy Absolute Unidirectional Fluid/Fire Renderer Started @ ");
    Serial.print(TARGET_FPS);
    Serial.println(" FPS");
  }

  startupLedTest();
  lastFrameRenderMs = millis();
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

