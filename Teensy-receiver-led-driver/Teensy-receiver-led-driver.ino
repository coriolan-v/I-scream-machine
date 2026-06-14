
/*
  Teensy 4.1 LED renderer (Dual Mic Audio Tunnel - High-Speed Burst Lock)

  Receives compact audio frames from TWO XIAO RP2040s over designated UARTs.
  Launches high-speed, dynamic short bursts that travel cleanly across the tube
  without bleeding or shifting back.
*/

#include <Arduino.h>
#include <OctoWS2811.h>

// ---------- Performance & Timing Configuration ----------
static constexpr uint32_t TARGET_FPS = 90;        
static constexpr uint32_t FRAME_PERIOD_MS = 1000 / TARGET_FPS;

// !!! BURST SPEED ADJUSTMENT !!!
// 1 = Crawling (1 pixel/frame). 4 = Fast Burst Packet (4 pixels/frame).
static constexpr int PROPAGATION_SPEED = 12; 

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

// ---------- LEDs Physical Geometry ----------
static constexpr int NUM_STRIPS = 12;
static constexpr int LEDS_PER_STRIP = 384;            
static constexpr int SIDE_LEDS = LEDS_PER_STRIP / 2;  // 192 LEDs on Outside
static constexpr int NUM_LEDS = NUM_STRIPS * LEDS_PER_STRIP;

static constexpr uint32_t UART_BAUD = 460800;

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

  uint8_t state = 0;
  uint8_t buf[8];
  uint8_t idx = 0;

  // Visual Smoothing
  float smVol = 0;
  float smBass = 0;
  float smMid = 0;
  float smTreble = 0;
};

MicEngine mic1;
MicEngine mic2;

// ---------- Fixed History Structures for Lock-Step Travel ----------
struct ShiftingPixelState {
  uint8_t hue = 0;
  uint8_t val = 0;
};

ShiftingPixelState mic1_history[SIDE_LEDS];
ShiftingPixelState mic2_history[SIDE_LEDS];

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
  fillAll(rgb(255, 0, 0)); delay(300);
  fillAll(rgb(0, 255, 0)); delay(300);
  fillAll(rgb(0, 0, 255)); delay(300);
  fillAll(rgb(0, 0, 0));   delay(100);
}

// ---------- Dual UART Parsers ----------
bool readAudioFrame(HardwareSerial &serialPort, MicEngine &mic) {
  while (serialPort.available()) {
    uint8_t b = serialPort.read();
    mic.rawByteCounter++;

    switch (mic.state) {
      case 0: if (b == 0xAA) mic.state = 1; break;
      case 1:
        if (b == 0x55) {
          mic.buf[0] = 0xAA; mic.buf[1] = 0x55; mic.idx = 2; mic.state = 2;
        } else { mic.state = 0; }
        break;
      case 2:
        mic.buf[mic.idx++] = b;
        if (mic.idx >= 8) mic.state = 3;
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

// ---------- Clean Pitch Category Selector ----------
uint8_t calculateSpectralHue(MicEngine &mic) {
  float b = mic.smBass;
  float m = mic.smMid;
  float t = mic.smTreble;

  if ((b + m + t) < 12.0f) return 0;

  // Sharp categorical locks to cleanly map low to high spectrum paths
  if (t >= b && t >= m && t > 18.0f) {
    return 195; // High Voices -> Blue/Violet
  } 
  else if (m >= b && m >= t && m > 15.0f) {
    return 85;  // Normal Mid Vocals -> Green/Teal
  } 
  else {
    return 0;   // Low Bass hums -> Crisp Red
  }
}

// ---------- Shift and Generate New Emitters ----------
void stepPropagationBuffers() {
  // Speed Step Shift: Pushes older items further forward to achieve true speed bounds
  for (int i = SIDE_LEDS - 1; i >= PROPAGATION_SPEED; i--) {
    mic1_history[i] = mic1_history[i - PROPAGATION_SPEED];
    mic2_history[i] = mic2_history[i - PROPAGATION_SPEED];
  }

  // Clear injection head zones
  for (int i = 0; i < PROPAGATION_SPEED; i++) {
    mic1_history[i].val = 0;
    mic2_history[i].val = 0;
  }

  // Burst Injector Mic 1
  float vol1 = mic1.smVol / 255.0f;
  if (vol1 > 0.05f) {
    uint8_t activeHue = calculateSpectralHue(mic1);
    float pulseInt = vol1 * 1.5f + (mic1.audio.beat ? 0.3f : 0.0f);
    uint8_t packedVal = (uint8_t)(constrain(pulseInt, 0.0f, 1.0f) * 255.0f);

    for (int i = 0; i < PROPAGATION_SPEED; i++) {
      mic1_history[i].hue = activeHue;
      mic1_history[i].val = packedVal;
    }
  }

  // Burst Injector Mic 2
  float vol2 = mic2.smVol / 255.0f;
  if (vol2 > 0.05f) {
    uint8_t activeHue = calculateSpectralHue(mic2);
    float pulseInt = vol2 * 1.5f + (mic2.audio.beat ? 0.3f : 0.0f);
    uint8_t packedVal = (uint8_t)(constrain(pulseInt, 0.0f, 1.0f) * 255.0f);

    for (int i = 0; i < PROPAGATION_SPEED; i++) {
      mic2_history[i].hue = activeHue;
      mic2_history[i].val = packedVal;
    }
  }
}

// ---------- Render Audio Tunnel Loops ----------
void renderAudioTunnel() {
  stepPropagationBuffers();

  for (int s = 0; s < NUM_STRIPS; s++) {
    for (int p = 0; p < SIDE_LEDS; p++) {
      
      // --- Mic 1 Processing Layer ---
      ShiftingPixelState stateM1 = mic1_history[p];
      uint32_t c1 = 0;
      if (stateM1.val > 0) {
        float structuralCooling = 1.0f - ((float)p / (float)SIDE_LEDS);
        uint8_t dynamicVal = stateM1.val * structuralCooling;
        c1 = hsvToRgb(stateM1.hue, 255, dynamicVal);
      }

      // --- Mic 2 Processing Layer ---
      int m2Distance = (SIDE_LEDS - 1) - p; 
      ShiftingPixelState stateM2 = mic2_history[m2Distance]; 
      uint32_t c2 = 0;
      if (stateM2.val > 0) {
        float structuralCooling = 1.0f - ((float)m2Distance / (float)SIDE_LEDS);
        uint8_t dynamicVal = stateM2.val * structuralCooling;
        c2 = hsvToRgb(stateM2.hue, 255, dynamicVal);
      }

      uint32_t finalOutsideColor = addColor(c1, c2);

      // Outside Loop Mapping
      leds.setPixel(ledIndex(s, p), scaleColor(finalOutsideColor, OUTSIDE_BRIGHTNESS));

      // Inside Inverted Mirror Mapping
      int insidePixel = SIDE_LEDS + (SIDE_LEDS - 1 - p);
      leds.setPixel(ledIndex(s, insidePixel), scaleColor(finalOutsideColor, INSIDE_BRIGHTNESS));
    }
  }

  leds.show();
}

void applySmoothingFilters(MicEngine &mic) {
  float speedNormalization = 90.0f / (float)TARGET_FPS;
  // High responsive filter setup cuts signals instantly when voice drops
  float vSmooth = 1.0f - (0.40f * speedNormalization); 
  float bmtSmooth = 1.0f - (0.35f * speedNormalization);

  mic.smVol = mic.smVol * constrain(vSmooth, 0.1f, 0.95f) + mic.audio.volume * (1.0f - constrain(vSmooth, 0.1f, 0.95f));
  mic.smBass = mic.smBass * constrain(bmtSmooth, 0.1f, 0.95f) + mic.audio.bass * (1.0f - constrain(bmtSmooth, 0.1f, 0.95f));
  mic.smMid = mic.smMid * constrain(bmtSmooth, 0.1f, 0.95f) + mic.audio.mid * (1.0f - constrain(bmtSmooth, 0.1f, 0.95f));
  mic.smTreble = mic.smTreble * constrain(bmtSmooth, 0.1f, 0.95f) + mic.audio.treble * (1.0f - constrain(bmtSmooth, 0.1f, 0.95f));

  if (mic.audio.volume == 0) {
    mic.smVol = 0; mic.smBass = 0; mic.smMid = 0; mic.smTreble = 0;
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

  if (VERBOSE) {
    Serial.print("\nTeensy High-Speed Dynamic Burst Tunnel Connected @ ");
    Serial.print(TARGET_FPS); Serial.println(" FPS");
  }

  startupLedTest();
  lastFrameRenderMs = millis();
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
}

