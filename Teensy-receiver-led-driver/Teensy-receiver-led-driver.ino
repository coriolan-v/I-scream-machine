/*
  Teensy 4.1 LED renderer

  Receives compact audio frames from XIAO RP2040 over Serial4.

  UART wiring:
    XIAO GPIO0 / TX  -> Teensy 4.1 pin 16 / Serial4 RX
    XIAO GND         -> Teensy GND

  LED wiring:
    Teensy pin 0  -> LED strip 0 data
    Teensy pin 1  -> LED strip 1 data
    Teensy pin 2  -> LED strip 2 data
    Teensy pin 3  -> LED strip 3 data
    Teensy pin 4  -> LED strip 4 data
    Teensy pin 5  -> LED strip 5 data
    Teensy pin 6  -> LED strip 6 data
    Teensy pin 7  -> LED strip 7 data
    Teensy pin 8  -> LED strip 8 data
    Teensy pin 9  -> LED strip 9 data
    Teensy pin 10 -> LED strip 10 data
    Teensy pin 11 -> LED strip 11 data

  12 strips, 192 LEDs each.
*/

#include <Arduino.h>
#include <OctoWS2811.h>

// ---------- Debug ----------
#define VERBOSE true
#define DEBUG_INTERVAL_MS 1000
#define PACKET_DEBUG_INTERVAL_MS 500

// ---------- LEDs ----------
static constexpr int NUM_STRIPS = 12;
static constexpr int LEDS_PER_STRIP = 192;
static constexpr int NUM_LEDS = NUM_STRIPS * LEDS_PER_STRIP;

static constexpr uint32_t UART_BAUD = 460800;

// Your requested Teensy LED pins
byte pinList[NUM_STRIPS] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
};

// OctoWS2811 memory sizing using PJRC generic formula
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

// ---------- Received audio state ----------
struct AudioFrame {
  uint8_t volume = 0;
  uint8_t bass = 0;
  uint8_t mid = 0;
  uint8_t treble = 0;
  uint8_t centroid = 0;
  uint8_t beat = 0;
};

AudioFrame audio;

uint32_t lastFrameMs = 0;
uint32_t frameCounter = 0;
uint32_t badChecksumCounter = 0;
uint32_t rawByteCounter = 0;

uint32_t lastDebugMs = 0;
uint32_t lastPacketDebugMs = 0;
uint32_t lastPacketCount = 0;
uint32_t lastBadChecksumCount = 0;
uint32_t lastRawByteCount = 0;

// ---------- Visual state ----------
float smVol = 0;
float smBass = 0;
float smMid = 0;
float smTreble = 0;
float smCentroid = 0;

float beatPulse = 0;
float flow = 0;

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
  delay(700);

  if (VERBOSE) Serial.println("Startup LED test: all green");
  fillAll(rgb(0, 255, 0));
  delay(700);

  if (VERBOSE) Serial.println("Startup LED test: all blue");
  fillAll(rgb(0, 0, 255));
  delay(700);

  if (VERBOSE) Serial.println("Startup LED test: all white, low brightness");
  fillAll(rgb(40, 40, 40));
  delay(700);

  if (VERBOSE) Serial.println("Startup LED test: per-strip test");

  fillAll(rgb(0, 0, 0));
  delay(200);

  for (int s = 0; s < NUM_STRIPS; s++) {
    fillAll(rgb(0, 0, 0));

    uint32_t c;

    if (s % 3 == 0) {
      c = rgb(255, 0, 0);
    } else if (s % 3 == 1) {
      c = rgb(0, 255, 0);
    } else {
      c = rgb(0, 0, 255);
    }

    for (int p = 0; p < LEDS_PER_STRIP; p++) {
      leds.setPixel(ledIndex(s, p), c);
    }

    leds.show();

    if (VERBOSE) {
      Serial.print("Testing strip ");
      Serial.print(s);
      Serial.print(" on Teensy pin ");
      Serial.println(pinList[s]);
    }

    delay(300);
  }

  if (VERBOSE) Serial.println("Startup LED test: pixel chase across all strips");

  fillAll(rgb(0, 0, 0));

  for (int p = 0; p < LEDS_PER_STRIP; p += 4) {
    fillAll(rgb(0, 0, 0));

    for (int s = 0; s < NUM_STRIPS; s++) {
      leds.setPixel(ledIndex(s, p), rgb(255, 255, 255));

      if (p + 1 < LEDS_PER_STRIP) {
        leds.setPixel(ledIndex(s, p + 1), rgb(80, 80, 80));
      }

      if (p + 2 < LEDS_PER_STRIP) {
        leds.setPixel(ledIndex(s, p + 2), rgb(30, 30, 30));
      }
    }

    leds.show();
    delay(25);
  }

  fillAll(rgb(0, 0, 0));

  if (VERBOSE) Serial.println("Startup LED test complete. Entering audio mode.");
}

// ---------- UART receive ----------
bool readAudioFrame() {
  static uint8_t state = 0;
  static uint8_t buf[8];
  static uint8_t idx = 0;

  while (Serial4.available()) {
    uint8_t b = Serial4.read();
    rawByteCounter++;

    switch (state) {
      case 0:
        if (b == 0xAA) {
          state = 1;
        }
        break;

      case 1:
        if (b == 0x55) {
          buf[0] = 0xAA;
          buf[1] = 0x55;
          idx = 2;
          state = 2;
        } else {
          state = 0;
        }
        break;

      case 2:
        buf[idx++] = b;

        if (idx >= 8) {
          state = 3;
        }

        break;

      case 3: {
        uint8_t expected = checksumFrame(buf, 8);

        if (b == expected) {
          audio.volume = buf[2];
          audio.bass = buf[3];
          audio.mid = buf[4];
          audio.treble = buf[5];
          audio.centroid = buf[6];
          audio.beat = buf[7];

          lastFrameMs = millis();
          frameCounter++;

          state = 0;
          return true;
        } else {
          badChecksumCounter++;
          state = 0;
        }

        break;
      }
    }
  }

  return false;
}

// ---------- Colour ----------
uint32_t hsvToRgb(uint8_t h, uint8_t s, uint8_t v) {
  uint8_t region = h / 43;
  uint8_t remainder = (h - region * 43) * 6;

  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  switch (region) {
    case 0:
      return rgb(v, t, p);

    case 1:
      return rgb(q, v, p);

    case 2:
      return rgb(p, v, t);

    case 3:
      return rgb(p, q, v);

    case 4:
      return rgb(t, p, v);

    default:
      return rgb(v, p, q);
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
  uint16_t ar = (a >> 16) & 0xFF;
  uint16_t ag = (a >> 8) & 0xFF;
  uint16_t ab = a & 0xFF;

  uint16_t br = (b >> 16) & 0xFF;
  uint16_t bg = (b >> 8) & 0xFF;
  uint16_t bb = b & 0xFF;

  return rgb(
    min(255, ar + br),
    min(255, ag + bg),
    min(255, ab + bb)
  );
}

// ---------- Debug ----------
void printPacketDebug() {
  if (!VERBOSE) return;

  uint32_t now = millis();

  if (now - lastPacketDebugMs < PACKET_DEBUG_INTERVAL_MS) {
    return;
  }

  float dt;

  if (lastPacketDebugMs == 0) {
    dt = PACKET_DEBUG_INTERVAL_MS / 1000.0f;
  } else {
    dt = (now - lastPacketDebugMs) / 1000.0f;
  }

  if (dt <= 0) {
    dt = 1.0f;
  }

  uint32_t packetsNow = frameCounter;
  uint32_t badNow = badChecksumCounter;
  uint32_t bytesNow = rawByteCounter;

  uint32_t packetsDelta = packetsNow - lastPacketCount;
  uint32_t badDelta = badNow - lastBadChecksumCount;
  uint32_t bytesDelta = bytesNow - lastRawByteCount;

  lastPacketDebugMs = now;
  lastPacketCount = packetsNow;
  lastBadChecksumCount = badNow;
  lastRawByteCount = bytesNow;

  Serial.print("[UART] packets=");
  Serial.print(packetsNow);

  Serial.print(" +");
  Serial.print(packetsDelta);

  Serial.print(" pkt/s=");
  Serial.print(packetsDelta / dt, 1);

  Serial.print(" bytes=");
  Serial.print(bytesNow);

  Serial.print(" +");
  Serial.print(bytesDelta);

  Serial.print(" B/s=");
  Serial.print(bytesDelta / dt, 1);

  Serial.print(" badChecksum=");
  Serial.print(badNow);

  Serial.print(" +");
  Serial.print(badDelta);

  Serial.print(" ageMs=");

  if (lastFrameMs == 0) {
    Serial.print("NO_FRAME_YET");
  } else {
    Serial.print(now - lastFrameMs);
  }

  Serial.print(" Serial4.available=");
  Serial.print(Serial4.available());

  Serial.print(" last: vol=");
  Serial.print(audio.volume);

  Serial.print(" bass=");
  Serial.print(audio.bass);

  Serial.print(" mid=");
  Serial.print(audio.mid);

  Serial.print(" treble=");
  Serial.print(audio.treble);

  Serial.print(" centroid=");
  Serial.print(audio.centroid);

  Serial.print(" beat=");
  Serial.println(audio.beat);
}

void printVisualDebug() {
  if (!VERBOSE) return;

  uint32_t now = millis();

  if (now - lastDebugMs < DEBUG_INTERVAL_MS) {
    return;
  }

  lastDebugMs = now;

  Serial.print("[VIS] smVol=");
  Serial.print(smVol, 1);

  Serial.print(" smBass=");
  Serial.print(smBass, 1);

  Serial.print(" smMid=");
  Serial.print(smMid, 1);

  Serial.print(" smTreble=");
  Serial.print(smTreble, 1);

  Serial.print(" smCentroid=");
  Serial.print(smCentroid, 1);

  Serial.print(" beatPulse=");
  Serial.print(beatPulse, 2);

  Serial.print(" flow=");
  Serial.println(flow, 1);
}

// ---------- Main visual ----------
void renderAudioTunnel() {
  float vol = smVol / 255.0f;
  float bass = smBass / 255.0f;
  float mid = smMid / 255.0f;
  float treble = smTreble / 255.0f;
  float centroid = smCentroid / 255.0f;

  // Frequency -> hue
  uint8_t baseHue = (uint8_t)(centroid * 185.0f + 5.0f);

  // Loudness controls how far the pattern grows along the strip
  float activeLen = 8.0f + powf(vol, 0.65f) * (LEDS_PER_STRIP - 8);

  // Bass makes larger wave packets
  float waveWidth = 8.0f + bass * 30.0f;

  // Treble adds sparkle
  float sparkleAmount = treble;

  // Flow speed
  flow += 0.35f + mid * 2.3f + bass * 0.8f;

  if (flow > 100000.0f) {
    flow = 0;
  }

  // Beat pulse
  beatPulse *= 0.88f;

  if (audio.beat > 0) {
    beatPulse = 1.0f;
  }

  // Fade if UART data stops
  if (millis() - lastFrameMs > 500) {
    smVol *= 0.85f;
    smBass *= 0.85f;
    smMid *= 0.85f;
    smTreble *= 0.85f;
  }

  for (int s = 0; s < NUM_STRIPS; s++) {
    float stripPhase = (float)s / NUM_STRIPS;
    float sideBias = sinf(stripPhase * TWO_PI + flow * 0.015f) * 0.5f + 0.5f;

    for (int p = 0; p < LEDS_PER_STRIP; p++) {
      float x = (float)p;

      // Main growing bar
      float edge = activeLen - x;
      float bar = constrain(edge / 18.0f, 0.0f, 1.0f);

      // Moving wave
      float wavePos = fmodf(flow + s * 13.0f, LEDS_PER_STRIP + waveWidth * 2.0f) - waveWidth;
      float d = fabsf(x - wavePos);
      float wave = max(0.0f, 1.0f - d / waveWidth);
      wave = wave * wave;

      // Reverse wave
      float wavePos2 = LEDS_PER_STRIP - wavePos;
      float d2 = fabsf(x - wavePos2);
      float wave2 = max(0.0f, 1.0f - d2 / (waveWidth * 1.4f));
      wave2 = wave2 * wave2 * 0.6f;

      // Beat pulse front
      float beatPos = (1.0f - beatPulse) * LEDS_PER_STRIP;
      float beatD = fabsf(x - beatPos);
      float beatGlow = max(0.0f, 1.0f - beatD / 20.0f) * beatPulse;

      // Deterministic pseudo-random sparkle
      uint32_t hash = (uint32_t)(p * 1103515245UL + s * 12345UL + (uint32_t)(flow * 17));
      float sparkle = ((hash >> 24) & 0xFF) / 255.0f;

      if (sparkle > (0.985f - sparkleAmount * 0.06f)) {
        sparkle = sparkleAmount;
      } else {
        sparkle = 0.0f;
      }

      float intensity =
        bar * 0.70f +
        wave * (0.25f + mid * 0.45f) +
        wave2 * 0.25f +
        beatGlow * 1.2f +
        sparkle * 0.8f;

      // True black in silence.
      // Increase this to 0.06 if you still see tiny noise.
      if (vol < 0.04f) {
        intensity = 0.0f;
      }

      intensity = constrain(intensity, 0.0f, 1.0f);
      intensity = powf(intensity, 1.35f);

      uint8_t hue = baseHue + s * 5 + p * 0.10f + sideBias * 18.0f;
      uint8_t sat = 230;
      uint8_t val = (uint8_t)(intensity * 255.0f);

      uint32_t c = hsvToRgb(hue, sat, val);

      // Warm bass body
      if (bass > 0.15f && p < activeLen * 0.55f && vol >= 0.04f) {
        uint32_t warm = hsvToRgb(15, 240, (uint8_t)(bass * 120));
        c = addColor(c, scaleColor(warm, 0.35f));
      }

      // White beat impact
      if (beatGlow > 0.01f && vol >= 0.04f) {
        c = addColor(c, rgb(
          (uint8_t)(beatGlow * 180),
          (uint8_t)(beatGlow * 180),
          (uint8_t)(beatGlow * 180)
        ));
      }

      leds.setPixel(ledIndex(s, p), c);
    }
  }

  leds.show();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial4.begin(UART_BAUD);

  leds.begin();
  leds.show();

  if (VERBOSE) {
    Serial.println();
    Serial.println("Teensy LED renderer started");
    Serial.println("UART:");
    Serial.println("  Receiving XIAO audio data on Serial4 RX");
    Serial.println("  Teensy 4.1 Serial4 RX = pin 16");
    Serial.print("  UART baud: ");
    Serial.println(UART_BAUD);

    Serial.println("LED setup:");
    Serial.print("  LED strips: ");
    Serial.println(NUM_STRIPS);

    Serial.print("  LEDs per strip: ");
    Serial.println(LEDS_PER_STRIP);

    Serial.print("  Total LEDs: ");
    Serial.println(NUM_LEDS);

    for (int s = 0; s < NUM_STRIPS; s++) {
      Serial.print("  Strip ");
      Serial.print(s);
      Serial.print(" -> Teensy pin ");
      Serial.println(pinList[s]);
    }
  }

  startupLedTest();

  lastPacketDebugMs = millis();
}

void loop() {
  while (readAudioFrame()) {
    // Drain pending frames
  }

  smVol = smVol * 0.82f + audio.volume * 0.18f;
  smBass = smBass * 0.78f + audio.bass * 0.22f;
  smMid = smMid * 0.78f + audio.mid * 0.22f;
  smTreble = smTreble * 0.78f + audio.treble * 0.22f;
  smCentroid = smCentroid * 0.85f + audio.centroid * 0.15f;

  // Extra hard silence clamp on Teensy side.
  if (audio.volume == 0) {
    smVol *= 0.50f;
    smBass *= 0.50f;
    smMid *= 0.50f;
    smTreble *= 0.50f;

    if (smVol < 3.0f) {
      smVol = 0.0f;
      smBass = 0.0f;
      smMid = 0.0f;
      smTreble = 0.0f;
    }
  }

  renderAudioTunnel();

  printPacketDebug();
  printVisualDebug();

  delay(16);
}