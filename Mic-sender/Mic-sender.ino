/*
  XIAO RP2040 + INMP441 I2S microphone

  Improved for scream / voice detection inside a tube.

  I2S pins:
    SCK / BCLK = GPIO27
    WS / LRCLK = GPIO28
    SD / DATA  = GPIO29

  UART:
    TX = GPIO0

  Sends compact audio feature frames over UART to Teensy 4.1.

  Frame format:
    0xAA 0x55 volume bass mid treble centroid beat checksum

  Wiring:
    INMP441 SCK -> XIAO GPIO27
    INMP441 WS  -> XIAO GPIO28
    INMP441 SD  -> XIAO GPIO29
    INMP441 VDD -> 3V3
    INMP441 GND -> GND
    INMP441 L/R -> GND

    XIAO GPIO0 TX -> Teensy Serial4 RX, pin 16
    XIAO GND      -> Teensy GND
*/

#include <Arduino.h>
#include <I2S.h>
#include <arduinoFFT.h>

// ---------- Debug ----------
#define VERBOSE true
#define DEBUG_INTERVAL_MS 500

// ---------- Pins ----------
#define PIN_SCK       27
#define PIN_WS        28
#define PIN_SD        29
#define PIN_UART_TX   0

// ---------- UART ----------
#define UART_BAUD 460800

// ---------- Audio ----------
static constexpr uint32_t SAMPLE_RATE = 16000;
static constexpr uint16_t FFT_SIZE = 256;

// Lower gain = less room noise sensitivity.
// Increase to 4.0 or 5.0 if people need to scream too hard.
static constexpr float MIC_GAIN = 3.0f;

// ---------- Manual voice gate tuning ----------
// This is the most important part.
//
// If silence still lights LEDs:
//   increase MANUAL_OPEN_THRESHOLD to 0.12, 0.16, 0.20.
//
// If normal shouting does not trigger:
//   lower MANUAL_OPEN_THRESHOLD to 0.06.
//
// Good starting values for a quiet-ish room:
static constexpr float MANUAL_OPEN_THRESHOLD = 0.120f;
static constexpr float MANUAL_CLOSE_THRESHOLD = 0.080f;

// Require sustained voice before opening gate.
// Higher = rejects keyboard clicks better, but slower response.
static constexpr uint8_t GATE_ATTACK_FRAMES = 6;

// How long gate stays open after the sound drops.
// Higher = smoother trailing light.
static constexpr uint8_t GATE_RELEASE_FRAMES = 8;

// Reject events that are mostly sharp high-frequency clicks.
static constexpr float MAX_TREBLE_TO_VOICE_RATIO = 1.0f;

// How quickly volume grows once gate is open.
// Lower denominator = louder / longer LED pattern.
// Try 4.0 for more dramatic, 8.0 for stricter.
static constexpr float VOLUME_RANGE_MULT = 6.0f;

I2S i2s(INPUT);

double vReal[FFT_SIZE];
double vImag[FFT_SIZE];

ArduinoFFT<double> FFT(vReal, vImag, FFT_SIZE, SAMPLE_RATE);

// ---------- Detection state ----------
float smoothVoiceEnergy = 0.0f;
float smoothVol = 0.0f;
float prevVol = 0.0f;

bool gateOpen = false;
uint8_t gateAttackCount = 0;
uint8_t gateReleaseCount = 0;

uint32_t lastDebugMs = 0;
uint32_t frameCounter = 0;

uint8_t checksumFrame(uint8_t *f, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; i++) c ^= f[i];
  return c;
}

uint8_t clampByte(float x) {
  if (x < 0) return 0;
  if (x > 255) return 255;
  return (uint8_t)(x + 0.5f);
}

void sendFeatures(
  uint8_t vol,
  uint8_t bass,
  uint8_t mid,
  uint8_t treble,
  uint8_t centroid,
  uint8_t beat
) {
  uint8_t frame[8];

  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = vol;
  frame[3] = bass;
  frame[4] = mid;
  frame[5] = treble;
  frame[6] = centroid;
  frame[7] = beat;

  uint8_t c = checksumFrame(frame, 8);

  Serial1.write(frame, 8);
  Serial1.write(c);
}

void printDebug(
  float rms,
  float voiceEnergy,
  float trebleRatio,
  bool tooClicky,
  float centroidHz,
  uint8_t vol,
  uint8_t bass,
  uint8_t mid,
  uint8_t treble,
  uint8_t centroid,
  uint8_t beat
) {
  if (!VERBOSE) return;

  uint32_t now = millis();
  if (now - lastDebugMs < DEBUG_INTERVAL_MS) return;
  lastDebugMs = now;

  Serial.print("[XIAO] frames=");
  Serial.print(frameCounter);

  Serial.print(" rms=");
  Serial.print(rms, 5);

  Serial.print(" voiceEnergy=");
  Serial.print(voiceEnergy, 5);

  Serial.print(" smoothVoice=");
  Serial.print(smoothVoiceEnergy, 5);

  Serial.print(" openTh=");
  Serial.print(MANUAL_OPEN_THRESHOLD, 5);

  Serial.print(" closeTh=");
  Serial.print(MANUAL_CLOSE_THRESHOLD, 5);

  Serial.print(" gate=");
  Serial.print(gateOpen ? "OPEN" : "closed");

  Serial.print(" atk=");
  Serial.print(gateAttackCount);

  Serial.print(" rel=");
  Serial.print(gateReleaseCount);

  Serial.print(" trebleRatio=");
  Serial.print(trebleRatio, 2);

  Serial.print(" clicky=");
  Serial.print(tooClicky ? "YES" : "no");

  Serial.print(" vol=");
  Serial.print(vol);

  Serial.print(" bass=");
  Serial.print(bass);

  Serial.print(" mid=");
  Serial.print(mid);

  Serial.print(" treble=");
  Serial.print(treble);

  Serial.print(" centroidHz=");
  Serial.print(centroidHz, 0);

  Serial.print(" centroid=");
  Serial.print(centroid);

  Serial.print(" beat=");
  Serial.println(beat);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial1.setTX(PIN_UART_TX);
  Serial1.begin(UART_BAUD);

  // With the RP2040 Arduino-Pico I2S library, WS/LRCLK is BCLK + 1.
  // So GPIO27 BCLK and GPIO28 WS is correct.
  i2s.setBCLK(PIN_SCK);
  i2s.setDATA(PIN_SD);
  i2s.setBitsPerSample(32);
  i2s.setBuffers(6, 256);

  if (!i2s.begin(SAMPLE_RATE)) {
    Serial.println("I2S begin failed");
    while (true) delay(100);
  }

  if (VERBOSE) {
    Serial.println();
    Serial.println("XIAO RP2040 scream / voice analyser started");
    Serial.println("I2S:");
    Serial.println("  SCK / BCLK = GPIO27");
    Serial.println("  WS / LRCLK = GPIO28");
    Serial.println("  SD / DATA  = GPIO29");
    Serial.println("UART:");
    Serial.println("  TX = GPIO0");
    Serial.print("UART baud: ");
    Serial.println(UART_BAUD);
    Serial.print("Sample rate: ");
    Serial.println(SAMPLE_RATE);
    Serial.print("FFT size: ");
    Serial.println(FFT_SIZE);
    Serial.println("Gate:");
    Serial.print("  Manual open threshold: ");
    Serial.println(MANUAL_OPEN_THRESHOLD, 5);
    Serial.print("  Manual close threshold: ");
    Serial.println(MANUAL_CLOSE_THRESHOLD, 5);
  }
}

void loop() {
  int32_t left = 0;
  int32_t right = 0;

  float sumSq = 0.0f;

  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    i2s.read32(&left, &right);

    // INMP441 channel depends on L/R pin.
    // If you get silence, swap to right:
    int32_t raw = left;
    // int32_t raw = right;

    // INMP441 24-bit data is usually left-aligned in a 32-bit word.
    float sample = (float)(raw >> 8) / 8388608.0f;

    // DC blocker
    static float dc = 0;
    dc = dc * 0.995f + sample * 0.005f;
    sample -= dc;

    sample *= MIC_GAIN;

    // Hann window
    float w = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_SIZE - 1)));
    vReal[i] = sample * w;
    vImag[i] = 0.0;

    sumSq += sample * sample;
  }

  float rms = sqrtf(sumSq / FFT_SIZE);

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  float lowVoiceE = 0;
  float midVoiceE = 0;
  float highVoiceE = 0;
  float trebleClickE = 0;

  float totalVoiceE = 0;
  float weightedFreq = 0;

  for (uint16_t bin = 1; bin < FFT_SIZE / 2; bin++) {
    float freq = (float)bin * SAMPLE_RATE / FFT_SIZE;
    float mag = (float)vReal[bin];

    if (mag < 0.001f) continue;

    // Voice / scream useful range.
    // Low voice / body: 120-350 Hz
    // Main voice:       350-1600 Hz
    // Harsh scream:     1600-3500 Hz
    // Click rejection:  3500-6500 Hz
    if (freq >= 120 && freq < 350) {
      lowVoiceE += mag;
    } else if (freq >= 350 && freq < 1600) {
      midVoiceE += mag;
    } else if (freq >= 1600 && freq < 3500) {
      highVoiceE += mag;
    } else if (freq >= 3500 && freq < 6500) {
      trebleClickE += mag;
    }

    if (freq >= 120 && freq < 3500) {
      totalVoiceE += mag;
      weightedFreq += mag * freq;
    }
  }

  // Weighted voice energy.
  // Mid band is strongest for voice.
  // High band catches screaming.
  float voiceEnergy =
    lowVoiceE * 0.60f +
    midVoiceE * 1.00f +
    highVoiceE * 0.80f;

  // Reject short sharp events.
  float trebleRatio = trebleClickE / max(voiceEnergy, 0.0001f);
  bool tooClicky = trebleRatio > MAX_TREBLE_TO_VOICE_RATIO;

  // Smooth energy.
  // This helps ignore instantaneous keyboard clicks.
  smoothVoiceEnergy = smoothVoiceEnergy * 0.72f + voiceEnergy * 0.28f;

  bool wantsOpen = smoothVoiceEnergy > MANUAL_OPEN_THRESHOLD && !tooClicky;
  bool wantsClose = smoothVoiceEnergy < MANUAL_CLOSE_THRESHOLD || tooClicky;

  if (!gateOpen) {
    if (wantsOpen) {
      gateAttackCount++;

      if (gateAttackCount >= GATE_ATTACK_FRAMES) {
        gateOpen = true;
        gateReleaseCount = GATE_RELEASE_FRAMES;
      }
    } else {
      gateAttackCount = 0;
    }
  } else {
    if (wantsClose) {
      if (gateReleaseCount > 0) {
        gateReleaseCount--;
      } else {
        gateOpen = false;
        gateAttackCount = 0;
      }
    } else {
      gateReleaseCount = GATE_RELEASE_FRAMES;
    }
  }

  float centroidHz = totalVoiceE > 0 ? weightedFreq / totalVoiceE : 0;

  float normVol = 0.0f;

  if (gateOpen) {
    // Dead-zone: nothing happens until clearly above open threshold.
    float over =
      (smoothVoiceEnergy - MANUAL_OPEN_THRESHOLD) /
      max(MANUAL_OPEN_THRESHOLD * VOLUME_RANGE_MULT, 0.001f);

    normVol = constrain(over, 0.0f, 1.0f);

    // Less sensitive than before.
    // Louder voices grow the pattern.
    normVol = powf(normVol, 0.85f);
  }

  // Smooth output.
  // Fast attack, slower release.
  if (normVol > smoothVol) {
    smoothVol = smoothVol * 0.45f + normVol * 0.55f;
  } else {
    smoothVol = smoothVol * 0.82f + normVol * 0.18f;
  }

  // If gate is closed, force true black quickly.
  if (!gateOpen) {
    smoothVol *= 0.40f;
    if (smoothVol < 0.020f) {
      smoothVol = 0.0f;
    }
  }

  uint8_t bass = 0;
  uint8_t mid = 0;
  uint8_t treble = 0;
  uint8_t centroid = 0;

  if (gateOpen) {
    // Manual scaling of band brightness.
    // If colour/pattern feels too weak while volume works, increase this.
    float bandScale = 0.020f;

    bass = clampByte(constrain(lowVoiceE * bandScale, 0.0f, 1.0f) * 255);
    mid = clampByte(constrain(midVoiceE * bandScale * 0.75f, 0.0f, 1.0f) * 255);
    treble = clampByte(constrain(highVoiceE * bandScale * 0.85f, 0.0f, 1.0f) * 255);

    float cNorm = (centroidHz - 120.0f) / (3500.0f - 120.0f);
    centroid = clampByte(constrain(cNorm, 0.0f, 1.0f) * 255);
  }

  bool isBeat = gateOpen && smoothVol > 0.20f && smoothVol > prevVol * 1.35f;
  prevVol = prevVol * 0.90f + smoothVol * 0.10f;

  uint8_t vol = clampByte(smoothVol * 255.0f);
  uint8_t beat = isBeat ? 255 : 0;

  sendFeatures(
    vol,
    bass,
    mid,
    treble,
    centroid,
    beat
  );

  frameCounter++;

  printDebug(
    rms,
    voiceEnergy,
    trebleRatio,
    tooClicky,
    centroidHz,
    vol,
    bass,
    mid,
    treble,
    centroid,
    beat
  );
}