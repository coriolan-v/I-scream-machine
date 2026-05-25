/*
  XIAO RP2040 + INMP441 I2S microphone

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
static constexpr float MIC_GAIN = 8.0f;

I2S i2s(INPUT);

double vReal[FFT_SIZE];
double vImag[FFT_SIZE];

ArduinoFFT<double> FFT(vReal, vImag, FFT_SIZE, SAMPLE_RATE);

// ---------- Smoothing ----------
float noiseFloor = 0.015f;
float smoothVol = 0.0f;
float prevVol = 0.0f;
float agc = 1.0f;

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
  float cleaned,
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

  Serial.print(" noise=");
  Serial.print(noiseFloor, 5);

  Serial.print(" cleaned=");
  Serial.print(cleaned, 5);

  Serial.print(" agc=");
  Serial.print(agc, 2);

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

  // UART TX on GPIO0
  Serial1.setTX(PIN_UART_TX);
  Serial1.begin(UART_BAUD);

  // I2S setup
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
    Serial.println("XIAO RP2040 audio analyser started");
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
  }
}

void loop() {
  int32_t left = 0;
  int32_t right = 0;

  float sumSq = 0.0f;

  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    i2s.read32(&left, &right);

    // INMP441 channel depends on L/R pin.
    // If you get silence, change this to:
    // int32_t raw = right;
    int32_t raw = left;

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

  // Adaptive noise floor
  if (rms < noiseFloor * 1.5f) {
    noiseFloor = noiseFloor * 0.995f + rms * 0.005f;
  }

  float cleaned = max(0.0f, rms - noiseFloor);

  // Simple AGC
  float target = 0.12f;

  if (cleaned > 0.0001f) {
    float desiredAgc = target / cleaned;
    desiredAgc = constrain(desiredAgc, 0.8f, 80.0f);
    agc = agc * 0.995f + desiredAgc * 0.005f;
  }

  float normVol = constrain(cleaned * agc, 0.0f, 1.0f);
  smoothVol = smoothVol * 0.80f + normVol * 0.20f;

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  float bassE = 0;
  float midE = 0;
  float trebleE = 0;
  float totalE = 0;
  float weightedFreq = 0;

  for (uint16_t bin = 1; bin < FFT_SIZE / 2; bin++) {
    float freq = (float)bin * SAMPLE_RATE / FFT_SIZE;
    float mag = (float)vReal[bin];

    if (mag < 0.001f) continue;

    if (freq >= 80 && freq < 350) {
      bassE += mag;
    } else if (freq >= 350 && freq < 1800) {
      midE += mag;
    } else if (freq >= 1800 && freq < 5500) {
      trebleE += mag;
    }

    if (freq >= 80 && freq < 5500) {
      totalE += mag;
      weightedFreq += mag * freq;
    }
  }

  float centroidHz = totalE > 0 ? weightedFreq / totalE : 0;

  float bandGain = 0.0035f * agc;

  uint8_t bass = clampByte(constrain(bassE * bandGain, 0.0f, 1.0f) * 255);
  uint8_t mid = clampByte(constrain(midE * bandGain * 0.55f, 0.0f, 1.0f) * 255);
  uint8_t treble = clampByte(constrain(trebleE * bandGain * 0.90f, 0.0f, 1.0f) * 255);

  float cNorm = (centroidHz - 100.0f) / (5000.0f - 100.0f);
  uint8_t centroid = clampByte(constrain(cNorm, 0.0f, 1.0f) * 255);

  bool isBeat = smoothVol > 0.16f && smoothVol > prevVol * 1.45f;
  prevVol = prevVol * 0.92f + smoothVol * 0.08f;

  uint8_t vol = clampByte(powf(smoothVol, 0.55f) * 255.0f);
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
    cleaned,
    centroidHz,
    vol,
    bass,
    mid,
    treble,
    centroid,
    beat
  );
}