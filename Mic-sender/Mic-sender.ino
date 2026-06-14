/*
  XIAO RP2040 + INMP441 I2S microphone

  Adaptive festival version:
    - Learns background noise at startup
    - Tracks changing ambient noise when gate is closed
    - Freezes background learning when someone screams/talks loudly
    - Sends volume=0 for background
    - Sends strong volume for close voice/scream inside tube
    - Estimates dominant voice frequency
    - Maps pitch to rainbow solfège colour

  I2S pins:
    SCK / BCLK = GPIO27
    WS / LRCLK = GPIO28
    SD / DATA  = GPIO29

  UART Mapping (Using PIO Serial):
    TX = Pin D5 (GPIO7) -> Connects to Teensy Pin 16 (Serial4 RX)
    RX = Unused

  Frame format:
    0xAA 0x55 volume bass mid treble noteHue beat checksum
*/

#include <Arduino.h>
#include <I2S.h>
#include <arduinoFFT.h>
//#include <SerialPIO.h>  // Required for PIO-based Serial on D5

// ---------- Debug ----------
#define VERBOSE true
#define DEBUG_INTERVAL_MS 50

// ---------- Pins ----------
#define PIN_WS   3    // D10 / GPIO3
#define PIN_SD   4    // D9 / GPIO4
#define PIN_SCK  2    // D8 / GPIO2
#define PIN_UART_TX   28  // Changed to physical Pin D5 (GPIO7)

// ---------- UART ----------
#define UART_BAUD 460800

// Initialize PIO Serial on Pin D5 for TX, and disable RX (-1 / PIN_UNUSED)
//SerialPIO Serial1(PIN_UART_TX, 6);

// ---------- Audio ----------
static constexpr uint32_t SAMPLE_RATE = 16000;
static constexpr uint16_t FFT_SIZE = 256;
static constexpr float MIC_GAIN = 3.0f;

// ---------- Adaptive background / gate tuning ----------
static constexpr uint32_t STARTUP_CALIBRATION_MS = 3000;
static constexpr float BACKGROUND_MIN = 0.018f;
static constexpr float BACKGROUND_MAX = 1.500f;
static constexpr float OPEN_RATIO = 4.5f;
static constexpr float CLOSE_RATIO = 2.4f;
static constexpr float ABS_OPEN_MIN = 0.055f;
static constexpr float ABS_CLOSE_MIN = 0.035f;
static constexpr uint8_t GATE_ATTACK_FRAMES = 4;
static constexpr uint8_t GATE_RELEASE_FRAMES = 12;
static constexpr float MAX_TREBLE_TO_VOICE_RATIO = 1.8f;
static constexpr float BG_DOWN_RATE = 0.030f;
static constexpr float BG_UP_RATE = 0.004f;
static constexpr float BG_FAST_UP_RATE = 0.012f;
static constexpr float VOLUME_RANGE_MULT = 5.0f;
static constexpr float VOLUME_CURVE = 0.70f;

// ---------- Pitch / colour tuning ----------
static constexpr float PITCH_MIN_HZ = 120.0f;
static constexpr float PITCH_MAX_HZ = 1400.0f;
static constexpr float NOTE_HUE_SMOOTHING = 0.70f;

I2S i2s(INPUT);
double vReal[FFT_SIZE];
double vImag[FFT_SIZE];
ArduinoFFT<double> FFT(vReal, vImag, FFT_SIZE, SAMPLE_RATE);

// ---------- Detection state ----------
float backgroundEnergy = BACKGROUND_MIN;
float smoothVoiceEnergy = 0.0f;
float smoothVol = 0.0f;
float prevVol = 0.0f;
float smoothNoteHue = 0.0f;
uint8_t lastNoteHue = 0;
const char *lastNoteName = "Do";
bool gateOpen = false;
bool startupCalibrating = true;
uint8_t gateAttackCount = 0;
uint8_t gateReleaseCount = 0;
uint32_t startupMs = 0;
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

void sendFeatures(uint8_t vol, uint8_t bass, uint8_t mid, uint8_t treble, uint8_t noteHue, uint8_t beat) {
  uint8_t frame[8];
  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = vol;
  frame[3] = bass;
  frame[4] = mid;
  frame[5] = treble;
  frame[6] = noteHue;
  frame[7] = beat;

  uint8_t c = checksumFrame(frame, 8);

  // Swapped to use the newly initialized PIO Serial1 channel
  Serial1.write(frame, 8);
  Serial1.write(c);
}

void updateBackground(float voiceEnergy, bool tooClicky, bool nearGateOpening) {
  if (gateOpen || tooClicky) return;
  float target = constrain(voiceEnergy, BACKGROUND_MIN, BACKGROUND_MAX);

  if (startupCalibrating) {
    backgroundEnergy = backgroundEnergy * 0.90f + target * 0.10f;
    backgroundEnergy = constrain(backgroundEnergy, BACKGROUND_MIN, BACKGROUND_MAX);
    return;
  }

  if (nearGateOpening) {
    backgroundEnergy = backgroundEnergy * (1.0f - BG_FAST_UP_RATE) + target * BG_FAST_UP_RATE;
  } else if (target < backgroundEnergy) {
    backgroundEnergy = backgroundEnergy * (1.0f - BG_DOWN_RATE) + target * BG_DOWN_RATE;
  } else {
    backgroundEnergy = backgroundEnergy * (1.0f - BG_UP_RATE) + target * BG_UP_RATE;
  }
  backgroundEnergy = constrain(backgroundEnergy, BACKGROUND_MIN, BACKGROUND_MAX);
}

uint8_t noteHueFromFrequency(float freqHz, const char **noteNameOut) {
  if (freqHz < 40.0f || freqHz > 4000.0f) {
    if (noteNameOut) *noteNameOut = "Do";
    return 0;
  }
  float midiFloat = 69.0f + 12.0f * log(freqHz / 440.0f) / log(2.0f);
  int midi = (int)(midiFloat + 0.5f);
  int pitchClass = midi % 12;
  if (pitchClass < 0) pitchClass += 12;

  switch (pitchClass) {
    case 0: case 1:   if (noteNameOut) *noteNameOut = "Do";  return 0;
    case 2: case 3:   if (noteNameOut) *noteNameOut = "Re";  return 24;
    case 4:           if (noteNameOut) *noteNameOut = "Mi";  return 42;
    case 5: case 6:   if (noteNameOut) *noteNameOut = "Fa";  return 85;
    case 7: case 8:   if (noteNameOut) *noteNameOut = "Sol"; return 128;
    case 9: case 10:  if (noteNameOut) *noteNameOut = "La";  return 170;
    case 11:          if (noteNameOut) *noteNameOut = "Si";  return 205;
  }
  if (noteNameOut) *noteNameOut = "Do";
  return 0;
}

void printDebug(float rms, float voiceEnergy, float trebleRatio, bool tooClicky, float openThreshold, float closeThreshold, float peakVoiceFreq, float centroidHz, uint8_t vol, uint8_t bass, uint8_t mid, uint8_t treble, uint8_t noteHue, uint8_t beat) {
  if (!VERBOSE) return;
  uint32_t now = millis();
  if (now - lastDebugMs < DEBUG_INTERVAL_MS) return;
  lastDebugMs = now;

  Serial.print("[XIAO] frames=");       Serial.print(frameCounter);
  Serial.print(" calib=");             Serial.print(startupCalibrating ? "YES" : "no");
  Serial.print(" rms=");               Serial.print(rms, 5);
  Serial.print(" voice=");             Serial.print(voiceEnergy, 5);
  Serial.print(" bg=");                Serial.print(backgroundEnergy, 5);
  Serial.print(" gate=");              Serial.print(gateOpen ? "OPEN" : "closed");
  Serial.print(" peakHz=");            Serial.print(peakVoiceFreq, 0);
  Serial.print(" note=");              Serial.print(lastNoteName);
  Serial.print(" vol=");               Serial.print(vol);
  Serial.print(" beat=");              Serial.println(beat);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  startupMs = millis();

  // Pull GPIO 28 HIGH just like in your successful test script
  pinMode(28, OUTPUT);
  digitalWrite(28, HIGH);

  // Initialize the standard hardware Serial1 
  Serial1.begin(UART_BAUD);

  // Re-assign I2S to the non-conflicting pins
  i2s.setBCLK(PIN_SCK);
  i2s.setDATA(PIN_SD);
  i2s.setBitsPerSample(32);
  i2s.setBuffers(6, 256);

  if (!i2s.begin(SAMPLE_RATE)) {
    Serial.println("I2S begin failed");
    while (true) delay(100);
  }

  if (VERBOSE) {
    Serial.println("\nXIAO RP2040 adaptive festival scream analyzer initialized");
    Serial.print("Hardware Serial1 Running on Pin D9 (GPIO4) at: "); Serial.println(UART_BAUD);
  }
}

void loop() {
  int32_t left = 0, right = 0;
  float sumSq = 0.0f;

  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    i2s.read32(&left, &right);
    int32_t raw = left; 
    float sample = (float)(raw >> 8) / 8388608.0f;

    static float dc = 0;
    dc = dc * 0.995f + sample * 0.005f;
    sample -= dc;
    sample *= MIC_GAIN;

    float w = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_SIZE - 1)));
    vReal[i] = sample * w;
    vImag[i] = 0.0;
    sumSq += sample * sample;
  }

  float rms = sqrtf(sumSq / FFT_SIZE);

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  float lowVoiceE = 0, midVoiceE = 0, highVoiceE = 0, trebleClickE = 0;
  float totalVoiceE = 0, weightedFreq = 0;
  float peakVoiceMag = 0, peakVoiceFreq = 0;

  for (uint16_t bin = 1; bin < FFT_SIZE / 2; bin++) {
    float freq = (float)bin * SAMPLE_RATE / FFT_SIZE;
    float mag = (float)vReal[bin];
    if (mag < 0.001f) continue;

    if (freq >= 120 && freq < 350)         lowVoiceE += mag;
    else if (freq >= 350 && freq < 1600)   midVoiceE += mag;
    else if (freq >= 1600 && freq < 3500)  highVoiceE += mag;
    else if (freq >= 3500 && freq < 6500)  trebleClickE += mag;

    if (freq >= 120 && freq < 3500) {
      totalVoiceE += mag;
      weightedFreq += mag * freq;
    }
    if (freq >= PITCH_MIN_HZ && freq <= PITCH_MAX_HZ) {
      if (mag > peakVoiceMag) {
        peakVoiceMag = mag;
        peakVoiceFreq = freq;
      }
    }
  }

  float voiceEnergy = lowVoiceE * 0.55f + midVoiceE * 1.00f + highVoiceE * 0.90f;
  float trebleRatio = trebleClickE / max(voiceEnergy, 0.0001f);
  bool tooClicky = trebleRatio > MAX_TREBLE_TO_VOICE_RATIO;

  if (voiceEnergy > smoothVoiceEnergy) smoothVoiceEnergy = smoothVoiceEnergy * 0.55f + voiceEnergy * 0.45f;
  else                                 smoothVoiceEnergy = smoothVoiceEnergy * 0.82f + voiceEnergy * 0.18f;

  startupCalibrating = (millis() - startupMs) < STARTUP_CALIBRATION_MS;

  float openThreshold = max(ABS_OPEN_MIN, backgroundEnergy * OPEN_RATIO);
  float closeThreshold = max(ABS_CLOSE_MIN, backgroundEnergy * CLOSE_RATIO);
  bool nearGateOpening = smoothVoiceEnergy > backgroundEnergy * (OPEN_RATIO * 0.70f);
  
  updateBackground(voiceEnergy, tooClicky, nearGateOpening);

  openThreshold = max(ABS_OPEN_MIN, backgroundEnergy * OPEN_RATIO);
  closeThreshold = max(ABS_CLOSE_MIN, backgroundEnergy * CLOSE_RATIO);

  bool wantsOpen = smoothVoiceEnergy > openThreshold && !tooClicky && !startupCalibrating;
  bool wantsClose = smoothVoiceEnergy < closeThreshold || tooClicky || startupCalibrating;

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
      if (gateReleaseCount > 0) gateReleaseCount--;
      else {
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
    float over = (smoothVoiceEnergy - closeThreshold) / max(backgroundEnergy * VOLUME_RANGE_MULT, 0.001f);
    normVol = constrain(over, 0.0f, 1.0f);
    normVol = powf(normVol, VOLUME_CURVE);
  }

  if (normVol > smoothVol) smoothVol = smoothVol * 0.35f + normVol * 0.65f;
  else                     smoothVol = smoothVol * 0.84f + normVol * 0.16f;

  if (!gateOpen || startupCalibrating) {
    smoothVol *= 0.35f;
    if (smoothVol < 0.020f) smoothVol = 0.0f;
  }

  uint8_t bass = 0, mid = 0, treble = 0;
  uint8_t noteHue = lastNoteHue;

  if (gateOpen) {
    float adaptiveBandScale = 0.20f / max(backgroundEnergy, 0.020f);
    adaptiveBandScale = constrain(adaptiveBandScale, 0.015f, 0.12f);

    bass = clampByte(constrain(lowVoiceE * adaptiveBandScale, 0.0f, 1.0f) * 255);
    mid = clampByte(constrain(midVoiceE * adaptiveBandScale * 0.75f, 0.0f, 1.0f) * 255);
    treble = clampByte(constrain(highVoiceE * adaptiveBandScale * 0.85f, 0.0f, 1.0f) * 255);

    const char *noteName = "Do";
    uint8_t targetHue = noteHueFromFrequency(peakVoiceFreq, &noteName);

    smoothNoteHue = smoothNoteHue * NOTE_HUE_SMOOTHING + targetHue * (1.0f - NOTE_HUE_SMOOTHING);
    noteHue = clampByte(smoothNoteHue);
    lastNoteHue = noteHue;
    lastNoteName = noteName;
  } else {
    noteHue = lastNoteHue;
  }

  bool isBeat = gateOpen && smoothVol > 0.22f && smoothVol > prevVol * 1.30f;
  prevVol = prevVol * 0.88f + smoothVol * 0.12f;

  uint8_t vol = clampByte(smoothVol * 255.0f);
  uint8_t beat = isBeat ? 255 : 0;

  sendFeatures(vol, bass, mid, treble, noteHue, beat);
  frameCounter++;

  printDebug(rms, voiceEnergy, trebleRatio, tooClicky, openThreshold, closeThreshold, peakVoiceFreq, centroidHz, vol, bass, mid, treble, noteHue, beat);
}