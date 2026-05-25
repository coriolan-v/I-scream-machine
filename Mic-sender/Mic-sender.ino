/*
  XIAO RP2040 + INMP441 I2S microphone

  Adaptive festival version:
    - Learns background noise at startup
    - Tracks changing ambient noise when gate is closed
    - Freezes background learning when someone screams/talks loudly
    - Sends volume=0 for background
    - Sends strong volume for close voice/scream inside tube
    - Estimates dominant voice frequency
    - Maps pitch to rainbow solfège colour:
        Do  -> red
        Ré  -> orange
        Mi  -> yellow
        Fa  -> green
        Sol -> cyan
        La  -> blue
        Si  -> violet

  I2S pins:
    SCK / BCLK = GPIO27
    WS / LRCLK = GPIO28
    SD / DATA  = GPIO29

  UART:
    TX = GPIO0

  Frame format:
    0xAA 0x55 volume bass mid treble noteHue beat checksum

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

// For festival use, keep this moderate.
// If screams are not strong enough, try 4.0.
// If background is too sensitive, try 2.0.
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
// Dominant voice search range.
// Lower max makes it more stable for normal voices.
// Raise to 1500 if you want very high screams to push colour more.
static constexpr float PITCH_MIN_HZ = 120.0f;
static constexpr float PITCH_MAX_HZ = 1400.0f;

// Smooth note colour to avoid flicker.
// Higher = smoother but slower colour changes.
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

void sendFeatures(
  uint8_t vol,
  uint8_t bass,
  uint8_t mid,
  uint8_t treble,
  uint8_t noteHue,
  uint8_t beat
) {
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

  Serial1.write(frame, 8);
  Serial1.write(c);
}

void updateBackground(float voiceEnergy, bool tooClicky, bool nearGateOpening) {
  if (gateOpen) return;
  if (tooClicky) return;

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

// MIDI pitch-class note-to-colour mapping.
// Hue values are for the Teensy HSV function:
// 0 red, ~24 orange, ~42 yellow, ~85 green, ~128 cyan, ~170 blue, ~205 violet.
uint8_t noteHueFromFrequency(float freqHz, const char **noteNameOut) {
  if (freqHz < 40.0f || freqHz > 4000.0f) {
    if (noteNameOut) *noteNameOut = "Do";
    return 0;
  }

  // A4 = 440 Hz = MIDI note 69
  float midiFloat = 69.0f + 12.0f * log(freqHz / 440.0f) / log(2.0f);
  int midi = (int)(midiFloat + 0.5f);

  // Pitch class: C=0, C#=1, D=2, ..., B=11
  int pitchClass = midi % 12;
  if (pitchClass < 0) pitchClass += 12;

  switch (pitchClass) {
    case 0:   // C
    case 1:   // C#
      if (noteNameOut) *noteNameOut = "Do";
      return 0;     // red

    case 2:   // D
    case 3:   // D#
      if (noteNameOut) *noteNameOut = "Re";
      return 24;    // orange

    case 4:   // E
      if (noteNameOut) *noteNameOut = "Mi";
      return 42;    // yellow

    case 5:   // F
    case 6:   // F#
      if (noteNameOut) *noteNameOut = "Fa";
      return 85;    // green

    case 7:   // G
    case 8:   // G#
      if (noteNameOut) *noteNameOut = "Sol";
      return 128;   // cyan

    case 9:   // A
    case 10:  // A#
      if (noteNameOut) *noteNameOut = "La";
      return 170;   // blue

    case 11:  // B
      if (noteNameOut) *noteNameOut = "Si";
      return 205;   // violet
  }

  if (noteNameOut) *noteNameOut = "Do";
  return 0;
}

void printDebug(
  float rms,
  float voiceEnergy,
  float trebleRatio,
  bool tooClicky,
  float openThreshold,
  float closeThreshold,
  float peakVoiceFreq,
  float centroidHz,
  uint8_t vol,
  uint8_t bass,
  uint8_t mid,
  uint8_t treble,
  uint8_t noteHue,
  uint8_t beat
) {
  if (!VERBOSE) return;

  uint32_t now = millis();
  if (now - lastDebugMs < DEBUG_INTERVAL_MS) return;
  lastDebugMs = now;

  Serial.print("[XIAO] frames=");
  Serial.print(frameCounter);

  Serial.print(" calib=");
  Serial.print(startupCalibrating ? "YES" : "no");

  Serial.print(" rms=");
  Serial.print(rms, 5);

  Serial.print(" voice=");
  Serial.print(voiceEnergy, 5);

  Serial.print(" smoothVoice=");
  Serial.print(smoothVoiceEnergy, 5);

  Serial.print(" bg=");
  Serial.print(backgroundEnergy, 5);

  Serial.print(" openTh=");
  Serial.print(openThreshold, 5);

  Serial.print(" closeTh=");
  Serial.print(closeThreshold, 5);

  Serial.print(" ratio=");
  Serial.print(smoothVoiceEnergy / max(backgroundEnergy, 0.0001f), 2);

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

  Serial.print(" peakHz=");
  Serial.print(peakVoiceFreq, 0);

  Serial.print(" centroidHz=");
  Serial.print(centroidHz, 0);

  Serial.print(" note=");
  Serial.print(lastNoteName);

  Serial.print(" hue=");
  Serial.print(noteHue);

  Serial.print(" vol=");
  Serial.print(vol);

  Serial.print(" bass=");
  Serial.print(bass);

  Serial.print(" mid=");
  Serial.print(mid);

  Serial.print(" treble=");
  Serial.print(treble);

  Serial.print(" beat=");
  Serial.println(beat);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  startupMs = millis();

  Serial1.setTX(PIN_UART_TX);
  Serial1.begin(UART_BAUD);

  // With the RP2040 Arduino-Pico I2S library, WS/LRCLK is BCLK + 1.
  // GPIO27 BCLK and GPIO28 WS is correct.
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
    Serial.println("XIAO RP2040 adaptive festival scream / voice analyser started");
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
    Serial.println("Adaptive gate:");
    Serial.print("  Startup calibration ms: ");
    Serial.println(STARTUP_CALIBRATION_MS);
    Serial.print("  Open ratio: ");
    Serial.println(OPEN_RATIO, 2);
    Serial.print("  Close ratio: ");
    Serial.println(CLOSE_RATIO, 2);
    Serial.println("Note colour mapping:");
    Serial.println("  Do=red, Re=orange, Mi=yellow, Fa=green, Sol=cyan, La=blue, Si=violet");
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

  float peakVoiceMag = 0;
  float peakVoiceFreq = 0;

  for (uint16_t bin = 1; bin < FFT_SIZE / 2; bin++) {
    float freq = (float)bin * SAMPLE_RATE / FFT_SIZE;
    float mag = (float)vReal[bin];

    if (mag < 0.001f) continue;

    // Voice/scream bands.
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

    // Dominant pitch-ish frequency for note colour.
    // We only search the lower voice band because upper harmonics can dominate screams.
    if (freq >= PITCH_MIN_HZ && freq <= PITCH_MAX_HZ) {
      if (mag > peakVoiceMag) {
        peakVoiceMag = mag;
        peakVoiceFreq = freq;
      }
    }
  }

  // Voice-weighted energy.
  float voiceEnergy =
    lowVoiceE * 0.55f +
    midVoiceE * 1.00f +
    highVoiceE * 0.90f;

  // Reject mostly clicky sounds.
  float trebleRatio = trebleClickE / max(voiceEnergy, 0.0001f);
  bool tooClicky = trebleRatio > MAX_TREBLE_TO_VOICE_RATIO;

  // Smooth voice energy.
  if (voiceEnergy > smoothVoiceEnergy) {
    smoothVoiceEnergy = smoothVoiceEnergy * 0.55f + voiceEnergy * 0.45f;
  } else {
    smoothVoiceEnergy = smoothVoiceEnergy * 0.82f + voiceEnergy * 0.18f;
  }

  startupCalibrating = (millis() - startupMs) < STARTUP_CALIBRATION_MS;

  float openThreshold = max(ABS_OPEN_MIN, backgroundEnergy * OPEN_RATIO);
  float closeThreshold = max(ABS_CLOSE_MIN, backgroundEnergy * CLOSE_RATIO);

  bool nearGateOpening = smoothVoiceEnergy > backgroundEnergy * (OPEN_RATIO * 0.70f);
  updateBackground(voiceEnergy, tooClicky, nearGateOpening);

  // Recalculate after background update.
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
    float over =
      (smoothVoiceEnergy - closeThreshold) /
      max(backgroundEnergy * VOLUME_RANGE_MULT, 0.001f);

    normVol = constrain(over, 0.0f, 1.0f);
    normVol = powf(normVol, VOLUME_CURVE);
  }

  // Smooth output.
  if (normVol > smoothVol) {
    smoothVol = smoothVol * 0.35f + normVol * 0.65f;
  } else {
    smoothVol = smoothVol * 0.84f + normVol * 0.16f;
  }

  // If gate is closed or calibrating, force true silence quickly.
  if (!gateOpen || startupCalibrating) {
    smoothVol *= 0.35f;
    if (smoothVol < 0.020f) {
      smoothVol = 0.0f;
    }
  }

  uint8_t bass = 0;
  uint8_t mid = 0;
  uint8_t treble = 0;
  uint8_t noteHue = lastNoteHue;

  if (gateOpen) {
    float adaptiveBandScale = 0.20f / max(backgroundEnergy, 0.020f);
    adaptiveBandScale = constrain(adaptiveBandScale, 0.015f, 0.12f);

    bass = clampByte(constrain(lowVoiceE * adaptiveBandScale, 0.0f, 1.0f) * 255);
    mid = clampByte(constrain(midVoiceE * adaptiveBandScale * 0.75f, 0.0f, 1.0f) * 255);
    treble = clampByte(constrain(highVoiceE * adaptiveBandScale * 0.85f, 0.0f, 1.0f) * 255);

    const char *noteName = "Do";
    uint8_t targetHue = noteHueFromFrequency(peakVoiceFreq, &noteName);

    // Smooth the hue a little. Since hue wraps, but our palette is not continuous full rainbow,
    // simple smoothing is acceptable here.
    smoothNoteHue = smoothNoteHue * NOTE_HUE_SMOOTHING + targetHue * (1.0f - NOTE_HUE_SMOOTHING);

    noteHue = clampByte(smoothNoteHue);
    lastNoteHue = noteHue;
    lastNoteName = noteName;
  } else {
    // Keep last hue during silence; volume is zero so LEDs remain black.
    noteHue = lastNoteHue;
  }

  bool isBeat = gateOpen && smoothVol > 0.22f && smoothVol > prevVol * 1.30f;
  prevVol = prevVol * 0.88f + smoothVol * 0.12f;

  uint8_t vol = clampByte(smoothVol * 255.0f);
  uint8_t beat = isBeat ? 255 : 0;

  sendFeatures(
    vol,
    bass,
    mid,
    treble,
    noteHue,
    beat
  );

  frameCounter++;

  printDebug(
    rms,
    voiceEnergy,
    trebleRatio,
    tooClicky,
    openThreshold,
    closeThreshold,
    peakVoiceFreq,
    centroidHz,
    vol,
    bass,
    mid,
    treble,
    noteHue,
    beat
  );
}