#include <Arduino.h>
#include <I2S.h>
#include <arduinoFFT.h>

#define BAUD_RATE 1000000 // 1 Megabaud link to Teensy
#define SAMPLING_FREQ 16000 
#define SAMPLES 64        // Fast window size for instant, low-latency updates

// Pin Assignments for INMP441 on XIAO RP2040
#define PIN_SCK      27 
#define PIN_WS       28 
#define PIN_SD       29 
#define PIN_UART_TX   0 

I2S i2sInput(INPUT);

// The structural packet matching what the Teensy will expect
struct RichAudioPacket {
  uint16_t header;          // 0xABCD magic sync code
  uint8_t masterVolume;     // Overall intensity of the sound
  uint8_t frequencyBins[16]; // 16 distinct EQ bands (Bass to Treble)
  uint8_t checksum;         // Safety check to prevent LED glitching
};

RichAudioPacket packet;

// FFT Processing Arrays
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

int sampleCount = 0;

void setup() {
  // Serial1 goes out through the MAX3485 transceiver
  Serial1.setTX(PIN_UART_TX);
  Serial1.begin(BAUD_RATE);
  
  i2sInput.setBCLK(PIN_SCK);
  i2sInput.setDATA(PIN_SD); 
  i2sInput.setBitsPerSample(24);
  
  if (!i2sInput.begin(SAMPLING_FREQ)) {
    while (1); 
  }
  
  packet.header = 0xABCD; // Fixed signature
}

void loop() {
  if (i2sInput.available()) {
    int32_t rawSample = 0;
    i2sInput.read(&rawSample, sizeof(rawSample));
    
    // Core native sign alignment algorithm
    int32_t processedSample = (rawSample << 8) / 256;
    
    // Save to our real component array for the frequency transform
    vReal[sampleCount] = (double)processedSample;
    vImag[sampleCount] = 0.0;
    sampleCount++;
    
    // Once we have a full mathematical window, compute the magic!
    if (sampleCount >= SAMPLES) {
      
      // 1. Calculate overall raw volume (RMS style)
      double sumSquares = 0;
      for (int i = 0; i < SAMPLES; i++) {
        sumSquares += vReal[i] * vReal[i];
      }
      double rms = sqrt(sumSquares / SAMPLES);
      // Map and constrain to a single byte (0-255)
      packet.masterVolume = constrain(map(rms, 0, 8000, 0, 255), 0, 255);
      
      // 2. Compute Fast Fourier Transform (Extract pitches)
      FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(FFT_FORWARD);
      FFT.complexToMagnitude();
      
      // 3. Compress the complex spectrum down into 16 neat animation bins
      // Human voices sit heavily in the lower half of a 16kHz sample rate
      for (int i = 0; i < 16; i++) {
        // We pull sequential harmonic bars directly out of the real spectrum array
        double val = vReal[i + 2]; 
        packet.frequencyBins[i] = constrain(map(val, 0, 150000, 0, 255), 0, 255);
      }
      
      // 4. Create an XOR data validation checksum
      uint8_t calcCheck = packet.masterVolume;
      for (int i = 0; i < 16; i++) {
        calcCheck ^= packet.frequencyBins[i];
      }
      packet.checksum = calcCheck;
      
      // 5. Blast the 20-byte packed footprint instantly down the RS-485 line
      Serial1.write((uint8_t*)&packet, sizeof(RichAudioPacket));
      
      sampleCount = 0;
    }
  }
}