#include <Arduino.h>
#include <I2S.h>
#include <math.h>

#define BAUD_RATE 1000000 // 1 Megabaud link to Teensy

// Pin Assignments for INMP441 on XIAO RP2040
#define PIN_SCK      27 // D1 (P27) -> SCK on mic
#define PIN_WS       28 // D2 (P28) -> WS on mic
#define PIN_SD       29 // D3 (P29) -> SD on mic
#define PIN_UART_TX   0 // D6 (P0)  -> Hardware Serial1 TX to RS-485

I2S i2sInput(INPUT);

#define AUDIO_BLOCK_SIZE 32
struct AudioChunk {
  uint16_t header; // 0x55AA sync code
  int32_t samples[AUDIO_BLOCK_SIZE];
};

AudioChunk chunk;
int sampleIndex = 0;

// Signal Processing Windows
int32_t rawSamplesWindow[AUDIO_BLOCK_SIZE];

// --- 15 FPS PLOTTER TIMER VARIABLES ---
unsigned long lastPlotTime = 0;
const unsigned long plotInterval = 66; // 66ms = ~15 frames per second
double latestRmsVolume = 0;           // Holds the last calculated volume frame

void setup() {
  Serial.begin(115200); 
  
  Serial1.setTX(PIN_UART_TX);
  Serial1.begin(BAUD_RATE);
  
  i2sInput.setBCLK(PIN_SCK);
  i2sInput.setDATA(PIN_SD); 
  i2sInput.setBitsPerSample(24);
  
  if (!i2sInput.begin(16000)) {
    while (1); 
  }
  
  chunk.header = 0x55AA; 
}

void loop() {
  if (i2sInput.available()) {
    int32_t rawSample = 0;
    i2sInput.read(&rawSample, sizeof(rawSample));
    
    // Core native sign alignment algorithm
    int32_t processedSample = (rawSample << 8) / 256;
    
    // Software noise floor gate to filter out trace oscillation
    if (abs(processedSample) < 150) processedSample = 0; 
    
    chunk.samples[sampleIndex] = processedSample;
    rawSamplesWindow[sampleIndex] = processedSample;
    sampleIndex++;
    
    if (sampleIndex >= AUDIO_BLOCK_SIZE) {
      // KEEP SENDING TO TEENSY AT MAXIMUM HARDWARE SPEED (NO DELAYS)
      Serial1.write((uint8_t*)&chunk, sizeof(AudioChunk));
      
      // Calculate current Root Mean Square (RMS) Volume
      double sumSquares = 0;
      for (int i = 0; i < AUDIO_BLOCK_SIZE; i++) {
        sumSquares += (double)rawSamplesWindow[i] * (double)rawSamplesWindow[i];
      }
      latestRmsVolume = sqrt(sumSquares / AUDIO_BLOCK_SIZE);
      
      sampleIndex = 0;
    }
  }

  // --- NON-BLOCKING 15 FPS VISUAL OUTPUT ENGINE ---
  unsigned long currentTime = millis();
  if (currentTime - lastPlotTime >= plotInterval) {
    lastPlotTime = currentTime;
    
    // Prints the data at a readable 15Hz human refresh rate
    Serial.print("Volume_RMS:");
    Serial.print(latestRmsVolume);
    Serial.print(",");
    Serial.print("Max_Limit:15000,"); 
    Serial.println("Min_Limit:0");
  }
}