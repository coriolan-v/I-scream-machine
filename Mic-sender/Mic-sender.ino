#include <Arduino.h>
#include <I2S.h>
#include <arduinoFFT.h>

#define BAUD_RATE 115200 
#define SAMPLING_FREQ 16000 
#define SAMPLES 64        

// --- COMPILATION FLAGS ---
#define VERBOSE_DEBUG 1  // Set to 1 to view live audio data on your PC Serial Monitor

// Pin Assignments for INMP441 on XIAO RP2040
#define PIN_SCK      27 
#define PIN_WS       28 
#define PIN_SD       29 
#define PIN_UART_TX   0 

I2S i2sInput(INPUT);

// The structural rich packet mapping
struct RichAudioPacket {
  uint16_t header;          
  uint8_t masterVolume;     
  uint8_t frequencyBins[16]; 
  uint8_t checksum;         
};

RichAudioPacket packet;

// FFT Processing Arrays
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

int sampleCount = 0;

// --- DIAGNOSTIC MONITOR VARIABLES ---
unsigned long lastTelemPrint = 0;
const unsigned long telemInterval = 250; // Update local PC monitor every 250ms (~4 FPS)
uint32_t totalPacketsSent = 0;

void setup() {
  // Initialize native USB port for PC local debugging
  Serial.begin(115200); 
  
  // Set up the long-distance hardware RS-485 serial link
  Serial1.setTX(PIN_UART_TX);
  Serial1.begin(BAUD_RATE);
  
  i2sInput.setBCLK(PIN_SCK);
  i2sInput.setDATA(PIN_SD); 
  i2sInput.setBitsPerSample(24);
  
  if (!i2sInput.begin(SAMPLING_FREQ)) {
    Serial.println("CRITICAL ERROR: I2S Hardware Bus Failed to Start!");
    while (1); 
  }
  
  packet.header = 0xABCD; 

  delay(1500);
  Serial.println("\n===========================================");
  Serial.println("    XIAO RP2040 SMART ENDPOINT SENDER      ");
  Serial.println("===========================================");
  Serial.print("RS-485 Data Output on Pin D6 (Serial1) @ "); Serial.println(BAUD_RATE);
  Serial.println("Processing local 64-point FFT structures...");
  Serial.println("-------------------------------------------\n");
}

void loop() {
  if (i2sInput.available()) {
    int32_t rawSample = 0;
    i2sInput.read(&rawSample, sizeof(rawSample));
    
    // Core native sign alignment algorithm
    int32_t processedSample = (rawSample << 8) / 256;
    
    vReal[sampleCount] = (double)processedSample;
    vImag[sampleCount] = 0.0;
    sampleCount++;
    
    if (sampleCount >= SAMPLES) {
      // 1. Calculate overall raw volume (RMS style)
      double sumSquares = 0;
      for (int i = 0; i < SAMPLES; i++) {
        sumSquares += vReal[i] * vReal[i];
      }
      double rms = sqrt(sumSquares / SAMPLES);
      packet.masterVolume = constrain(map(rms, 0, 8000, 0, 255), 0, 255);
      
      // 2. Compute Fast Fourier Transform
      FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(FFT_FORWARD);
      FFT.complexToMagnitude();
      
      // 3. Compress the complex spectrum down into 16 neat animation bins
      for (int i = 0; i < 16; i++) {
        double val = vReal[i + 2]; 
        packet.frequencyBins[i] = constrain(map(val, 0, 150000, 0, 255), 0, 255);
      }
      
      // 4. Create an XOR data validation checksum
      uint8_t calcCheck = packet.masterVolume;
      for (int i = 0; i < 16; i++) {
        calcCheck ^= packet.frequencyBins[i];
      }
      packet.checksum = calcCheck;
      
      // 5. Blast uncompressed data footprint instantly to Teensy
      Serial1.write((uint8_t*)&packet, sizeof(RichAudioPacket));
      totalPacketsSent++;
      
      sampleCount = 0;
    }
  }

  // --- LOCAL SENDER VERBOSE DEBUG PANEL ---
#if VERBOSE_DEBUG
  unsigned long currentMillis = millis();
  if (currentMillis - lastTelemPrint >= telemInterval) {
    lastTelemPrint = currentMillis;
    
    Serial.print("[SENDER LOG] Total Packets Shipped: ");
    Serial.print(totalPacketsSent);
    Serial.print(" | Vol: ");
    Serial.print(packet.masterVolume);
    
    // Print a mini ASCII-art bar graph of the frequency spectrum
    Serial.print(" | EQ Spectrum: [");
    for (int i = 0; i < 16; i++) {
      if (packet.frequencyBins[i] > 180)      Serial.print("#"); // Heavy energy
      else if (packet.frequencyBins[i] > 80)  Serial.print(":"); // Mid energy
      else if (packet.frequencyBins[i] > 15)  Serial.print("."); // Low background noise
      else                                    Serial.print(" "); // Dead silence
    }
    Serial.println("]");
  }
#endif
}