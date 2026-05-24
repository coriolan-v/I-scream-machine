#include <Arduino.h>
#include <I2S.h>
#include <arduinoFFT.h>

#define BAUD_RATE 1000000 
#define SAMPLING_FREQ 16000 
#define SAMPLES 64        

// --- COMPILATION FLAGS ---
#define VERBOSE_DEBUG 0  // Set to 1 for live telemetry, 0 to silence USB output

// Pin Assignments for INMP441 on XIAO RP2040
#define PIN_SCK      27 
#define PIN_WS       28 
#define PIN_SD       29 
#define PIN_UART_TX   0 

I2S i2sInput(INPUT);

struct RichAudioPacket {
  uint16_t header;          // 0xABCD
  uint8_t masterVolume;     
  uint8_t frequencyBins[16]; 
  uint8_t checksum;         
};

RichAudioPacket packet;

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

int sampleCount = 0;

// Verbose tracking variables
unsigned long lastTelemPrint = 0;
const unsigned long telemInterval = 250; // 4 updates per second for readability
uint32_t totalPacketsSent = 0;

void setup() {
#if VERBOSE_DEBUG
  Serial.begin(115200); 
#endif
  
  Serial1.setTX(PIN_UART_TX);
  Serial1.begin(BAUD_RATE);
  
  i2sInput.setBCLK(PIN_SCK);
  i2sInput.setDATA(PIN_SD); 
  i2sInput.setBitsPerSample(24);
  
  if (!i2sInput.begin(SAMPLING_FREQ)) {
    while (1); 
  }
  
  packet.header = 0xABCD; 
  delay(1000);

#if VERBOSE_DEBUG
  Serial.println("\n--- XIAO SENDER ACTIVE (VERBOSE DEBUG ON) ---");
#endif
}

void loop() {
  if (i2sInput.available()) {
    int32_t rawSample = 0;
    i2sInput.read(&rawSample, sizeof(rawSample));
    
    int32_t processedSample = (rawSample << 8) / 256;
    
    vReal[sampleCount] = (double)processedSample;
    vImag[sampleCount] = 0.0;
    sampleCount++;
    
    if (sampleCount >= SAMPLES) {
      // 1. Calculate overall Volume
      double sumSquares = 0;
      for (int i = 0; i < SAMPLES; i++) {
        sumSquares += vReal[i] * vReal[i];
      }
      double rms = sqrt(sumSquares / SAMPLES);
      packet.masterVolume = constrain(map(rms, 0, 45000, 0, 255), 0, 255);
      
      // 2. Compute FFT
      FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(FFT_FORWARD);
      FFT.complexToMagnitude();
      
      // 3. Populate frequency bins
      for (int i = 0; i < 16; i++) {
        double val = vReal[i + 2]; 
        packet.frequencyBins[i] = constrain(map(val, 0, 900000, 0, 255), 0, 255);
      }
      
      // 4. Calculate Checksum
      uint8_t calcCheck = packet.masterVolume;
      for (int i = 0; i < 16; i++) {
        calcCheck ^= packet.frequencyBins[i];
      }
      packet.checksum = calcCheck;
      
      // 5. Stream the raw binary packet
      Serial1.write((uint8_t*)&packet, sizeof(RichAudioPacket));
      totalPacketsSent++;
      
      sampleCount = 0;
    }
  }

  // --- LOCAL SENDER TELEMETRY ---
#if VERBOSE_DEBUG
  unsigned long currentMillis = millis();
  if (currentMillis - lastTelemPrint >= telemInterval) {
    lastTelemPrint = currentMillis;
    
    Serial.print("[SENDER LOG] Shipped: ");
    Serial.print(totalPacketsSent);
    Serial.print(" | Vol: ");
    Serial.print(packet.masterVolume);
    Serial.print(" | EQ Bins: [");
    for (int i = 0; i < 16; i++) {
      if (packet.frequencyBins[i] > 150)      Serial.print("#");
      else if (packet.frequencyBins[i] > 60)  Serial.print(":");
      else if (packet.frequencyBins[i] > 15)  Serial.print(".");
      else                                    Serial.print(" ");
    }
    Serial.println("]");
  }
#endif
}