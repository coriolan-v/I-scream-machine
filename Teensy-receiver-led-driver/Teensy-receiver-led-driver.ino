#include <OctoWS2811.h>
#include <FastLED.h>

#define BAUD_RATE 115200
#define NUM_STRIPS 12
#define LEDS_PER_STRIP 200
#define TOTAL_LEDS (NUM_STRIPS * LEDS_PER_STRIP)

#define VERBOSE_DEBUG 1  

CRGB leds[TOTAL_LEDS];
const int numPins = 12;
byte pinList[numPins] = {2, 14, 7, 8, 6, 20, 21, 5, 22, 23, 41, 42};

DMAMEM int displayMemory[TOTAL_LEDS * 3 / 4];
int drawingMemory[TOTAL_LEDS * 3 / 4];
const int config = WS2811_GRB | WS2811_800kHz;
OctoWS2811 octo(LEDS_PER_STRIP, displayMemory, drawingMemory, config, numPins, pinList);

struct RichAudioPacket {
  uint16_t header;          
  uint8_t masterVolume;     
  uint8_t frequencyBins[16]; 
  uint8_t checksum;         
};

RichAudioPacket micA; 
RichAudioPacket micB; 

uint32_t micA_PacketsReceived = 0;
uint32_t micB_PacketsReceived = 0;
uint32_t micA_ChecksumErrors = 0;
uint32_t micB_ChecksumErrors = 0;
unsigned long lastTelemPrint = 0;
const unsigned long telemInterval = 500; 

void showOctoBuffer() {
  for (int i = 0; i < TOTAL_LEDS; i++) {
    octo.setPixel(i, leds[i].r, leds[i].g, leds[i].b);
    delay(10);
  }
  octo.show();
}

void setup() {
  Serial.begin(115200); 
  
  octo.begin();
  octo.show();
  
  // Quick validation flash
  fill_solid(leds, TOTAL_LEDS, CRGB::Green); showOctoBuffer(); delay(300);
  fill_solid(leds, TOTAL_LEDS, CRGB::Black); showOctoBuffer();

  // Initialize Serial4 on Pin 16 instead of Serial1
  Serial4.begin(BAUD_RATE);
  Serial4.addMemoryForRead(malloc(1024), 1024);
  
  // Keeping Serial3 on Pin 15 active for your second mic channel
  Serial3.begin(BAUD_RATE);
  Serial3.addMemoryForRead(malloc(1024), 1024);

  delay(1000);
  Serial.println("\n===========================================");
  Serial.println("  TEENSY 4.1 PORT-ADJUSTED RECEIVER RUNNING ");
  Serial.println("===========================================");
  Serial.print("Listening for Mic A on Pin 16 (Serial4) at "); Serial.print(BAUD_RATE); Serial.println(" bds.");
  Serial.print("Listening for Mic B on Pin 15 (Serial3) at "); Serial.print(BAUD_RATE); Serial.println(" bds.");
  Serial.println("-------------------------------------------\n");
}

void loop() {
  // 1. Scan and parse Mic A Stream via Serial4 (Pin 16)
// 1. Scan and parse Mic A Stream via Serial4 (Pin 16)
  if (Serial4.available() >= sizeof(RichAudioPacket)) {
    // Look for our specific magic sync code
    if (Serial4.read() == 0xAB && Serial4.peek() == 0xCD) {
      Serial4.read(); // Clear trailing header byte
      Serial4.readBytes((char*)&micA + 2, sizeof(RichAudioPacket) - 2);
      
      // Checksum Validation
      uint8_t verifyCheck = micA.masterVolume;
      for (int i = 0; i < 16; i++) verifyCheck ^= micA.frequencyBins[i];
      
      if (verifyCheck == micA.checksum) {
        micA_PacketsReceived++;
      } else {
        micA_ChecksumErrors++;
      }
    } else {
      // If the header framing is misaligned, purge the corrupt data pool 
      // so the next full 20-byte packet can land cleanly on the next pass
      Serial4.clear(); 
    }
  }

  // 2. Scan and parse Mic B Stream via Serial3 (Pin 15)
  while (Serial3.available() >= sizeof(RichAudioPacket)) {
    if (Serial3.read() == 0xAB && Serial3.peek() == 0xCD) {
      Serial3.read(); 
      Serial3.readBytes((char*)&micB + 2, sizeof(RichAudioPacket) - 2);
      
      uint8_t verifyCheck = micB.masterVolume;
      for (int i = 0; i < 16; i++) verifyCheck ^= micB.frequencyBins[i];
      
      if (verifyCheck == micB.checksum) {
        micB_PacketsReceived++;
      } else {
        micB_ChecksumErrors++;
      }
    }
  }

  // Mini animation engine placeholder
  fadeToBlackBy(leds, TOTAL_LEDS, 30);
  showOctoBuffer();

  // --- TELEMETRY DASHBOARD ---
#if VERBOSE_DEBUG
  unsigned long currentMillis = millis();
  if (currentMillis - lastTelemPrint >= telemInterval) {
    lastTelemPrint = currentMillis;
    
    Serial.println("--- LIVE BUS DATA REPORT ---");
    Serial.print(" [MIC A (Serial4 - Pin 16)] Good Packets: "); Serial.print(micA_PacketsReceived);
    Serial.print(" | CRC Errors: "); Serial.print(micA_ChecksumErrors);
    Serial.print(" | Vol: "); Serial.println(micA.masterVolume);
    
    Serial.print(" [MIC B (Serial3 - Pin 15)] Good Packets: "); Serial.print(micB_PacketsReceived);
    Serial.print(" | CRC Errors: "); Serial.print(micB_ChecksumErrors);
    Serial.print(" | Vol: "); Serial.println(micB.masterVolume);
    Serial.println(); 
  }
#endif
}