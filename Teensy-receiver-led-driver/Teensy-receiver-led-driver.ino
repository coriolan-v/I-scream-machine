#include <OctoWS2811.h>
#include <FastLED.h>

#define BAUD_RATE 1000000 // Rock solid 1 Megabaud
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

volatile RichAudioPacket micA; 

// --- NEW ROLLING CIRCULAR STREAM PARSER ---
#define PACKET_SIZE sizeof(RichAudioPacket)
uint8_t slidingBuffer[PACKET_SIZE];

volatile uint32_t goodPackets = 0;
volatile uint32_t badChecksums = 0;
unsigned long lastPrintTime = 0;

IntervalTimer serialParserTimer;

// This interrupt runs every 500 microseconds to pull bits out dynamically
void parseSerialStream() {
  while (Serial4.available() > 0) {
    // Shift all elements in the buffer left by 1 byte
    for (uint i = 0; i < PACKET_SIZE - 1; i++) {
      slidingBuffer[i] = slidingBuffer[i + 1];
    }
    // Drop the fresh byte into the tail of our sliding window
    slidingBuffer[PACKET_SIZE - 1] = Serial4.read();

    // Check if the front of our sliding window matches the 0xABCD signature
    if (slidingBuffer[0] == 0xAB && slidingBuffer[1] == 0xCD) {
      
      // Calculate Checksum across the payload inside the frame
      uint8_t verifyCheck = slidingBuffer[2]; // masterVolume
      for (int i = 0; i < 16; i++) {
        verifyCheck ^= slidingBuffer[3 + i]; // EQ bins
      }

      // Validate against the trailing checksum byte
      if (verifyCheck == slidingBuffer[PACKET_SIZE - 1]) {
        goodPackets++;
        memcpy((void*)&micA, slidingBuffer, PACKET_SIZE);
      } else {
        badChecksums++;
      }
    }
  }
}

void showOctoBuffer() {
  for (int i = 0; i < TOTAL_LEDS; i++) {
    octo.setPixel(i, leds[i].r, leds[i].g, leds[i].b);
  }
  octo.show();
}

void setup() {
#if VERBOSE_DEBUG
  Serial.begin(115200);
#endif
  
  octo.begin();
  octo.show();

  Serial4.begin(BAUD_RATE);
  Serial4.addMemoryForRead(malloc(2048), 2048);

  // Poll twice as fast (every 500us) to handle the 1 Megabaud flow cleanly
  serialParserTimer.begin(parseSerialStream, 500);

  delay(1000);
}

void loop() {
  animate_tube_fluid();

#if VERBOSE_DEBUG
  unsigned long currentMillis = millis();
  if (currentMillis - lastPrintTime >= 500) {
    lastPrintTime = currentMillis;
    Serial.print("[ROLLING ENGINE] Good: ");
    Serial.print(goodPackets);
    Serial.print(" | Bad CRC: ");
    Serial.print(badChecksums);
    Serial.print(" | Vol: ");
    Serial.println(micA.masterVolume);
  }
#endif
}

void animate_tube_fluid() {
  fadeToBlackBy(leds, TOTAL_LEDS, 35);
  
  uint8_t currentVol = micA.masterVolume;
  int volumeBars = map(currentVol, 0, 255, 0, LEDS_PER_STRIP);
  
  for(int s = 0; s < NUM_STRIPS; s++) {
    for(int i = 0; i < volumeBars; i++) {
      leds[(s * LEDS_PER_STRIP) + i] |= CHSV(140 + (currentVol / 4), 240, 200); 
    }
  }
  
  showOctoBuffer();
}