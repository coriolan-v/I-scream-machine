#include <OctoWS2811.h>
#include <FastLED.h>

#define BAUD_RATE 1000000 
#define NUM_STRIPS 12
#define LEDS_PER_STRIP 200
#define TOTAL_LEDS (NUM_STRIPS * LEDS_PER_STRIP)

// --- COMPILATION FLAGS ---
#define VERBOSE_DEBUG 0  // Set to 1 for live telemetry, 0 to silence USB output

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

// Stream Parser State Variables
enum ParseState { STATE_LOOK_FOR_HEADER_HIGH, STATE_LOOK_FOR_HEADER_LOW, STATE_READ_PAYLOAD };
ParseState currentState = STATE_LOOK_FOR_HEADER_HIGH;

uint8_t packetBuffer[sizeof(RichAudioPacket)];
int bytesRead = 0;

// Telemetry Counters
uint32_t goodPackets = 0;
uint32_t badChecksums = 0;
unsigned long lastPrintTime = 0;

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

  // Listen on Pin 16 (Serial4)
  Serial4.begin(BAUD_RATE);
  Serial4.addMemoryForRead(malloc(1024), 1024);

  delay(1000);
#if VERBOSE_DEBUG
  Serial.println("\n--- TEENSY RECEIVER ACTIVE (VERBOSE DEBUG ON) ---");
#endif
}

void loop() {
  while (Serial4.available() > 0) {
    uint8_t incomingByte = Serial4.read();

    switch (currentState) {
      
      case STATE_LOOK_FOR_HEADER_HIGH:
        if (incomingByte == 0xAB) {
          packetBuffer[0] = incomingByte;
          currentState = STATE_LOOK_FOR_HEADER_LOW;
        }
        break;

      case STATE_LOOK_FOR_HEADER_LOW:
        if (incomingByte == 0xCD) {
          packetBuffer[1] = incomingByte;
          bytesRead = 2; 
          currentState = STATE_READ_PAYLOAD;
        } else {
          currentState = STATE_LOOK_FOR_HEADER_HIGH;
        }
        break;

      case STATE_READ_PAYLOAD:
        packetBuffer[bytesRead] = incomingByte;
        bytesRead++;

        if (bytesRead >= sizeof(RichAudioPacket)) {
          memcpy(&micA, packetBuffer, sizeof(RichAudioPacket));

          uint8_t verifyCheck = micA.masterVolume;
          for (int i = 0; i < 16; i++) {
            verifyCheck ^= micA.frequencyBins[i];
          }

          if (verifyCheck == micA.checksum) {
            goodPackets++;
            animate_tube(); 
          } else {
            badChecksums++;
          }

          currentState = STATE_LOOK_FOR_HEADER_HIGH;
        }
        break;
    }
  }

  // --- RECEIVER TELEMETRY DASHBOARD ---
#if VERBOSE_DEBUG
  unsigned long currentMillis = millis();
  if (currentMillis - lastPrintTime >= 500) {
    lastPrintTime = currentMillis;
    Serial.print("[TEENSY MONITOR] Valid Packets: ");
    Serial.print(goodPackets);
    Serial.print(" | Checksum Bad: ");
    Serial.print(badChecksums);
    Serial.print(" | Live Vol: ");
    Serial.println(micA.masterVolume);
  }
#endif
}

void animate_tube() {
  fadeToBlackBy(leds, TOTAL_LEDS, 40);
  
  int volumeBars = map(micA.masterVolume, 0, 255, 0, LEDS_PER_STRIP);
  for(int s=0; s<NUM_STRIPS; s++) {
    for(int i=0; i<volumeBars; i++) {
      leds[(s * LEDS_PER_STRIP) + i] = CHSV(160, 240, 200); 
    }
  }
  
  showOctoBuffer();
}