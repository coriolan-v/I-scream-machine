#include <OctoWS2811.h>

// Your 12 output pins
const int numPins = 12;
byte pinList[numPins] = {
  21, // Strip 1  (was 12)
  18, // Strip 2  (was 9)
  19, // Strip 3  (was 10)
  20, // Strip 4  (was 11)
  36, // Strip 5  (was 4)
  38, // Strip 6  (was 6)
  33, // Strip 7  (was 1)
  37, // Strip 8  (was 5)
  34, // Strip 9  (was 2)
  35, // Strip 10 (was 3)
  40, // Strip 11 (was 8)
  39  // Strip 12 (was 7)
};

const int ledsPerStrip = 192;

const int bytesPerLED = 3;
DMAMEM int displayMemory[ledsPerStrip * numPins * bytesPerLED / 4];
int drawingMemory[ledsPerStrip * numPins * bytesPerLED / 4];

const int config = WS2811_GRB | WS2811_800kHz;

OctoWS2811 leds(
  ledsPerStrip,
  displayMemory,
  drawingMemory,
  config,
  numPins,
  pinList
);

#define WHITE 0x202020

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  leds.begin();

  // Clear everything
  for (int i = 0; i < leds.numPixels(); i++) {
    leds.setPixel(i, 0);
  }

  // Strip N lights N+1 LEDs
  for (int strip = 0; strip < numPins; strip++) {

    int ledsToLight = strip + 1;

    for (int led = 0; led < ledsToLight; led++) {
      int pixelIndex = strip * ledsPerStrip + led;
      leds.setPixel(pixelIndex, WHITE);
    }
  }

  leds.show();
}

void loop() {
  // Nothing
}