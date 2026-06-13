#include <OctoWS2811.h>

// Your 12 output pins
const int numPins = 12;
byte pinList[numPins] = {
  33, 34, 35, 36, 37, 38,
  39, 40, 18, 19, 20, 21
};

const int ledsPerStrip = 120;

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