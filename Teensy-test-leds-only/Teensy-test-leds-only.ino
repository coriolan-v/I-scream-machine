#include <OctoWS2811.h>

// ======================================================
// USER SETTINGS
// ======================================================

const uint8_t MAX_BRIGHTNESS = 32;  // 0-255

const int numPins = 12;

byte pinList[numPins] = {
  33, 34, 35, 36,
  37, 38, 39, 40,
  18, 19, 20, 21
};

const int ledsPerStrip = 384;

// ======================================================

const int bytesPerLED = 3;  // RGB

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

// One color per strip
uint32_t stripColors[numPins] = {
  0xFF0000, // Red
  0x00FF00, // Green
  0x0000FF, // Blue
  0xFFFF00, // Yellow
  0xFF00FF, // Magenta
  0x00FFFF, // Cyan
  0xFF8000, // Orange
  0x8000FF, // Purple
  0xFFFFFF, // White
  0x80FF00, // Lime
  0xFF0080, // Pink
  0x0080FF  // Sky Blue
};

uint32_t scaleBrightness(uint32_t color, uint8_t brightness)
{
  uint8_t r = ((color >> 16) & 0xFF) * brightness / 255;
  uint8_t g = ((color >> 8) & 0xFF) * brightness / 255;
  uint8_t b = (color & 0xFF) * brightness / 255;

  return ((uint32_t)r << 16) |
         ((uint32_t)g << 8)  |
         b;
}

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Serial.begin(115200);

  while (!Serial && millis() < 3000) {
  }

  Serial.println();
  Serial.println("OctoWS2811 Strip Test Starting");

  leds.begin();

  // Clear all LEDs
  for (int i = 0; i < leds.numPixels(); i++) {
    leds.setPixel(i, 0);
  }

  leds.show();

  Serial.print("Total Pixels = ");
  Serial.println(leds.numPixels());

  Serial.print("Strips = ");
  Serial.println(numPins);

  Serial.print("LEDs Per Strip = ");
  Serial.println(ledsPerStrip);
}

void loop()
{
  const int delay_us = 5000;

  // Clear all LEDs before starting
  for (int i = 0; i < leds.numPixels(); i++) {
    leds.setPixel(i, 0);
  }
  leds.show();

  // Fill strips one at a time
  for (int strip = 0; strip < numPins; strip++) {

    uint32_t color =
      scaleBrightness(stripColors[strip], MAX_BRIGHTNESS);

    Serial.println();
    Serial.println("================================");
    Serial.print("STARTING STRIP ");
    Serial.print(strip);
    Serial.print("  PIN ");
    Serial.println(pinList[strip]);
    Serial.println("================================");

    for (int pixel = 0; pixel < ledsPerStrip; pixel++) {

      int index = strip * ledsPerStrip + pixel;

      Serial.print("Pin=");
      Serial.print(pinList[strip]);

      Serial.print(" Strip=");
      Serial.print(strip);

      Serial.print(" LED=");
      Serial.print(pixel);

      Serial.print(" GlobalIndex=");
      Serial.println(index);

      leds.setPixel(index, color);
      leds.show();

      delayMicroseconds(delay_us);
    }

    delay(500);
  }

  Serial.println();
  Serial.println("TEST COMPLETE - CLEARING");

  // Clear everything
  for (int i = 0; i < leds.numPixels(); i++) {
    leds.setPixel(i, 0);
  }

  leds.show();

  delay(3000);
}