#include <Arduino.h>
#include <I2S.h>
#include <arduinoFFT.h>
#include <SerialPIO.h>  // Required for PIO-based Serial on D5


// ---------- Pins ----------
#define PIN_WS   3
#define PIN_SD   4
#define PIN_SCK  2

I2S i2sInput(INPUT);

#define AUDIO_BLOCK_SIZE 64

float latestRmsVolume = 0;
float latestPeak = 0;

void setup()
{
    Serial.begin(115200);
    delay(500);

    i2sInput.setBCLK(PIN_SCK);
    i2sInput.setDATA(PIN_SD);
    i2sInput.setBitsPerSample(32);
    i2sInput.setBuffers(6, 256);

    if (!i2sInput.begin(16000))
    {
        Serial.println("I2S begin failed");
        while (1);
    }

    Serial.println("Microphone running");
}

void loop()
{
    double sumSquares = 0;
    float peak = 0;

    static float dc = 0;

    for (int i = 0; i < AUDIO_BLOCK_SIZE; i++)
    {
        int32_t left = 0;
        int32_t right = 0;

        i2sInput.read32(&left, &right);

        float sample = (float)(left >> 8) / 8388608.0f;

        dc = dc * 0.995f + sample * 0.005f;
        sample -= dc;

        float mag = fabsf(sample);

        if (mag > peak)
            peak = mag;

        sumSquares += sample * sample;
    }

    latestRmsVolume = sqrt(sumSquares / AUDIO_BLOCK_SIZE);
    latestPeak = peak;

    Serial.print("RMS:");
    Serial.print(latestRmsVolume * 10000.0f);

    Serial.print(",");

    Serial.print("Peak:");
    Serial.print(latestPeak * 10000.0f);

    Serial.println();

    delay(25);
}