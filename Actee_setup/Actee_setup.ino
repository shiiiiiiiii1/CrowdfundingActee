#include <Adafruit_NeoPixel.h>
#ifdef AVR
  #include <avr/power.h>
#endif

#define PIN 7
#define NUMPIXELS 6

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
    pixels.begin();

    pixels.setPixelColor(0, 255, 0, 0);
    pixels.show();
    delay(100);
    pixels.setPixelColor(1, 255, 255, 0);
    pixels.show();
    delay(100);
    pixels.setPixelColor(2, 0, 255, 0);
    pixels.show();
    delay(100);
    pixels.setPixelColor(3, 0, 255, 255);
    pixels.show();
    delay(100);
    pixels.setPixelColor(4, 0, 0, 255);
    pixels.show();
    delay(100);
    pixels.setPixelColor(5, 255, 0, 255);
    pixels.show();
}

void loop() {
}