// RGBの値自分で変えて実験できるよプログラム

/*
  http://sun-nya.com/wp-content/uploads/2014/03/rgbcol.html
  このサイトのRGBの値と同じ値にしたら、画面上の色とLEDで光らした時の色がどんな感じに違うかわかる！
  要するに、ここの値いじればLEDの色変わるよってことです！笑
*/
int R = 255;
int G = 255;
int B = 255;

#include <Adafruit_NeoPixel.h>
#ifdef AVR
#include <avr/power.h>
#endif
int ms_lag = 20;
const int PIN = 7;
const int NUMPIXELS = 3;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin();
  change_ledcolor(R, G, B);
}

void loop() {
}

void change_ledcolor(int r, int g, int b) {
  for(int i=0; i<=NUMPIXELS; i++){
  pixels.setPixelColor(i, pixels.Color(r, g, b));
  pixels.show();
  }
}