// 薄ピンクの光を加速度に合わせて明るさを調整する


// ここの1000000の値の変更で明るさの感度が変わる 大きくするほど感度が弱くなる
#define SENSITIVITY 1000000



// ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑変更していいのは　1000000　の値だけ！

// -----------------------------------------------
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
MPU6050 accelgyro;
// -----------------------------------------------
#include <Adafruit_NeoPixel.h>
#ifdef AVR
#include <avr/power.h>
#endif
const int PIN = 7;
const int NUMPIXELS = 3;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
// -----------------------------------------------
#define LED_White 12
#define LED_Blue 13
int16_t ax, ay, az;
int16_t gx, gy, gz;
float ax_lf_old=0.0,ax_lf=0.0,ax_hf=0.0;
float ay_lf_old=0.0,ay_lf=0.0,ay_hf=0.0;
float az_lf_old=0.0,az_lf=0.0,az_hf=0.0;
float a_alpha=0.99, a_beta=0.97;
float total_aa;
float gx_lf_old=0.0,gx_lf=0.0,gx_hf=0.0;
float gy_lf_old=0.0,gy_lf=0.0,gy_hf=0.0;
float gz_lf_old=0.0,gz_lf=0.0,gz_hf=0.0;
float g_alpha=0.99, g_beta=0.97;
float total_gg;
float aaa,taa;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pixels.begin();
  delay(40);
  pinMode(LED_Blue, OUTPUT);
  pinMode(LED_White, OUTPUT);
  accelgyro.initialize();
  Serial.println("start");
}
void loop() {
  total_aa=0.0;
  for(int j=0; j<51; j++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_lf = a_alpha*ax_lf_old + (1-a_alpha)*(float)ax;  // low pass filter 除去
    ax_hf = a_beta*(ax_hf+ax_lf-ax_lf_old);  // high pass filter 除去
    ax_lf_old=ax_lf;
    ay_lf = a_alpha*ay_lf_old + (1-a_alpha)*(float)ay;
    ay_hf = a_beta*(ay_hf+ay_lf-ay_lf_old);
    ay_lf_old=ay_lf;
    az_lf = a_alpha*az_lf_old + (1-a_alpha)*(float)az;
    az_hf = a_beta*(az_hf+az_lf-az_lf_old);
    az_lf_old=az_lf;
    total_aa += (ax_hf * ax_hf + ay_hf * ay_hf + az_hf * az_hf);  // 加速度の合成
  }
  aaa = total_aa / 51;
  Serial.println(aaa);

  float brightness_R = map(aaa, 0, SENSITIVITY, 30, 255);
  float brightness_GB = map(aaa, 0, SENSITIVITY, 0, 225);
  change_ledcolor(brightness_GB, brightness_R, brightness_GB);
}

void change_ledcolor(int r, int g, int b) {
  for(int i=0; i<=NUMPIXELS; i++){
    pixels.setPixelColor(i, pixels.Color(r, g, b));
    pixels.show();
  }
}