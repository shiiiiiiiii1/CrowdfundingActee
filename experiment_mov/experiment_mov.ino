/*
 使用方法：下記の用途によってコメントアウトするところを変える。
 例）普通のActeeとしてランダムに色を投げたい場合は、[ #define RANDOM ]のみコメントアウトを外す。
 - [#define RANDOM]：普通のActeeとしてランダムに色を変えたい時
 - [#define COLOR5]：投げた時に青を出したい時
 - [#define COLOR4]：投げた時に緑を出したい時
 - [#define COLOR3]：投げた時に黄色を出したい時
 - [#define COLOR2]：投げた時にオレンジを出したい時
 - [#define COLOR1]：投げた時にピンクを出したい時
 - [#define COLOR_1]：ピンクが三回連続で投げられた時のエフェクト
 - [#define COLOR_2]：オレンジが三回連続で投げられた時のエフェクト
 - [#define COLOR_3]：黄色が三回連続で投げられた時のエフェクト
 - [#define COLOR_4]：緑が三回連続で投げられた時のエフェクト
 - [#define COLOR_5]：青が三回連続で投げられた時のエフェクト
 - [#define COLOR_6]：前職が一回ずつで投げられた時のエフェクト
*/
// #define RANDOM
// #define COLOR5
// #define COLOR4
// #define COLOR3
// #define COLOR2
// #define COLOR1
// #define COLOR_1
// #define COLOR_2
// #define COLOR_3
// #define COLOR_4
// #define COLOR_5
// #define COLOR_6
/* ----------------------------------------------------------- */

#ifdef AVR
  #include <avr/power.h>
#endif
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
#include <Adafruit_NeoPixel.h>

#define PIN 7
#define NUMPIXELS 6

MPU6050 mpu;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define ROTATION_STORAGE 12   // 一定回数回転したらの回数
bool flying;   // 飛んでいるかの判定の変数
char rotation, before_rotation;   // 回転方向を格納する変数
char rotation_state;   // 回転回数を格納する変数
int before_angle;   // 一回前の角度

char count = 0;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  pinMode(LED_PIN, OUTPUT);
  pixels.begin();
  setupActee();   // 追記セットアップ
}


void loop() {
#ifdef RANDOM
  int a = random(1, 6);   // 1.0~6.0
  movFlyingColor(a);
#endif

#ifdef COLOR5
  movFlyingColor(5);
#endif

#ifdef COLOR4
  movFlyingColor(4);
#endif

#ifdef COLOR3
  movFlyingColor(3);
#endif

#ifdef COLOR2
  movFlyingColor(2);
#endif

#ifdef COLOR1
  movFlyingColor(1);
#endif

#ifdef COLOR_1
  changeColor(-1);
#endif

#ifdef COLOR_2
  changeColor(-2);
#endif

#ifdef COLOR_3
  changeColor(-3);
#endif

#ifdef COLOR_4
  changeColor(-4);
#endif

#ifdef COLOR_5
  changeColor(-5);
#endif

#ifdef COLOR_6
  changeColor(-6);
#endif
}


/*
 セットアップ関数
 - 飛んでる判定値の初期化
 - 回転方向の変数を初期化
    - 1：時計回り
    - 0：回ってない
    - -1：反時計回り
 - 一回前の回転方向の変数を初期化
 - 色変化変数の初期化
 - 連続で投げられた回数の変数の初期化
 - 一回前の角度の変数の初期化
 - 起動色の表示
 引数：なし
 戻り値：なし
*/
void setupActee() {
  flying = false;
  rotation = 0;
  before_rotation = 0;
  before_angle = 0;
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
  delay(1000);
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, 100, 100, 100);
  }
  pixels.show();
}

/*
 フリスビーが飛んだかどうかを判定するための関数
 - 前の角度と今の角度の差を計算
 - 現在角度をbefore変数に格納
 - 一定の値が続いたら回転したと判断
 - flying変数に回転状態を格納
 - どっち回転かを測定
 - rotation変数に回転方向を格納
 - 一定回数同じ方向に回転したら
 引数：一個前の回転角度 現在の回転角度
 戻り値：飛んでたらtrue、飛んでなかったらfalse
*/
bool stateActee(int before, int now) {
  int difference = before - now;
  if (difference >= 300) rotation = before_rotation;   // 反時計回り
  if (300 > difference && difference >= 6) rotation = 1;   // 時計回り
  if (6 > difference && difference > -6) rotation = 0;
  if (-6 >= difference && difference > -300) rotation = -1;   // 反時計回り
  if (-300 >= difference) rotation = before_rotation;   // 時計回り
  before_angle = now;

  if (flying) {   // 飛んでる時の処理
    if (before_rotation != rotation) {   // キャッチしたら
      rotation_state = 0;
      return false;
    }
    return true;
  } else {
    if (rotation == 0) {   // 動いてなかったら
      rotation_state = 0;
      return false;
    }
    if (before_rotation == rotation) {   // 前回と同じ方向に回ったら
      rotation_state ++;
      if (rotation_state == ROTATION_STORAGE) {   // YAW_STORAGE分、同じ方向に回り続けた -> 投げた！
        return true;
      }
    } else {   // 飛んでない時の処理
      rotation_state = 0;
    }
    before_rotation = rotation;
    return false;
  }
}

/*
 Acteeの色変化のための関数
 引数：色を指定するための変数
    - -6：全色連続
    - -5：青3回連続
    - -4：緑3回連続
    - -3：黄色3回連続
    - -2：オレンジ3回連続
    - -1：赤ピンク3回連続
    - 0 ：スタンダードの色
    - 1 ：評価での赤ピンク
    - 2 ：評価でのオレンジ
    - 3 ：評価での黄色
    - 4 ：評価での緑
    - 5 ：評価での青色
 戻り値：なし
*/
void changeColor(char color_type) {
//  int delay_time = 3000;
  int delay_time = 1000;
  switch(color_type) {
    case 0:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 100, 100, 100);
      }
      pixels.show();
      break;
    case 1:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 255, 50, 60);
      }
      pixels.show();
      break;
    case 2:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 255, 80, 0);
      }
      pixels.show();
      break;
    case 3:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 251, 211, 28);
      }
      pixels.show();
      break;
    case 4:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 56, 151, 35);
      }
      pixels.show();
      break;
    case 5:
      for (int i = 0; i < NUMPIXELS; i++) {
//        pixels.setPixelColor(i, 135, 206, 250);
        pixels.setPixelColor(i, 10, 255, 255);
      }
      pixels.show();
      break;
    case -1:
      for (int i = 0; i < NUMPIXELS; i++) {
        if (i % 2 == 0) {
          pixels.setPixelColor(i, 255, 50, 60);
        } else {
          pixels.setPixelColor(i, 0, 0, 0);
        }
      }
      pixels.show();
      delay(delay_time / 2);
      for (int i = 0; i < NUMPIXELS; i++) {
        if (i % 2 == 0) {
          pixels.setPixelColor(i, 0, 0, 0);
        } else {
          pixels.setPixelColor(i, 234, 145, 152);
        }
      }
      pixels.show();
      delay(delay_time / 2);
      break;
    case -2:
      for (int i = 0; i < NUMPIXELS; i++) {
        for (int j = 0; j < NUMPIXELS; j++) {
          pixels.setPixelColor(j, 0, 0, 0);
        }
        pixels.setPixelColor(i, 255, 80, 0);
        pixels.show();
        delay(delay_time / NUMPIXELS);
      }
      break;
    case -3:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 0, 0, 0);
      }
      pixels.setPixelColor(0, 255, 255, 0);
      pixels.show();
      delay(delay_time / NUMPIXELS);
      pixels.setPixelColor(0, 0, 0, 0);
      pixels.setPixelColor(2, 255, 255, 0);
      pixels.show();
      delay(delay_time / NUMPIXELS);
      pixels.setPixelColor(2, 0, 0, 0);
      pixels.setPixelColor(4, 255, 255, 0);
      pixels.show();
      delay(delay_time / NUMPIXELS);
      pixels.setPixelColor(4, 0, 0, 0);
      pixels.setPixelColor(1, 255, 255, 0);
      pixels.show();
      delay(delay_time / NUMPIXELS);
      pixels.setPixelColor(1, 0, 0, 0);
      pixels.setPixelColor(3, 255, 255, 0);
      pixels.show();
      delay(delay_time / NUMPIXELS);
      pixels.setPixelColor(3, 0, 0, 0);
      pixels.setPixelColor(5, 255, 255, 0);
      pixels.show();
      delay(delay_time / NUMPIXELS);
      break;
    case -4:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 0, 0, 0);
      }
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 0, 255, 0);
        pixels.show();
        delay(delay_time / NUMPIXELS);
      }
      break;
    case -5:
      for (int i = 0; i < NUMPIXELS; i++) {
        for (int j = 0; j < NUMPIXELS; j++) {
          pixels.setPixelColor(j, 10, 255, 255);
        }
        pixels.setPixelColor(i, 20, 20, 255);
        if (i != 5) {
          pixels.setPixelColor(i+1, 20, 20, 255);
        } else {
          pixels.setPixelColor(0, 20, 20, 255);
        }
        pixels.show();
        delay(delay_time / NUMPIXELS);
      }
     break;
    case -6:
      pixels.setPixelColor(0, 255, 0, 0);
      pixels.setPixelColor(1, 255, 255, 0);
      pixels.setPixelColor(2, 0, 255, 0);
      pixels.setPixelColor(3, 0, 255, 255);
      pixels.setPixelColor(4, 0, 0, 255);
      pixels.setPixelColor(5, 255, 0, 255);
      pixels.show();
      delay(delay_time / 2);
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 0, 0, 0, 0);
      }
      pixels.show();
      delay(delay_time / 2);
     break;
  }
}

// 動画用関数
void movFlyingColor(char c) {
  if (!dmpReady) return;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    /* ヨーピッチロールを取得 */
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    int yaw_angle = (int) (ypr[0] * 180/M_PI);   // 回転角度の測定
    bool flying_now = stateActee(before_angle, yaw_angle);
    if (!flying && flying_now) {   // 投げたら
      flying = true;
      changeColor(c);
      count++;
    } else if (flying && !flying_now) {   // キャッチしたら
      flying = false;
      rotation = 0;
      if (count != 3) {
        changeColor(0);
      } else {
        count = 0;
        changeColor(c*=-1);
        changeColor(c*=-1);
        changeColor(0);
      }

    }
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
