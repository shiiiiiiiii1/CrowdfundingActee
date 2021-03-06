// グローバル変数で必要なやつ
// 一個前の角度の値：float before_yaw;
const uint8_t YAW_STORAGE = 12;
// int yaw[YAW_STORAGE];   // float型をint型に変換してから格納する
int before_direction_of_rotation = 0;   // 前はどっちに回ったか保管
uint8_t sum_state = 0;  // 計測結果の蓄積変数
bool throwing = false;
int before_val = 0;

#include <Adafruit_NeoPixel.h>
#ifdef AVR
  #include <avr/power.h>
#endif
// const int PIN = 7;
// const int NUMPIXELS = 6;
#define PIN 7
#define NUMPIXELS 6

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int color_type = 0;   // 色判定
int H = 255;
// #define SENSITIVITYMAX 10000
#define SENSITIVITYMAX 10000
#define SENSITIVITYMIN 0

int good_count = 0;

float before_total_acceleration = 0.0;

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license (マサチューセッツ工科大学の代表的なソフトウェアライセンス)
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>

#include <MPU6050_6Axis_MotionApps20.h>
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
// 四元数を使いたい時は、コメントアウトを外す。
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
// オイラー角を使いたいは、コメントアウトを外す。
// ジンバルの詳細：https://ja.wikipedia.org/wiki/%E3%82%B8%E3%83%B3%E3%83%90%E3%83%AB
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
// 重力を除去する。基準系(https://ja.wikipedia.org/wiki/%E5%9F%BA%E6%BA%96%E7%B3%BB)を調整する。
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
// ティーポットデモとして使用するためのやつ
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    pixels.begin();
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

// ------------------------------------------------------------------------------------
    Serial.print("mpuInterrupt:"); Serial.print(mpuInterrupt);
    Serial.print(",  fifoCount:"); Serial.print(fifoCount);
    Serial.print(",  packetSize:"); Serial.println(packetSize);
    // Serial.println("");
    // Serial.println(mpu.getZAccelOffset());
    // Serial.print("x:"); Serial.print(mpu.getXAccelOffset());
    // Serial.print(", y:"); Serial.print(mpu.getYAccelOffset());
    // Serial.print(", z:"); Serial.println(mpu.getZAccelOffset());
    // Serial.print("x:"); Serial.print(mpu.getXGyroOffset());
    // Serial.print(", y:"); Serial.print(mpu.getYGyroOffset());
    // Serial.print(", z:"); Serial.println(mpu.getZGyroOffset());
    // Serial.println("");
  // for(int i = 0; i <= NUMPIXELS; i++) {
    aChangeLedColor(0, 255, 0, 0);
    delay(100);
    aChangeLedColor(1, 255, 255, 0);
    delay(100);
    aChangeLedColor(2, 0, 255, 0);
    delay(100);
    aChangeLedColor(3, 0, 255, 255);
    delay(100);
    aChangeLedColor(4, 0, 0, 255);
    delay(100);
    aChangeLedColor(5, 255, 0, 255);
    delay(1000);
    changeLedcolor(0, 0, 0);
  // }
// ------------------------------------------------------------------------------------
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    // while (!mpuInterrupt && fifoCount < packetSize) {
    //     // other program behavior stuff here
    //     // .
    //     // .
    //     // .
    //     // if you are really paranoid you can frequently test in between other
    //     // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    //     // while() loop to immediately process the MPU data
    //     // .
    //     // .
    //     // .
    // }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            // Serial.print("ypr\t");
            // Serial.println(ypr[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);

            if (color_type == 0 && throwing) {
              colorSet();
            }

            switch(color_type) {
              case 0:
                havingColorChange();
                break;
              default:
                flyingColorChange();
                break;
            }

            switch( frisbeeState() ) {
              case 1:
                ledOn();
                break;
              case -1:
                ledOff();
                break;
            }

        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif

        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

/*
================================================================
===                    MINE PROGRAM                          ===
================================================================
*/

char frisbeeState() {
  int yaw_angle = (int) (ypr[0] * 180/M_PI);

  int direction_of_rotation = directionOfRotation(before_val, yaw_angle);
  before_val = yaw_angle;

  if (throwing == false) {   // 投げてなかった時の処理
    if (direction_of_rotation == 0) {   // 動いてなかったら
      sum_state = 0;
      return 0;
    }
    if (before_direction_of_rotation == direction_of_rotation) {   // 前回と同じ方向に回ったら
      sum_state ++;
      if (sum_state == YAW_STORAGE) return 1;   // YAW_STORAGE分、同じ方向に回り続けた -> 投げた！
    } else {
      sum_state = 0;
    }
    before_direction_of_rotation = direction_of_rotation;
    return 0;
  } else {   // 投げてる時の処理
    if (before_direction_of_rotation != direction_of_rotation) {
      sum_state = 0;
      return -1;
    }
    return 0;
  }
}

int directionOfRotation(int before, int now) {   // 前回の値と今回の値から回転方向を計算
  int num = before - now;
  if (num >= 300) return before_direction_of_rotation;   // 反時計回り
  if (300 > num && num >= 6) return 1;   // 時計回り
  if (6 > num && num > -6) return 0;
  if (-6 >= num && num > -300) return -1;   // 反時計回り
  if (-300 >= num) return before_direction_of_rotation;   // 時計回り
}

void ledOn() {
  Serial.println("LED ON !!!!!");
  throwing = true;
}

void ledOff() {
  Serial.println("LED OFF !!!!!");
  if (throwing && good_count) {
    changeLedcolorGood(good_count);
    delay(1000);
  }
  throwing = false;
  color_type = 0;
  H = 255;
}

void colorSet() {
  float pitch = ypr[1] * 180/M_PI;
  float roll = ypr[2] * 180/M_PI;
  float pr = sqrt(pitch*pitch + roll*roll);
  Serial.println(pr);
  if ((int)pr > 40) {
    color_type = 1;
    good_count = 0;
  }
  if (40 >= (int)pr && (int)pr > 20) {
    color_type = 2;
    good_count = 0;
  }
  if (20 >= (int)pr) {
    color_type = 3;
    good_count += 1;
  }
}

void havingColorChange() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  float x = aaReal.x;
  float y = aaReal.y;
  float z = aaReal.z;
  float total_acceleration = sqrt(x * x + y * y + z * z);
  float total_acceleration_difference = abs(before_total_acceleration - total_acceleration);
  int brightness_main = (int)map(total_acceleration_difference, SENSITIVITYMIN, SENSITIVITYMAX, 40, 255);
  int brightness_sub = (int)map(total_acceleration_difference, SENSITIVITYMIN, SENSITIVITYMAX, 0, 255);
  // Serial.print("before_total_acceleration:"); Serial.println(before_total_acceleration);
  // Serial.print("total_acceleration:"); Serial.println(total_acceleration);
  // Serial.print("total_acceleration_difference:"); Serial.println(total_acceleration_difference);
  Serial.print("brightness_main:"); Serial.println(brightness_main);
  Serial.print("brightness_sub:"); Serial.println(brightness_sub);
  Serial.println();
  changeLedcolor(brightness_sub, brightness_main, brightness_sub);
  before_total_acceleration = total_acceleration;
}

void flyingColorChange() {
  if (0 < H && H <= 255) {
    switch(color_type) {
      case 1:
        if (H > 50) {
          changeLedcolor(255, H, H);   // 255, 50, 50
        } else {
          changeLedcolor(255, 50, 50);   // 255, 50, 50
        }
        break;
      case 2:
        changeLedcolor(255, 255, H);
        break;
      case 3:
        if (H > 150) {
          changeLedcolor(H, H, 255);
        } else {
          changeLedcolor(H, 150, 255);   // 0, 150, 255
        }
        break;
    }
    // Serial.print("color type : "); Serial.println(color_type);
    H -= 10;
  }
}

void aChangeLedColor(int count, int r, int g, int b) {
    pixels.setPixelColor(count, r, g, b);
    pixels.show();
}

void changeLedcolor(int r, int g, int b) {
  for(int i = 0; i <= NUMPIXELS; i++) {
    aChangeLedColor(i, r, g, b);
  }
}

void changeLedcolorGood(int good_count) {
  switch(good_count) {
    case 1:
      aChangeLedColor(0, 255, 0, 0);
      break;
    case 2:
      aChangeLedColor(0, 255, 0, 0);
      delay(500);
      aChangeLedColor(1, 255, 255, 0);
      break;
    case 3:
      aChangeLedColor(0, 255, 0, 0);
      delay(500);
      aChangeLedColor(1, 255, 255, 0);
      delay(500);
      aChangeLedColor(2, 0, 255, 0);
      break;
    case 4:
      aChangeLedColor(0, 255, 0, 0);
      delay(500);
      aChangeLedColor(1, 255, 255, 0);
      delay(500);
      aChangeLedColor(2, 0, 255, 0);
      delay(500);
      aChangeLedColor(3, 0, 255, 255);
      break;
    case 5:
      aChangeLedColor(0, 255, 0, 0);
      delay(500);
      aChangeLedColor(1, 255, 255, 0);
      delay(500);
      aChangeLedColor(2, 0, 255, 0);
      delay(500);
      aChangeLedColor(3, 0, 255, 255);
      delay(500);
      aChangeLedColor(4, 0, 0, 255);
      break;
    case 6:
      aChangeLedColor(0, 255, 0, 0);
      delay(500);
      aChangeLedColor(1, 255, 255, 0);
      delay(500);
      aChangeLedColor(2, 0, 255, 0);
      delay(500);
      aChangeLedColor(3, 0, 255, 255);
      delay(500);
      aChangeLedColor(4, 0, 0, 255);
      delay(500);
      aChangeLedColor(5, 255, 0, 255);
      break;
    default:
      break;
  }
}