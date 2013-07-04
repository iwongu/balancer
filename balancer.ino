#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
//#define DEBUG__

// Constants for tuing complementary filter, PID, and sampling rate.
const double W = 0.98;
const double Kp = 100;
const double Ki = 0;
const double Kd = 30;
const int Hz = 50;

// Constants for H-bridge motor driver.
const int controlPin[2][2] = {{8, 1}, {0, 7}};
const int speedPin[] = {11, 3};
const int FORWARD = 0;
const int BACKWARD = 1;

MPU6050 accel;

double gyZero = 0.0;
double angleZero = 0.0;
double complementaryAngle = 0.0;
double lastAngle = 0.0;
double angleSum = 0.0;

long timer;

void setup() {
#ifdef DEBUG__
  Serial.begin(115200); 
#endif
  
  Wire.begin();
  accel.initialize();
  accel.setFullScaleGyroRange(1);  // +/- 500 degrees/s, 65.5 LSB/deg/s
  
  for (int i = 0; i < 2; i++) {
    digitalWrite(speedPin[i], LOW);
    for (int j = 0; j < 2; j++) {
      pinMode(controlPin[i][j], OUTPUT);
    }
    pinMode(speedPin[i], OUTPUT);
  }

  delay(1000);  // You need to make your bot up right within 1 second. :)
  calibrateSensors();
 
  timer = micros();
}

void loop() {
  long current = micros();
  double dt = current - timer;
  
  if (dt < 1000000 / Hz) {
    return;
  }
  
  timer = current;
  
  int16_t ax, ay, az, gx, gy, gz;
  accel.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  double rate = getRate(gy);
  double angle = getAngle(ax, az);
  complementaryAngle = W * (complementaryAngle + rate * dt / 1000000.0) + (1 - W) * angle;
  
  double diff = (lastAngle - complementaryAngle) * dt / 1000000.0;
  angleSum += complementaryAngle;
  lastAngle = complementaryAngle;
  
  double output = Kp * complementaryAngle + Ki * angleSum + Kd * diff;
  

#ifdef DEBUG__
  Serial.print(rate);
  Serial.print("\t");
  Serial.print(angle);
  Serial.print("\t");
  Serial.print(complementaryAngle);
  Serial.print("\t");
  Serial.println(output);
#endif

  run(output);
}

void turnOn(int index, int dir, int speed) {
#ifndef DEBUG__
  digitalWrite(controlPin[index][0], dir == FORWARD ? LOW : HIGH);
  digitalWrite(controlPin[index][1], dir == FORWARD ? HIGH : LOW);
  analogWrite(speedPin[index], speed);
#endif
}

void run(int output) {
  int dir = output > 0 ? FORWARD : BACKWARD;
  int speed = max(0, min(255, abs(output)));
#ifdef DEBUG__
  Serial.print(dir);
  Serial.print("\t");
  Serial.println(speed);
#endif
  for (int i = 0; i < 2; i++) {
    turnOn(i, dir, speed);
  }
}

void calibrateSensors() {
  gyZero = 0.;
  angleZero = 0.;
  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    accel.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyZero += gy;
    angleZero += degrees(atan2(ax, az + 4096));
    delay(10);
  }
  gyZero /= 100.;
  angleZero /= 100.;
}

double getRate(int16_t gy) {
  return -((double) gy - gyZero) / 65.5;  // See MPU6050::getRotation() comment for 65.5.
}

double getAngle(int16_t ax, int16_t az) {
  return degrees(atan2(ax, az + 4096)) - angleZero;  // See MPU6050::getAcceleration for 4096.
}

