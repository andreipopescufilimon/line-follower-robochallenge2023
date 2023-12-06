/*
*  Fast Line Followe Code - NerdBot Poseidon
*  Competing in Robochallenge 2023
*
*  Sum off all errors or only on last six errors for KI.
*  
*  Author: Popescu Filimon Andrei Cosmin
*  Last update: 4th November, 2023
*/

// Include necessary libraries
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>


const int BUTTON_PIN = 8;  // the number of the pushbutton pin
const int startpin = 13;   //create variable and declare where Micro Start receiver attached

// motor driver pins
#define AIN1 5
#define AIN2 4
#define BIN1 6
#define BIN2 7
#define PWMA 3
#define PWMB 11
//#define STBY 9

// sensor declare
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Define your custom minimum and maximum thresholds for each sensor
const uint16_t minThreshold[SensorCount] = { 35, 35, 35, 35, 35, 35, 35, 35 };
const uint16_t maxThreshold[SensorCount] = { 905, 905, 905, 905, 905, 905, 905, 905 };



//PID
#define KP 0.3
#define KI 0.001
#define KD 10




uint16_t position = 3500;
int P, I, D;
int motorspeed, motorspeeddif, cnt;
int basespeed = 230;
int speedp = 250, speedn = -240;

int error = 0, error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0, lastError = 0;
int middlepoint = 3500;
int exitcond;

int debug = 1;
int calibration = 0;


void setup() {

  pinMode(startpin, INPUT);  //set startpin (pin 13 on Arduino) as input pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // sensors init
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5, A6, A7 }, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);

  // motor set
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  ////pinMode(STBY, OUTPUT);


  //Button press 1 - Calibration
  while (digitalRead(BUTTON_PIN) == 1) {
    //wait
  }

  if (debug == 0) {
    Serial.begin(9600);
    Serial.println("Calibration Start");
  }

  if (calibration == 0) {
    //manual calibration
    for (uint16_t i = 0; i < 100; i++) {
      qtr.calibrate();
    }
    delay(1000);
    //auto calibration
    cnt = 0;
    while (cnt < 5) {
      for (int i = 0; i < 10; i++) {
        motors_move(0, -200);
        delay(21);
        motors_move(0, 0);
        qtr.calibrate();
      }
      for (int i = 0; i < 20; i++) {
        motors_move(0, 200);
        delay(20);
        motors_move(0, 0);
        qtr.calibrate();
      }
      for (int i = 0; i < 10; i++) {
        motors_move(0, -200);
        delay(21);
        motors_move(0, 0);
        qtr.calibrate();
      }
      cnt++;
    }
    motors_move(0, 0);
  }

  if (debug == 0) {
    Serial.println("Calibration End");
    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(5000);
  }

  //Button press 2 - Start
  while (digitalRead(BUTTON_PIN) == 1 && digitalRead(startpin) == 0) {
    // wait
    if (debug == 0) {
      Serial.println("standby");
      Serial.print(digitalRead(startpin));
      Serial.println(digitalRead(BUTTON_PIN));
    }
  }
  if (debug == 0) {
    Serial.println("Start");
  }
  delay(2000);
}


void loop() {


  // line position = error
  position = qtr.readLineBlack(sensorValues);
  error = position - middlepoint;
  lastError = error;
  /*`error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = error;*/

  // 2nd check
  if (position <= 1000) {
    inert_stop();
    motors_move(speedp, speedn);
  } else if (position >= 6000) {
    inert_stop();
    motors_move(speedn, speedp);
  }

  while (position < 1000 || position > 6000) {
    position = qtr.readLineBlack(sensorValues);
  }
  // 2nd check end

  if (debug == 0) {
    // sensors values
    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println(position);
  }

  // line position
  position = qtr.readLineBlack(sensorValues);

  // PID
  error = position - middlepoint;
  P = error;
  I = I + error;
  //I = error6 + error5 + error4 + error3 + error2 + error1 + error;
  D = error - lastError;
  lastError = error;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = error;

  motorspeeddif = P * KP + I * KI + D * KD;

  if (motorspeeddif > basespeed)
    motorspeeddif = basespeed;
  else if (motorspeeddif < -basespeed)
    motorspeeddif = -basespeed;

  if (position >= 3300 && position <= 3700) {
    motors_move(240, 240);
  } else {
    if (motorspeeddif < 0)
      motors_move(basespeed, basespeed + motorspeeddif);
    else if (motorspeeddif >= 0)
      motors_move(basespeed - motorspeeddif, basespeed);
  }


  //Button press 3 - Stop
  if (digitalRead(BUTTON_PIN) == 0) {
    motors_move(0, 0);
    if (debug == 0) {
      Serial.println("Stop");
    }
    while (1)
      ;
  }
}

void inert_stop() {
  motors_move(-250, -250);
  int time1, time2;
  time1 = millis();
  while (time2 - time1 < 25) {
    motors_move(-250, -250);
    time2 = millis();
  }
}

void motors_move(int leftMotorSpeed, int rightMotorSpeed) {
  if (leftMotorSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    leftMotorSpeed = leftMotorSpeed * (-1);
  }
  analogWrite(PWMA, leftMotorSpeed);

  if (rightMotorSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    rightMotorSpeed = rightMotorSpeed * (-1);
  }
  analogWrite(PWMB, rightMotorSpeed);
}
