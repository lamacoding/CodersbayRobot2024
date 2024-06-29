#include <SoftwareSerial.h>
#include <Servo.h>
#include "hcsr04.h"
//#include "bt.h"

// Motor pins
#define leftMotorsForward 40
#define leftMotorsBackward 41
#define leftMotorPWM 8

#define rightMotorsForward 52
#define rightMotorsBackward 53
#define rightMotorPWM 9

// parameters
unsigned short motorSpeed = 200;
const int crashDistance = 500;

char c = ' ';
bool NL = true;

SoftwareSerial BTserial(6, 7);

Servo sensorServo;

void setup() {
  pinMode(13, OUTPUT);  //built in LED

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  sensorServo.attach(4);

  digitalWrite(trig, HIGH);  //Turn off sensor signal

  Serial.begin(9600);
  // set the data rate for the SoftwareSerial port

  BTserial.begin(9600);

  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorsForward, OUTPUT);
  pinMode(rightMotorsBackward, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorsForward, OUTPUT);
  pinMode(leftMotorsBackward, OUTPUT);

  analogWrite(leftMotorPWM, motorSpeed);
  analogWrite(rightMotorPWM, motorSpeed);

  // initially center the sensor
  sensorServo.write(90);
  delay(300);
}



void loop() {
   // Read from the Bluetooth module and send to the Arduino Serial Monitor
  if (BTserial.available()) {
    c = BTserial.read();
    Serial.write(c);
  }


  // Read from the Serial Monitor and send to the Bluetooth module
  if (Serial.available()) {
    c = Serial.read();
    BTserial.write(c);

    // Echo the user input to the main window. The ">" character indicates the user entered text.
    if (NL) {
      Serial.print(">");
      NL = false;
    }
    Serial.write(c);
    if (c == 10) { NL = true; }
  }
  int currentDistance = getDistance();

  if (currentDistance > crashDistance) {
    analogWrite(leftMotorPWM, motorSpeed);
    analogWrite(rightMotorPWM, motorSpeed);
    driveForward();
  }

  else {
    stopMotors();
    delay(500);
    driveBackwards();
    delay(250);
    stopMotors();

    // sensor right
    sensorServo.write(0);
    delay(800);
    int distanceRight = getDistance();

    //sensor left
    sensorServo.write(180);
    delay(1000);
    int distanceLeft = getDistance();
    delay(50);
    sensorServo.write(90);

    if (distanceRight > distanceLeft) {
      turnRight();
    } else {
      turnLeft();
    }

    driveForward();
  }

  delay(10);
}


void stopMotors() {
  Serial.write("stop\n");

  digitalWrite(rightMotorsForward, LOW);
  digitalWrite(leftMotorsForward, LOW);
  digitalWrite(rightMotorsBackward, LOW);
  digitalWrite(leftMotorsBackward, LOW);
}

void driveForward() {
  digitalWrite(rightMotorsForward, HIGH);
  digitalWrite(leftMotorsForward, HIGH);
  digitalWrite(rightMotorsBackward, LOW);
  digitalWrite(leftMotorsBackward, LOW);
}

void driveBackwards() {
  Serial.write("backwards\n");

  digitalWrite(rightMotorsForward, LOW);
  digitalWrite(leftMotorsForward, LOW);
  digitalWrite(rightMotorsBackward, HIGH);
  digitalWrite(leftMotorsBackward, HIGH);
}

void turnRight() {
  Serial.write("turn right\n");

  analogWrite(leftMotorPWM, 200);
  analogWrite(rightMotorPWM, 200);
  digitalWrite(rightMotorsForward, LOW);
  digitalWrite(leftMotorsForward, HIGH);
  digitalWrite(rightMotorsBackward, HIGH);
  digitalWrite(leftMotorsBackward, LOW);
  delay(600);
  stopMotors();
  analogWrite(leftMotorPWM, motorSpeed);
  analogWrite(rightMotorPWM, motorSpeed);
}

void turnLeft() {
  Serial.write("turn left\n");

  analogWrite(leftMotorPWM, 200);
  analogWrite(rightMotorPWM, 200);
  digitalWrite(rightMotorsForward, HIGH);
  digitalWrite(leftMotorsForward, LOW);
  digitalWrite(rightMotorsBackward, LOW);
  digitalWrite(leftMotorsBackward, HIGH);
  delay(600);
  stopMotors();
  analogWrite(leftMotorPWM, motorSpeed);
  analogWrite(rightMotorPWM, motorSpeed);
}
