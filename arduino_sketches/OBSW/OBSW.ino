#include "MPU6050.h"
#include <Wire.h>

#include <Servo.h>

MPU6050 IMU;
Servo esc;

void setup() {
  Serial.begin(9600);

  int sensitivity = 1;        // +/- 4g     +/- 500 deg/s
  int clock = 400000;         // [Hz]

  Serial.print("\n\n----------------\n");
  Serial.print("IMU INITIALIZATION");
  Serial.print("\n----------------\n");
  IMU.init(sensitivity, sensitivity, clock);
  Serial.print("\nINITIALIZATION COMPLETE");

  Serial.print("\n\n----------------\n");
  Serial.print("IMU CALIBRATION");
  Serial.print("\n----------------\n");
  IMU.calibration();
  Serial.print("CALIBRATION COMPLETE");

  Serial.print("\n\n----------------\n");
  Serial.print("ACQUISITION");
  Serial.print("\n----------------\n");

  esc.attach(11);

}

String inputString = "";
bool stringComplete = false;
int vel = 1000;


void loop() {

  IMU.get_meas();

  IMU.logger();

/* 
  vel = analogRead(A0);
  vel = map(vel, 0, 1024, 1000, 2000);
  Serial.print(vel);
*/


  if (stringComplete) {
    // Convert the inpunt string in a number
    vel = inputString.toInt();

    // Print the input number
    Serial.print("Numero ricevuto: ");
    Serial.println(vel);

    // Input string and flag reset
    inputString = "";
    stringComplete = false;
  }

  esc.writeMicroseconds(vel);
}


void serialEvent() {
  while (Serial.available()) {

    // Read incoming character
    char inChar = (char)Serial.read();
    // Update the input string
    inputString += inChar;

    // If end line character ('\n') is read, the input is complete
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


