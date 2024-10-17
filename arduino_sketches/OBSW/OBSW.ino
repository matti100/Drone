#include "MPU6050.h"
#include "BLDC.h"

MPU6050 IMU;
BLDC Motors;

void setup() {
  Serial.begin(9600);

  int sensitivity = 1;        // +/- 4g     +/- 500 deg/s
  int clock = 400000;         // [Hz]
  int initialSignal = 1000;   // PWM min

  Serial.print("\n\n----------------\n");
  Serial.print("IMU INITIALIZATION");
  Serial.print("\n----------------\n");
  IMU.init(sensitivity, sensitivity, clock);
  Motors.init(initialSignal);
  Serial.print("\nINITIALIZATION COMPLETE");

  Serial.print("\n\n----------------\n");
  Serial.print("IMU CALIBRATION");
  Serial.print("\n----------------\n");
  IMU.calibration();
  Serial.print("IMU CALIBRATION COMPLETE");

  Serial.print("\n\n----------------\n");
  Serial.print("DATA ACQUISITION");
  Serial.print("\n----------------\n");

}

String inputString = "";
bool stringComplete = false;
int input = 0;
int vel = 1000;


void loop() {

  IMU.get_meas();

  if (stringComplete) {
    // Convert the inpunt string in a number
    input = inputString.toInt();

    // Input string and flag reset
    inputString = "";
    stringComplete = false;

    // vel = ceil((255.0/100.0) * vel);
    vel = 1000 + 10*input;
  }

  Motors.setVelocity(vel, vel, vel, vel);

  IMU.computeLiftTorqueCoefficients(vel);
  Serial.print("zAccel:");
  Serial.print(IMU.zAccel);
  Serial.print("\tLift:");
  Serial.print(IMU.lift);
  Serial.print("\tInput command:");
  Serial.print(input);
  Serial.print(" \tLift Coefficients:");
  Serial.println(IMU.kF);
  delay(100);
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

/*

void logger(MPU6050 IMU, BLDC motor1, BLDC motor2, BLDC motor3, BLDC motor4) {

}
*/


