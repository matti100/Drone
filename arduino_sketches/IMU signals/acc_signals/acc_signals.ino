////////////////////////////////////////
/////// ACCELERATIONS //////////////////
////////////////////////////////////////
/*
*   This script is meant to be a quick reference for the implementation of
*   a more sofisticated accelerometer measurement algorithm for the acqisition
*   of linear accelerations
* 
*   DEVICES:
*   - Arduino Uno R3
*   - MPU6050 (IMU sensor)
* 
*   LIBRARIES:
*   - Wire.h
*
*   BRIEF EXPLANATION of the code:
*   1) We need to access the internal registers of the IMU sensor in order to:
*   - set important parameters and settings
*   - reading data 
*
*   This type of devices are divided into "internal registers" that are responsible for different actions, such
*   as storing data, settings ecc.
*   Each internal register has an unique "address" (generally written in hexadecimal format)
*   and each register contains a total of 8 bits (from bit0 to bit7) that can store informations
*   It is possible to write and read data from the register in an hexadecimal format (instead of sending data in binary,
*   which can be very impractical).
*   
*  
*   REFERENCES:
*   - Register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
*   - Datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
*

/* ----------------------- Libraries ----------------------- */

// Library for I2C communication 
#include <Wire.h>

/* ---------------------- Global Variables ------------------- */

#define MPU6050_ADDRESS 0x68                // I2C address associated with MPU6050
#define CONFIG 0x1A                         // Register address to access Low Pass Filter Register in MPU6050
                                            //                      (here we can set the LowPassFilter)      
#define ACCEL_CONFIG 0x1C                   // Register address to access Accelerations Configurations settings in MPU6050
                                            //                      (here we can set the Sensitivity of the accelerations)
#define ACC_X_MEAS 0x3B                     // Register address to access Accelerometer Measurements in MPU6050
                                            //                      (more precisely, X measurements)

float xAcc;
float yAcc;
float zAcc;

float xAccCalibration = 0;
float yAccCalibration = 0;
float zAccCalibration = 0;
int RateCalibrationNumber;

/* ------------------------- Functions --------------------------- */

// SETUP AND RECEIVE SIGNALS FROM ACCELEROMETER
void acc_signals() {

  // Switch on the LOW PASS FILTER
  Wire.beginTransmission(MPU6050_ADDRESS);  // starting communication with MPU6050
  Wire.write(CONFIG);                       // accessing the internal register responsible for the Low Pass Filter
  Wire.write(0x05);                         // sending data in hexadecimal format to turn on the LPF
  Wire.endTransmission();

  // Set the sensitivity scale factor
  Wire.beginTransmission(MPU6050_ADDRESS);    // starting communication with MPU6050
  Wire.write(ACCEL_CONFIG);                   // accessing accelerometer configuration register
  Wire.write(0x08);                           // sending data in hexadecimal format to set the desidered sensitivity
                                              //                             ( +- 4g)
  Wire.endTransmission();
  
  // ACC MEASUREMENTS
  Wire.beginTransmission(MPU6050_ADDRESS);    // starting communication with MPU6050
  Wire.write(ACC_X_MEAS);                     // accessing the internal register for accelerometer measurements
  Wire.endTransmission();                     // now we are set to the accelerometer internal register

  // Request Data
  Wire.requestFrom(MPU6050_ADDRESS, 6);       // requesting 6 BYTES from MPU6050

  int16_t SensAccX = Wire.read() << 8 | Wire.read();   // the data of the X acceleration are formatted in 16 BITS (2 bytes).
                                                       // The << 8 operator move the 8 bits received from the first
                                                       // Wire.read() to the left of 8 positions.
                                                       // The | operator lets the following 8 bits to join the previous 8 bits received
                                                       // Wire.read() let us receive the last 8 bits.
                                                       // AccX now is a variable composed of 16 bits (2 bytes)
  int16_t SensAccY = Wire.read() << 8 | Wire.read();
  int16_t SensAccZ = Wire.read() << 8 | Wire.read();
  // All the data about the X, Y and Z acceleration has been received.

  // Convert data from sensor readings to g (see LSB sensitivity in Register Map for context)
  xAcc = (float)SensAccX / 8192;
  yAcc = (float)SensAccY / 8192;
  zAcc = (float)SensAccZ / 8192;
}

// SETUP
void setup() {
  // Start Serial Communication with computer
  Serial.begin(57600);

  // Set parameters for the I2C communication with MPU6050
  Wire.setClock(400000);                    // set the clock frequency (how many times in a second master and slave communicate)
                                            
  Wire.begin();                           
  delay(250);

  Wire.beginTransmission(MPU6050_ADDRESS);  // starting communication with MPU6050
  Wire.write(0x6B);                         // accessing the internal register responsible for Power Management Mode
  Wire.write(0x00);                         // Device Reset
  Wire.endTransmission();

  // Starting Calibration Procedure
  Serial.print("\n\nCalibrating. Don't move the sensor");

  // Measuring data for calibration
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 5000; RateCalibrationNumber ++) {
    acc_signals();                         // measuring

    xAccCalibration += xAcc;              // adding 5000 accelerations measurements
    yAccCalibration += yAcc;
    zAccCalibration += zAcc;
    delay(1);
  }

  xAccCalibration /= 5000;             // computing the avarage of the first 5000 measurements 
  yAccCalibration /= 5000;             // They represent the zero mean noise 
  zAccCalibration /= 5000;             // associated to the sensor
  zAccCalibration -= 1;                // add gravitational contribution (removed with calibration)
}


// LOOP
void loop() {
  acc_signals();                           // measuring accelerations

  xAcc -= xAccCalibration;                // update xAcc with calibration data
  yAcc -= yAccCalibration;                // update yAcc with calibration data
  zAcc -= zAccCalibration;                // update zAcc with calibration data

  // Convert acc from g to m/s^2
  xAcc *= 9.81;
  yAcc *= 9.81;
  zAcc *= 9.81;

  Serial.print("\n\n-------------------------\n");
  Serial.print("X Acc [m/s^2]:\t");
  Serial.print(xAcc);
  Serial.print("\t\tY Acc [m/s^2]:\t");
  Serial.print(yAcc);
  Serial.print("\tZ Acc [m/s^2]:\t");
  Serial.print(zAcc);

  delay(50);
}
