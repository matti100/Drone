#include "MPU6050.h"
#include <Wire.h>

// -------------- CONSTRUCTOR --------------
MPU6050::MPU6050(){
};

// -------------- INITIALIZATION --------------
void MPU6050::init(int accelSensitivityOption, int gyroSensitivityOption, int clock) {
  // I2C Initialization
  Serial.print("\nSet Clock frequency at ");
  Serial.print((float)clock);
  Serial.print(" Hz...");

  Wire.begin();
  Wire.setClock(clock);
  delay(250);

  Serial.print("COMPLETE");

  // MPU6050 Initialization
  Serial.print("\nIMU sensor reset...");

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(0x6B);                   // Register: Power Management Mode
  // Wire.write(0x80);                   // device reset (0x80)
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.print("COMPLETE");

  // Set Low Pass Filter
  Serial.print("\nActivate Low Pass Filter...");

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(CONFIG);                 // Register: Configuration
  Wire.write(0x05);                   // turn on LPF
  Wire.endTransmission();

  Serial.print("COMPLETE");

  // Manage Sensitivity Scale Factor
  Serial.print("\nSet Sensitiviy Scale Factor for Accelerometer and Gyroscope...");

  accelSens = accLSB_manager(accelSensitivityOption);
  gyroSens = gyroLSB_manager(gyroSensitivityOption);

  // Set Sensitivity Scale Factor for Accelerometer
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(ACCEL_CONFIG);               // Register: Accelerometer Configuration
  Wire.write(accelSens.sensScaleFactor);  // sensitivity scale factor
  Wire.endTransmission();

  // Set Sensitivity Scale Factor for Gyroscope
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(GYRO_CONFIG);                // Register: Gyroscope Configuration
  Wire.write(gyroSens.sensScaleFactor);   // sensitivity scale factor
  Wire.endTransmission();

  Serial.print("COMPLETE");

  delay(1000);
}

// -------------- SENSITIVITY MANAGER --------------
Sensitivity MPU6050::accLSB_manager(int accelSensitivityOption) {
  Sensitivity result;
  switch (accelSensitivityOption) {
    case 0:                     // +/- 2g
      result.LSB = 16384;
      result.sensScaleFactor = 0x00;
      break;
    case 1:                     // +/- 4g
      result.LSB = 8192;
      result.sensScaleFactor = 0x08;
      break;
    case 2:                     // +/- 8g
      result.LSB = 4096; 
      result.sensScaleFactor = 0x18;
      break;
    case 3:                     // +/- 16g
      result.LSB = 2048; 
      result.sensScaleFactor = 0x10;
      break;
    default:    
      result.LSB = 16384;
      result.sensScaleFactor = 0x00;
      break;
  }

  return result;
}

Sensitivity MPU6050::gyroLSB_manager(int gyroSensitivityOption) {
  Sensitivity result;
  switch (gyroSensitivityOption) {
    case 0:                     // +/- 250 deg/sec      
      result.LSB = 131;
      result.sensScaleFactor = 0x00;
      break;
    case 1:                     // +/- 500 deg/sec
      result.LSB = 65.5;
      result.sensScaleFactor = 0x08;
      break;
    case 2:                     // +/- 1000 deg/sec
      result.LSB = 32.8; 
      result.sensScaleFactor = 0x18;
      break;
    case 3:                     // +/- 2000 deg/sec
      result.LSB = 16.4; 
      result.sensScaleFactor = 0x10;
      break;
    default:    
      result.LSB = 131;
      result.sensScaleFactor = 0x00;
      break;
  }

  return result;
}

// -------------- ACCELEROMETER MEASUREMENTS --------------
void MPU6050::accel_meas() {

  // Accessing accelerometer readings register
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(ACCEL_X);
  Wire.endTransmission();

  // Requesting data
  Wire.requestFrom(I2C_ADDRESS, 6);

  int16_t SensAccX = Wire.read() << 8 | Wire.read();
  int16_t SensAccY = Wire.read() << 8 | Wire.read();
  int16_t SensAccZ = Wire.read() << 8 | Wire.read(); 

  // Scale data with sensitivity
  xAccel = (float)SensAccX / accelSens.LSB;
  yAccel = (float)SensAccY / accelSens.LSB;
  zAccel = (float)SensAccZ / accelSens.LSB;
}

// -------------- GYROSCOPE MEASUREMENTS --------------
void MPU6050::gyro_meas() {

  // Accessing gyroscope readings register
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(GYRO_X);
  Wire.endTransmission();

  // Requesting data
  Wire.requestFrom(I2C_ADDRESS, 6);

  int16_t SensGyroX = Wire.read() << 8 | Wire.read();
  int16_t SensGyroY = Wire.read() << 8 | Wire.read();
  int16_t SensGyroZ = Wire.read() << 8 | Wire.read(); 

  // Scale data with sensitivity
  rollRate = (float)SensGyroX / gyroSens.LSB;
  pitchRate = (float)SensGyroY / gyroSens.LSB;
  yawRate = (float)SensGyroZ / gyroSens.LSB;
}

// -------------- CALIBRATION --------------
void MPU6050::calibration() {

  Serial.print("Calibrating. Don't move the sensor");

  for (CalibrationIter = 0; CalibrationIter < 5000; CalibrationIter ++) {
    accel_meas();
    AccelCalibrationX += xAccel;
    AccelCalibrationY += yAccel;
    AccelCalibrationZ += zAccel;

    gyro_meas();
    RateCalibrationRoll += rollRate;
    RateCalibrationPitch += pitchRate;
    RateCalibrationYaw += yawRate;

    delay(1);
  }

  AccelCalibrationX /= 5000;
  AccelCalibrationY /= 5000;
  AccelCalibrationZ /= 5000;
  AccelCalibrationZ -= 1;
  RateCalibrationRoll /= 5000;
  RateCalibrationPitch /= 5000;
  RateCalibrationYaw /= 5000;
}

// -------------- POST PROCESSING SIGNALS --------------
void MPU6050::process_signals() {

}

// -------------- ACQUISITION --------------
void MPU6050::get_meas() {
  // ACCELEROMETER
  accel_meas();
  // Update with calibration data
  xAccel -= AccelCalibrationX;
  yAccel -= AccelCalibrationY;
  zAccel -= AccelCalibrationZ;

  // Conversion from [g] to [m/s^2]
  xAccel *= 9.81;
  yAccel *= 9.81;
  zAccel *= 9.81;

  // GYROSCOPE
  gyro_meas();
  // Update with calibration data
  rollRate -= RateCalibrationRoll;
  pitchRate -= RateCalibrationPitch;
  yawRate -= RateCalibrationYaw;
}

// -------------- COMPUTE LIFT-TORQUE COEFFICIENTS --------------
void MPU6050::computeLiftTorqueCoefficients(float vel) {

  // L = kF * w^2       ->      kF = T / w^2
  // C = kM * w^2       ->      kM = C / w^2

  // Convert angular velocity from [millisec] to [rpm]
  // BLDC motor used: 1000 KV
  // Battery used: 3S (11.1 V fully charge)
  // RPM = 1000 * 11.1 * DutyCycle
  float DC = (vel / 2000) * 100;          // DutyCycle = (Time ON / Period) * 100
  float RPM = 1000 * 11.1 * DC;
  
  // Compute Lift
  // Lift = (zAccel * mass) - Weight
  float mass = 1;             // mass of the experimental system [kg]
  float g = 9.81;
  lift = zAccel - g;    // specific lift (lift over mass) [N / kg]

  // Compute Torque

  // Compute coefficients
  float SCALE_FACTOR = 1e12;
  kF = SCALE_FACTOR * mass * lift / (4*(RPM * RPM)); 
}


// -------------- LOGGER --------------
void MPU6050::logger() {
  /*
  Serial.print("\nAcc: [" + String(xAccel) + ", " + String(yAccel) + ", " + String(zAccel) + "]  \t");
  Serial.print("AngRates: [" + String(rollRate) + ", " + String(pitchRate) + ", " + String(yawRate) + "]\n");
  */

  Serial.print("\nAcc: [");
  Serial.print(xAccel);
  Serial.print(",");
  Serial.print(yAccel);
  Serial.print(",");
  Serial.print(zAccel);
  Serial.print("] \t");
  Serial.print("AngRates: [");
  Serial.print(rollRate);
  Serial.print(",");
  Serial.print(pitchRate);
  Serial.print(",");
  Serial.print(yawRate);
  Serial.print("]\n");
  
}




