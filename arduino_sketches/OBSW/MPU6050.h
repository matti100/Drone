// MPU6050.h

#ifndef MPU6050_h

#define MPU6050_h


// ---------------- LIBRARIES --------------
#include <Arduino.h>
#include <Wire.h>

// ---------------- DATA TYPES --------------
struct Sensitivity {
  float LSB;
  int sensScaleFactor;
};

// ---------------- CLASS DEFINITION --------------
class MPU6050 {
private:
  // CONSTANTS
  // Registers Address
  #define I2C_ADDRESS 0x68
  #define CONFIG 0x1A
  #define ACCEL_CONFIG 0x1C
  #define GYRO_CONFIG 0x1B
  #define ACCEL_X 0x3B
  #define GYRO_X 0x43

  // VARIABLES
  float AccelCalibrationX = 0;
  float AccelCalibrationY = 0;
  float AccelCalibrationZ = 0;
  float RateCalibrationRoll = 0;
  float RateCalibrationPitch = 0;
  float RateCalibrationYaw = 0;

  int CalibrationIter;

  Sensitivity accelSens;
  Sensitivity gyroSens;

public:
  // VARIABLES
  float xAccel;
  float yAccel;
  float zAccel;
  float rollRate;
  float pitchRate;
  float yawRate;

  float kF;
  float kM;

  // FUNCTIONS
  MPU6050();
  void init(int accelSensitivityOption, int gyroSensitivityOption, int clock);
  void calibration();
  void accel_meas();
  void gyro_meas();
  void process_signals();
  void get_meas();

  void computeLiftTorqueCoefficients(float vel);

  Sensitivity accLSB_manager(int accelSensitivityOption);
  Sensitivity gyroLSB_manager(int gyroSensitivityOption);
  void logger();
};

#endif