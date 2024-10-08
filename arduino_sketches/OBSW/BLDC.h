// BLDC.h

#ifndef BLDC_h

#define BLDC_h

// ---------------- LIBRARIES --------------

// ---------------- CLASS DEFINITION --------------
class BLDC {
private:

  // CONSTANTS
  #define MOTOR1_PIN 11
  #define MOTOR2_PIN 11
  #define MOTOR3_PIN 11
  #define MOTOR4_PIN 11

  #define MOTOR_POWER 11.1  // [KV]

  // VARIABLES
  int signal1;        // PWM signal [0-255]
  int signal2;
  int signal3;
  int signal4;

public:

  // VARIABLES
  float w1;           // [rpm]
  float w2;
  float w3;
  float w4;

  // FUNCTIONS
  BLDC();
  void init(int initialSignal);
  void updateCMD(int pwmSignal1, int pwmSignal2, int pwmSignal3, int pwmSignal4);
  void setVelocity(int pwmSignal1, int pwmSignal2, int pwmSignal3, int pwmSignal4);

};

#endif