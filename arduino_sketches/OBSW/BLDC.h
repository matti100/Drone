// BLDC.h

#ifndef BLDC_h

#define BLDC_h

// ---------------- LIBRARIES --------------
#include "Servo.h" // same syntax both for Arduino and ESP32 boards

// ---------------- CLASS DEFINITION --------------
class BLDC {
private:

  // CONSTANTS
  #define MOTOR_PIN_1 11
  #define MOTOR_PIN_2 10
  #define MOTOR_PIN_3 9
  #define MOTOR_PIN_4 6

  #define MOTOR_POWER 11.1  // [KV]

  #define maxPWM 2000   // [microseconds]
  #define minPWM 1000   // [microseconds]

  #define maxVoltage 11.1   // [V]

  // VARIABLES
  int signal1;        // PWM signal [1000-2000 microseconds]
  int signal2;
  int signal3;
  int signal4;

  // OBJECTS
  Servo esc1;
  Servo esc2;
  Servo esc3;
  Servo esc4;

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