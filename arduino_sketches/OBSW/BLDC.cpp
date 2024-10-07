#include "BLDC.h"
#include "Arduino.h"

// -------------- CONSTRUCTOR --------------
BLDC::BLDC(){
};

// -------------- INITIALIZATION --------------
void BLDC::init(int initialSignal) {

  setVelocity(initialSignal, initialSignal, initialSignal, initialSignal);

};

// -------------- UPDATE COMMANDS --------------
void BLDC::updateCMD(int pwmSignal1, int pwmSignal2, int pwmSignal3, int pwmSignal4) {

  // PWM signals
  signal1 = pwmSignal1;
  signal2 = pwmSignal2;
  signal3 = pwmSignal3;
  signal4 = pwmSignal4;

  // Conversion from [pwm] to [rpm]
  //    -> RPM = motorPower [KV] * batteryVoltage [V] * batteryLevel [0, 1] * DutyCycle [%]
  //    DutyCycle [%] = (100 / 255) * pwmSignal;
  
  float voltage = 11.1;        // !! to do: batteryLevel, batteryVoltage !! 

  w1 = MOTOR_POWER * voltage * (signal1 / 255) * 100;
  w2 = MOTOR_POWER * voltage * (signal2 / 255) * 100;
  w3 = MOTOR_POWER * voltage * (signal3 / 255) * 100;
  w4 = MOTOR_POWER * voltage * (signal4 / 255) * 100;

};


// -------------- SET MOTOR VELOCITY --------------
void BLDC::setVelocity(int pwmSignal1, int pwmSignal2, int pwmSignal3, int pwmSignal4) {

  updateCMD(pwmSignal1, pwmSignal2, pwmSignal3, pwmSignal4);

  analogWrite(MOTOR1_PIN, signal1);
  analogWrite(MOTOR2_PIN, signal2);
  analogWrite(MOTOR3_PIN, signal3);
  analogWrite(MOTOR4_PIN, signal4);

};
