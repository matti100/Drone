#include "BLDC.h"
#include "Arduino.h"
#include "Servo.h"

// -------------- CONSTRUCTOR --------------
BLDC::BLDC(){
};

// -------------- INITIALIZATION --------------
void BLDC::init(int initialSignal) {
  esc1.attach(MOTOR_PIN_1, minPWM, maxPWM);
  esc2.attach(MOTOR_PIN_2, minPWM, maxPWM);
  esc3.attach(MOTOR_PIN_3, minPWM, maxPWM);
  esc4.attach(MOTOR_PIN_4, minPWM, maxPWM);

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
  //    DutyCycle [%] = (minPWM / maxPWM) * pwmSignal;      minPWM = 1000 microseconds, maxPWM = 2000 microseconds
  
  float voltage = 11.1;        // !! to do: batteryLevel, batteryVoltage !! 

  w1 = MOTOR_POWER * maxVoltage * (signal1 / maxPWM) * minPWM;
  w2 = MOTOR_POWER * maxVoltage * (signal2 / maxPWM) * minPWM;
  w3 = MOTOR_POWER * maxVoltage * (signal3 / maxPWM) * minPWM;
  w4 = MOTOR_POWER * maxVoltage * (signal4 / maxPWM) * minPWM;

};


// -------------- SET MOTOR VELOCITY --------------
void BLDC::setVelocity(int pwmSignal1, int pwmSignal2, int pwmSignal3, int pwmSignal4) {

  updateCMD(pwmSignal1, pwmSignal2, pwmSignal3, pwmSignal4);

  esc1.writeMicroseconds(signal1);
  esc2.writeMicroseconds(signal2);
  esc3.writeMicroseconds(signal3);
  esc4.writeMicroseconds(signal4);
};
