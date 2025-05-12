
#include <ESP32Servo.h>
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

void setup() {

  Serial.begin(9600);

  esc1.attach(25, 1000, 2000);
  esc2.attach(26, 1000, 2000);
  esc3.attach(27, 1000, 2000);
  esc4.attach(32, 1000, 2000);


  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  esc4.writeMicroseconds(2000);

  delay(2000);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
}

String incomingString = "";
int speed = 1000;

void loop() {

  if (Serial.available()) {
    incomingString = Serial.readStringUntil('\n');
    speed = (int)atof(incomingString.c_str());
    Serial.println("Speed Received");
  }

  esc1.writeMicroseconds(speed);
  esc2.writeMicroseconds(speed);
  esc3.writeMicroseconds(speed);
  esc4.writeMicroseconds(speed);
}
