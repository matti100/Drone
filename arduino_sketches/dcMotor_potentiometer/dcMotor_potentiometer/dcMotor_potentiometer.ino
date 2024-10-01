// Pin 11 -> ESC control pin
// Pin A0 -> Analog pin that reads potentiometer voltage.
#include <Servo.h>
Servo esc;

void setup() {
  esc.attach(11);
  Serial.begin(9600);
}

int v = 0;

void loop() {
  v = analogRead(A0);
  v = map(v, 0, 1024, 1000, 2000);
  Serial.println(v);
  esc.writeMicroseconds(v);
}

