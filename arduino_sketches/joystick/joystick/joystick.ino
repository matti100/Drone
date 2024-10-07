#define JOYSTICK_X A0
#define JOYSTICK_Y A1
#define JOYSTICK_BUTTON 2

void setup() {
  Serial.begin(9600);
  pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);

}

int x;
int y;
int button;


void loop() {

  x = analogRead(JOYSTICK_X);
  y = analogRead(JOYSTICK_Y);

  button = !digitalRead(JOYSTICK_BUTTON);

  Serial.print("X: " + String(x));
  Serial.print(",\tY: " + String(y));
  Serial.println(",\tButton: " + String(button));
  delay(100);

}
