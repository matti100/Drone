
#define MOTOR_PIN 9

int value;

void setup()
{
  pinMode(MOTOR_PIN, OUTPUT);
  Serial.begin(9600);

}

void loop()
{
  value = analogRead(A0);
  value = map(value, 0, 1023, 0, 255);
  
  Serial.println(value);
  
  analogWrite(MOTOR_PIN, value);
}