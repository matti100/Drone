#define trigPin 16
#define echoPin 17

void setup() {

  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, LOW);

}

float duration = 0;
float distance = 0;

void loop() {
  digitalWrite(trigPin, HIGH);

  delayMicroseconds(20);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration * 0.0343 / 2;
  Serial.print("Distance [cm]:\t");
  Serial.println(distance);
}
