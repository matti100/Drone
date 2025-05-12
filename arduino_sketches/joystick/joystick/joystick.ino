#define JOYSTICK_X 0
#define JOYSTICK_Y 4
#define JOYSTICK_BUTTON 2

int xCalibration = 0;
int yCalibration = 0;

float x = 0;
float x_prev;
float y = 0;
float y_prev;
int button = 0;

const float T = 0.3;
const float Ts = 0.01;
const float K = 0.8;

float t_start = 0;
float t_end = 0;
float t_exe = 0;
float t_sample_us = Ts * 1e6;

void setup() {

  Serial.begin(115200);
  pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);

  for (int i = 0; i<10000; i++) {

    x = analogRead(JOYSTICK_X);
    y = analogRead(JOYSTICK_Y);

    xCalibration += x;
    yCalibration += y;
  };

  xCalibration /= 10000;
  yCalibration /= 10000;

}


void loop() {

  t_start = micros();

  x_prev = x;
  y_prev = y;

  x = analogRead(JOYSTICK_X);
  y = analogRead(JOYSTICK_Y);
  button = !digitalRead(JOYSTICK_BUTTON);

  // x = DLPF(x_prev, x);
  // y = DLPF(y_prev, y);
  x -= xCalibration;
  y -= yCalibration;

  x = map(x, 0, 4095, -100, 100) + 100;

  y = map(y, 0, 4095, -100, 100) + 100;

  x = DLPF(x_prev, x);

  Serial.print(x);
  Serial.print("\t\t");
  Serial.print(y);
  Serial.print("\t\t");
  Serial.println(button);

  t_end = micros();
  t_exe = t_end - t_start;
  if (t_exe < t_sample_us) {
    delayMicroseconds(t_sample_us - t_exe);
  };

}

float DLPF(float x_prev, float u) {
  y = (1 - Ts/T)*x_prev + K*(Ts/T)*u;
  return y;
};
