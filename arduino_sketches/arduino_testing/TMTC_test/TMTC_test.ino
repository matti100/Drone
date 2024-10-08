#include "../../OBSW/TMTC.h"
#include "../../OBSW/TMTC.cpp"

TMTC esp32;

void setup() {
  Serial.begin(115200);
  const char* ssid = "FASTWEB-AUACL6";
  const char* psw = "KXFX32ELXE";
  const int serverPort = 80;
  const char* url = "http://192.168.1.53:3000/data";

  esp32.wifi_init(ssid, psw, serverPort, url);
}

float acc[] = {0, 0, 0};
float gy[] = {0, 0, 0};
float sp[] = {0, 0, 0};
float* accel = &acc[0];
float* gyro = &gy[0];
float* motorSpeed = &sp[0];

void loop() {

  esp32.sendData(accel, gyro, motorSpeed);

  for (int i = 0; i <3; i++) {
    *(accel + i) = *(accel + i) + 1;
    *(gyro + i) = *(gyro + i) + 1;
    *(motorSpeed + i) = *(motorSpeed + i) + 1;
  }

}
