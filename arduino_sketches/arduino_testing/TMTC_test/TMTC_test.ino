#include "../../OBSW/TMTC.h"
#include "../../OBSW/TMTC.cpp"

TMTC esp32;

void setup() {
  Serial.begin(115200);
  const char* ssid = "NETGEAR59";
  const char* psw = "curlyskates816";
  const int serverPort = 3002;
  const char* url = "http://10.0.0.17:3001/data";

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
  esp32.handleClient();
  
  for (int i = 0; i <3; i++) {
    *(accel + i) = *(accel + i) + 1;
    *(gyro + i) = *(gyro + i) + 1;
    *(motorSpeed + i) = esp32.inputCMD;
  }

  delay(1000);

}
