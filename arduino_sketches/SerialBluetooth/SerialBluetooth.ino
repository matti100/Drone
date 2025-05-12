#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
// MASTER
void setup() {

  SerialBT.begin("ESP32_MASTER");
  SerialBT.connect("ESP32_SLAVE");

  while (!SerialBT.connected()) {
    SerialBT.connect("ESP32_SLAVE");
  }
}

void loop() {
  if (SerialBT.connected()) {
    SerialBT.println("Ciao dal master");
    delay(1000);
  }
}
