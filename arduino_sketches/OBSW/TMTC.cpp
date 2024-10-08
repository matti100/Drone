#include "Arduino.h"
#include "TMTC.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// -------------- CONSTRUCTOR --------------
TMTC::TMTC(){
}

// -------------- INITIALIZATION --------------
// -------------- WIFI Initialization --------------
void TMTC::wifi_init(const char* ssid, const char* psw, const int serverPort, const char* url) {

  // Initialize Wifi connection
  WiFi.begin(ssid, psw);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connection...");
  }

  Serial.print("Connected to Wi-Fi:\t");
  Serial.println(ssid);
  Serial.print("ESP32 IP:\t");
  Serial.println(WiFi.localIP());

  // Initialize Server
  server.begin(serverPort);

  // Begin connection
  URL = url;
  http.begin(URL);

}

// -------------- BLUETOOTH INITIALIZATION --------------
void TMTC::bluetooth_init() {

}

// -------------- WIFI --------------
void TMTC::sendData(float* accel, float* gyro, float* motorSpeed) {

  if (WiFi.status() == WL_CONNECTED) {
    
    // Begin connection
    // http.begin(URL);

    // Create JSON object
    StaticJsonDocument<200> doc;

    JsonArray accelArray = doc.createNestedArray("Accel");
    accelArray.add(accel[0]);
    accelArray.add(accel[1]);
    accelArray.add(accel[2]);

    JsonArray gyroArray = doc.createNestedArray("Gyro");
    gyroArray.add(gyro[0]);
    gyroArray.add(gyro[1]);
    gyroArray.add(gyro[2]);
    
    JsonArray motorArray = doc.createNestedArray("motorSpeed");
    motorArray.add(motorSpeed[0]);
    motorArray.add(motorSpeed[1]);
    motorArray.add(motorSpeed[2]);

    // Convert into String JSON
    String jsonData;
    serializeJson(doc, jsonData);

    // Send JSON data as a POST request
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(jsonData); 

    // Receive response from server
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Server response: " + response);
    } else {
      Serial.println("Error during POST request");
    }

    // End connection
    // http.end();

  } else {
    Serial.println("Not connected");
  }

}

void TMTC::receiveData() {

  server.handleClient();
  
  if (server.hasArg("plain")) {  // Verifica se ci sono dati nella richiesta
    String jsonData = server.arg("plain");  // Estrai i dati JSON
    Serial.println("Dati ricevuti: " + jsonData);

    // Extract data from JSON
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonData);

    if (!error) {
      int inputCMD = doc["motore"];

      Serial.println("InputCMD: \t");
      Serial.println(inputCMD);

      // Answer to client 
      server.send(200, "application/json", "{\"status\":\"ok\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\", \"message\":\"Invalid JSON\"}");
    }
  }

}

// -------------- BLUETOOTH --------------

