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

  // Initialize WiFi connection
  WiFi.begin(ssid, psw);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connection...");
  }

  Serial.print("Connected to Wi-Fi:\t");
  Serial.println(ssid);
  Serial.print("ESP32 IP:\t");
  Serial.println(WiFi.localIP());

  // Initialize Server (for GET requests)
  server.on("/data", HTTP_POST, [this]() {
    this -> receiveData();
  });
  server.begin(serverPort);

  // Initialize HTTP connection (for POST requests)
  URL = url;
  http.begin(URL);
}

// -------------- BLUETOOTH INITIALIZATION --------------
void TMTC::bluetooth_init() {
}

// -------------- WIFI --------------
void TMTC::sendData(float* accel, float* gyro, float* motorSpeed) {

  if (WiFi.status() == WL_CONNECTED) {

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

  } else {
    Serial.println("ERROR: WiFi connection failed");
  }

}

/* 
void TMTC::receiveData() {
  // Verify if request has "plain text" data type (JSON string)
  if (server.hasArg("plain")) { 
    // Extract JSON
    String jsonData = server.arg("plain");
    Serial.println("Received data: " + jsonData);

    // Extract data from JSON
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonData);

    if (!error) {
      inputCMD = doc["speed"];

      // Answer to client 
      server.send(200, "application/json", "{\"status\":\"ok\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\", \"message\":\"Invalid JSON\"}");
    }
  }
}
*/ 

void TMTC::receiveData() {
  // Extract JSON string
  String jsonData = server.arg("plain"); // text plain data type (JSON string)

  // Extract JSON data
  StaticJsonDocument<200> doc;
  deserializeJson(doc, jsonData);

  inputCMD = doc["speed"];

  Serial.println(inputCMD);

  // Send response to client
      server.send(200, "application/json", "{\"message\":\"ESP32 received data successfully\"}");
}


void TMTC::handleClient() {
  server.handleClient();
}

// -------------- BLUETOOTH --------------

