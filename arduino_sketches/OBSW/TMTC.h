
#ifndef TMTC_h

#define TMTC_h

// ---------------- LIBRARIES --------------
#include <WebServer.h>
#include <HTTPClient.h>

// ---------------- CLASS DEFINITION --------------
class TMTC {
private:

  // CONSTANTS
  const char* URL;

  // VARIABLES

  // OBJECTS
  HTTPClient http;

public:

  // VARIABLES
  int inputCMD;

  // OBJECTS
  WebServer server;


  // FUNCTIONS
  TMTC();
  void wifi_init(const char* ssid, const char* psw, const int serverPort, const char* url);
  void bluetooth_init();

  void receiveData();
  void sendData(float* accel, float* gyro, float* motorSpeed);
  void handleClient();
};




#endif
