#include <iterator>
#ifndef TMTC_h

#define TMTC_h

// ---------------- LIBRARIES --------------
#include <HTTPClient.h>
#include <WebServer.h>

// ---------------- CLASS DEFINITION --------------
class TMTC {
private:

  // CONSTANTS
  const char* URL;

  // VARIABLES

  // OBJECTS
  WebServer server;
  HTTPClient http;



public:

  // VARIABLES

  // FUNCTIONS
  TMTC();
  void wifi_init(char* ssid, char* psw, int serverPort, char* url);
  void bluetooth_init();

  void receiveData();
  void sendData(float* accel, float* gyro, float* motorSpeed);

};




#endif
