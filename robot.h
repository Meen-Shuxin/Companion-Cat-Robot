#ifndef ROBOT_H_
#define ROBOT_H_
//#include <ESPAsyncWebServer.h>
#include "FS.h"
#include "SPIFFS.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include<WebServer.h>



class Robot {
public:
  Robot();
  ~Robot(){};
  bool initSD();
  bool initWifi(const char *ssid, const char *password);
//  void setStream(const char *name,OV2640 * camera);
  void setIndex(const char *html);
  void setIndex(const char *path,const char *name);
//  typedef std::function<void(void)> THandlerFunction;
  void setControl(const char *name, WebServer::THandlerFunction fn);
  void setVoice(const char *name);
  void begin();
  bool IsVoice = false;
  WebServer _server;
private:
//  AsyncWebServer *_server;
};
#endif
