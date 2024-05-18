#include "robot.h"

#define TAG "Robot"

Robot::Robot() {
  //  this->_server = WebServer();

  //  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
}
//void Robot::setStream(const char *name, OV2640 *camera) {
//  this->_server->on(name, HTTP_GET, [camera](AsyncWebServerRequest * request) {
//    camera->run();
//    request->send_P(200, "image/jpeg", (const uint8_t *)camera->getfb(), camera->getSize());
//  });
//}
void Robot::setControl(const char *name, WebServer::THandlerFunction fn) {

  this->_server.on(
    name, HTTP_GET,
    fn);
}


void Robot::setVoice(const char *name) {
  //  this->_server->on(name, HTTP_POST,
  //  [](AsyncWebServerRequest * request) {
  //    request->send(200);
  //  },
  //  [this](AsyncWebServerRequest * request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  //    if (!index) {
  //      request->_tempFile = SPIFFS.open("/" + filename, "w");
  //    }
  //
  //    if (len) {
  //      request->_tempFile.write(data, len);
  //    }
  //
  //    if (final) {
  //      request->_tempFile.close();
  ////      String msgControl = "{\"cmd\":\"voice\",\"data\":\"ON\"}";
  //      Serial.println("Write File success "+filename);
  //      this->IsVoice = true;
  ////      Serial.flush();
  //    }
  //
  //  });
}

void Robot::setIndex(const char *html) {
  //  this->_server->on("/", HTTP_GET, [html](AsyncWebServerRequest * request) {
  //    request->send_P(200, "text/html", html);
  //  });
}
void Robot::setIndex(const char *path, const char *name) {
  //  this->_server->on("/", HTTP_GET, [name](AsyncWebServerRequest * request) {
  //    request->send(SPIFFS, name, "text/html");
  //  });
  //  this->_server->serveStatic("/", SPIFFS, "/");
  this->_server.on(path, [this]() {
    this->_server.send(200, "text/html", "<html>123456789</html>");
  });
//  this->_server.on("/test", HTTP_GET, [this]() {
//
//    // send(200, "text/html", "<html>123456789</html>");
//
//    String message = "File Not Found\n\n";
//    message += "URI: ";
//    message += this->_server.uri();
//    message += "\nMethod: ";
//    message += (this->_server.method() == HTTP_GET) ? "GET" : "POST";
//    message += "\nArguments: ";
//    message += this->_server.args();
//    message += "\n";
//    for (uint8_t i = 0; i < this->_server.args(); i++) {
//      message += " " +  this->_server.argName(i) + ": " + this->_server.arg(i) + "\n";
//    }
//    this->_server.send(200, "text/plain", message);
//
//  });



}

void Robot::begin() {
  this->_server.enableCORS();
  this->_server.begin();

};
bool Robot::initWifi(const char *ssid, const char *password) {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("[WiFi] WiFi is connected!");
  Serial.print("[WiFi] IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}
bool Robot::initSD() {
  if (!SPIFFS.begin(false)) {
    Serial.println("Card Mount Failed");
    return false;
  }
  return true;
}
