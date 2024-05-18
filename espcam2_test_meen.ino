//#include <ArduinoJson.h>
//#include <ESPAsyncWebServer.h>
//#include "FS.h"
//#include "SD_MMC.h"
#include <WiFi.h>
//#include "screen.h";
#include <ESP32Servo.h>
#include "Kinematics.h"
#include <SPI.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include "robot.h"
#include <PCF8574.h>
//PCF8574 PCF(0x20);
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//#include "Audio.h"
//#include "BluetoothA2DPSink.h"
//BluetoothA2DPSink a2dp_sink;
//#define FORMAT_SPIFFS_IF_FAILED true
#include<btAudio.h>
#define I2S_DOUT  27
#define I2S_BCLK  26
#define I2S_LRC   25
btAudio audio = btAudio("ESP_Speaker");
Robot robot;



#define MAX_IMAGE_WDITH 240
int16_t xpos = 0;
int16_t ypos = 0;
#include "SPI.h"
//#include <TFT_eSPI.h>              // Hardware-specific library
//TFT_eSPI tft = TFT_eSPI();         // Invoke custom library
//TFT_eSprite background = TFT_eSprite(&tft);

#include <TFT_eSPI.h>  // Include the TFT_eSPI library
TFT_eSPI tft = TFT_eSPI();  // Create an instance of the TFT_eSPI library

#define MOTOR_MAX_RPM 255     // motor's maximum rpm
#define WHEEL_DIAMETER 0.06     // robot's wheel diameter expressed in meters
#define FR_WHEEL_DISTANCE 0.18   // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.19   // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)
Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS);
Kinematics::output rpm;


const int MAX_LINEAR_VEL = 100; //เกี่ยวกับล้อ
const int MAX_ANGULAR_VEL = 100;

#define motor1_a  6
#define motor1_b  7
#define motor1_e  8

#define motor2_a  9
#define motor2_b  10
#define motor2_e  11

#define motor3_a  0
#define motor3_b  1
#define motor3_e  2

#define motor4_a  3
#define motor4_b  4
#define motor4_e  5

#define laser  12
//#define S_ervo  23
TaskHandle_t Task1;

const int S_ervo = 23;
bool toyEn = false;
int toySpeed = 1;

int angle = 10;

//#define SCREEN_WIDTH 128 // OLED display width, in pixels
//#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//
//#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
//#define SCREEN_ADDRESS 0x3C
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

String msgControl;
Servo meen;

DynamicJsonDocument doc(1023);
DeserializationError error;

float linear_vel_x = 0;  // 1 m/s
float linear_vel_y = 0;  // 0 m/s
float angular_vel_z = 0; // 1 m/s

unsigned long previousTime = 0;

const char *ssid = "test111";
const char *password = "12345678";
//Audio audio;
//bool isVoice = false;
//--------------------------------Fuction-------------------------
void run_motor(int a, int b, int e, int speed) {
  if (speed > 0) {
    //    PCF.digitalWrite(a, 0);
    //    PCF.digitalWrite(b, 1);
    pwm.setPWM(a, 0, 4096);
    pwm.setPWM(b, 4096, 0);
  }
  else if (speed < 0) {
    //    PCF.digitalWrite(a, 1);
    //    PCF.digitalWrite(b, 0);
    pwm.setPWM(a, 4096, 0);
    pwm.setPWM(b, 0, 4096);
  }
  else {
    //    PCF.digitalWrite(a, 0);
    //    PCF.digitalWrite(b, 0);
    pwm.setPWM(a, 0, 4096);
    pwm.setPWM(b, 0, 4096);
  }
  Serial.println(speed);
  //  analogWrite(e, abs(speed));
  pwm.setPWM(e, abs(speed), 0);
}

void Task1code( void * pvParameters ) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    if (toyEn) {

      for (int i = 0; i <= 180; i++) {
        meen.write(i);  // Set the servo position
        delay(toySpeed);

      }
      for (int i = 180; i >= 0; i--) {
        meen.write(i);  // Set the servo position
        delay(toySpeed);

      }
    }
    delay(50);
  }
}



//void w_ait (unsigned long interval) {
//  int w = 0;
//  while (w == 0) {
//    unsigned long currentTime = millis();
//    if (currentTime - previousTime >= interval) {
//      w = 1;
//      previousTime = currentTime;
//    }
//  }
//}



//--------------------------------setup-------------------------
void setup() {

  Serial.begin(115200);
  pinMode(laser, OUTPUT);

  pwm.begin();
  pwm.setPWMFreq(1000);
  Wire.setClock(400000);

  meen.attach(S_ervo); //เปลี่ยนขาพิน
  meen.write(0);
  //  pinMode(S_ervo, OUTPUT);

  xTaskCreatePinnedToCore(
    Task1code, /* Function to implement the task */
    "Task1", /* Name of the task */
    5000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    1,  /* Priority of the task */
    &Task1,  /* Task handle. */
    0); /* Core where the task should run */

  delay(500);

  tft.init();  // Initialize the display
  tft.setRotation(1);  // Set display rotation (0, 1, 2, or 3) to match your setup
  tft.fillScreen(TFT_BLACK);  // Fill the screen with black color

  tft.setTextColor(TFT_WHITE, TFT_BLACK); // Set text color to white on black background
  tft.setTextSize(3); // Set text size to a larger value for better visibility

  Serial.println("\n\n Using the PNGdec library");
  // Initialise the TFT
  //  tft.begin();
  //  tft.setRotation(1);
  //  tft.fillScreen(TFT_WHITE);
  //  tft.setSwapBytes(true);
  //  background.createSprite(517, 388);
  //  background.setSwapBytes(true);
  Serial.println("\r\nInitialisation done.");
  audio.begin();
  audio.reconnect();
  audio.I2S(I2S_BCLK, I2S_DOUT, I2S_LRC);

  robot.initWifi(ssid, password);
  //  robot.initSD();
  robot.setIndex("/", "");

  robot.setControl("/control", []() {
    msgControl = String(robot._server.arg(0));
    msgControl.trim();
    robot._server.send(200);
  });
  //  robot.setVoice("/upload");
  robot.begin();

}

//--------------------------------loop-------------------------
void loop()
{
  robot._server.handleClient();
  if (msgControl != "") {
    Serial.println(msgControl);
    error = deserializeJson(doc, msgControl.c_str());
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    if (doc["cmd"] == "joy") {
      //      Serial.printf("[WSc] joy text: %s\n", doc["data"]["y"]["x"]["z"]);
      Serial.printf("%d : ", doc["data"]["y"]);
      Serial.printf("%d : ", doc["data"]["x"]);
      Serial.printf("%d\n", doc["data"]["z"]);
      linear_vel_y = doc["data"]["y"];
      linear_vel_x = doc["data"]["x"];
      angular_vel_z = doc["data"]["z"];
      rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
      Serial.print("speed_motor_1 :");
      run_motor(motor1_a, motor1_b, motor1_e, rpm.motor1);
      Serial.print("speed_motor_2 :");
      run_motor(motor2_a, motor2_b, motor2_e,  rpm.motor2);
      Serial.print("speed_motor_3 :");
      run_motor(motor3_a, motor3_b, motor3_e,  rpm.motor3);
      Serial.print("speed_motor_4 :");
      run_motor(motor4_a, motor4_b, motor4_e,  rpm.motor4);

    }
    if (doc["cmd"] == "Toy") {
      String power = doc["data"].as<String>();
      Serial.println(power);
      if (power == "on") {
        toyEn = true;
      } else if (power == "off") {
        toyEn = false;
      }
    }
    if (doc["cmd"] == "ToySpeed") {
      int power = doc["data"].as<int>();
      toySpeed = map(power, 10, 100, 10, 1);
      //      Serial.println(power);
      //      if (power == "on") {
      //        toyEn = true;
      //      } else if (power == "off") {
      //        toyEn = false;
      //      }
    }
    if (doc["cmd"] == "laser") {
      String power = doc["data"].as<String>();
      Serial.println(power);
      if (power == "on") {
        pwm.setPWM(laser, 4096, 0);
      } else if (power == "off") {
        pwm.setPWM(laser, 0, 4096);
      }

    }

    if (doc["cmd"] == "Screen") {
      if (doc["data"] == "love") {
        displayMessage("LOVE");
      }


      if (doc["data"] == "smile") {
        displayMessage("SMILE");
      }

      if (doc["data"] == "sad") {
        displayMessage("SAD");
      }
      if (doc["data"] == "begging face") {
        displayMessage("BEGGING FACE");
      }
    }

    /*
          error = deserializeJson(doc, msgControl.c_str());
          if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
          }*/
    /*if (doc["cmd"] == "laser") {
      String power = doc["data"].as<String>();
      Serial.println(power);
      if (power == "off") {
        turnOnLED();
      } else if (power == "on") {
        turnOffLED();
      }
      }*/
    //    if (doc["cmd"] == "Toy") {
    //      Serial.printf(doc["data"]);
    //      for (int i = 0; i <= 180; i++) {
    //        meen.write(i);  // Set the servo position
    //        w_ait((int)doc["data"]);
    //      }
    //      for (int i = 180; i >= 0; i--) {
    //        meen.write(i);  // Set the servo position
    //        w_ait((int)doc["data"]);
    //      }
    //    }
    //    if (doc["cmd"] == "joy") {
    //      Serial.printf("[WSc] joy text: %s\n", doc["data"]["y"]["x"]["z"]);
    //      Serial.printf("%s\n", doc["data"]["y"]);
    //      Serial.printf("%s\n", doc["data"]["x"]);
    //      Serial.printf("%s\n", doc["data"]["z"]);
    //      linear_vel_y = doc["data"]["y"];
    //      linear_vel_x = doc["data"]["x"];
    //      angular_vel_z = doc["data"]["z"];
    //      rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
    //      Serial.print("speed_motor_1 :");
    //      run_motor(motor1_a, motor1_b, motor1_e, rpm.motor1);
    //      Serial.print("speed_motor_2 :");
    //      run_motor(motor2_a, motor2_b, motor2_e,  rpm.motor2);
    //      Serial.print("speed_motor_3 :");
    //      run_motor(motor3_a, motor3_b, motor3_e,  rpm.motor3);
    //      Serial.print("speed_motor_4 :");
    //      run_motor(motor4_a, motor4_b, motor4_e,  rpm.motor4);
    //
    //    }
    //    if (doc["cmd"] == "Screen") {
    //      if (doc["data"] == "love") {
    //        face8 ;
    //        display.clearDisplay();
    //        display.drawBitmap(0, 0, face8 , 128, 64, WHITE);
    //        display.display();
    //      }
    //      if (doc["data"] == "smile") {
    //        face9  ;
    //        display.clearDisplay();
    //        display.drawBitmap(0, 0, face9 , 128, 64, WHITE);
    //        display.display();
    //      }
    //      if (doc["data"] == "sad") {
    //        face10;
    //        display.clearDisplay();
    //        display.drawBitmap(0, 0, face10 , 128, 64, WHITE);
    //        display.display();
    //      }
    //      if (doc["data"] == "begging face") {
    //        face7 ;
    //        display.clearDisplay();
    //        display.drawBitmap(0, 0, face7 , 128, 64, WHITE);
    //        display.display();
    //      }
    //    }
    msgControl = "";
  }

}
void displayMessage(const char* message) {
  // Clear the screen
  tft.fillScreen(TFT_BLACK);

  // Get the dimensions of the display
  int screenWidth = tft.width();
  int screenHeight = tft.height();

  // Calculate the width of the message using the current text size
  int messageWidth = tft.textWidth(message);

  // Calculate the x-coordinate to center the message horizontally
  int xCenter = (screenWidth - messageWidth) / 2;

  // Calculate the y-coordinate to center the message vertically
  int yCenter = (screenHeight - tft.fontHeight()) / 2;

  // Set the cursor to the calculated coordinates
  tft.setCursor(xCenter, yCenter);

  // Print the message in the middle of the screen
  tft.println(message);

//  delay(1000); // Wait for 5 seconds before clearing the screen and repeating
}
