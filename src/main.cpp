#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"
#include <EEPROM.h>

//================================================================================================================================================
//                                                WiFi Configuration - AP (Access Point) Mode

const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";
const char* AP_SSID = "";  // Change to your desired AP name

//================================================================================================================================================
//                                                              Pin Definitions

// GPIO Pin Assignments (adjust based on our ESP32 board)
const int PIN_ENCODER_A = 1;      // Encoder A phase
const int PIN_ENCODER_B = 2;      // Encoder B phase
const int PIN_SDA = 22;           // I2C SDA (Note: Wire.begin() uses default pins)
const int PIN_SCL = 23;           // I2C SCL
const int PIN_LIMIT_SW = 16;      // Limit Switch input
const int PIN_LIMIT_SW_EN = 17;   // Limit Switch Enable
const int PIN_MOTOR_1 = 20;       // Motor Input 1
const int PIN_MOTOR_2 = 18;   
//================================================================================================================================================
//                                                              Setup Function
MS5837 pressureSensor;

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize pressure sensor
  while (!pressureSensor.init()) {
    Serial.println("Pressure sensor init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    delay(5000);
  }
  
  // Set fluid density
  pressureSensor.setFluidDensity(997);  // Freshwater in kg/m^3
  
  Serial.println("Pressure sensor initialized!");
}

//================================================================================================================================================
//                                                              Main Loop

void loop() {
  // Your main code here
  
  delay(1000);
}