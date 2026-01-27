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