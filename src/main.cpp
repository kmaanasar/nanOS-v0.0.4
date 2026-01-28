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
const int PIN_ENCODER_A = 1;      // Encoder A phase (optional, back-up for diagnostics)
const int PIN_ENCODER_B = 2;      // Encoder B phase (optional, back-up for diagnostics)
const int PIN_UNUSED_3 = 21;      // UNUSED
const int PIN_SDA = 22;           // I2C SDA (Note: Wire.begin() uses default pins)
const int PIN_SCL = 23;           // I2C SCL
const int PIN_LIMIT_SW = 16;      // Limit Switch input
const int PIN_LIMIT_SW_EN = 17;   // Limit Switch Enable
const int PIN_UNUSED_8 = 19;      // UNUSED
const int PIN_MOTOR_1 = 20;       // Motor Input 1
const int PIN_MOTOR_2 = 18;   

//================================================================================================================================================
//                                                              Global Variables

// Depth control parameters
float current_depth = 0.0;        // Current depth in meters
float target_depth = 0.0;         // Target depth in meters
float depth_tolerance = 0.2;      // Acceptable depth error in meters (20 cm)

// Safety limits
const float MAX_DEPTH = 30.0;     // Maximum safe depth in meters
const float MIN_DEPTH = 0.0;      // Minimum depth (surface)
const unsigned long MAX_MOTOR_TIME = 120000;  // Max motor run time: 2 minutes

// Encoder tracking (optional, for diagnostics only)
volatile int encoder_count = 0;
volatile int delta = 0;
volatile int a_prev = 0;

// Dive status
bool good_dive = true;
bool dive_again = true;

// Pressure sensor object
MS5837 pressureSensor;

// EEPROM for storing last known depth
const int EEPROM_DEPTH_ADDR = 0;
const int EEPROM_SIZE = 512;

//================================================================================================================================================
//                                                              Function Prototypes

void piston_out();
void piston_in();
void piston_stop();
float read_depth();
bool dive_to_depth(float target_depth_m);
bool surface();
bool hold_depth(float target_depth_m, unsigned long duration_ms);
void auto_dive_cycle();
void depth_test(int repetitions);
void motor_test();
void save_depth();
void load_depth();
void IRAM_ATTR encoder_isr();

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