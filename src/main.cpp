#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>  
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
void save_depth();
void load_depth();
void IRAM_ATTR encoder_isr();
bool dive_to_depth(float target_depth_m);
bool surface();
bool hold_depth(float target_depth_m, unsigned long duration_ms);
//void auto_dive_cycle();
//void depth_test(int repetitions);
//void motor_test();

//================================================================================================================================================
//                                                              Encoder ISR (Optional)

// Interrupt Service Routine for encoder (for diagnostics only)
void IRAM_ATTR encoder_isr() {
  int a = digitalRead(PIN_ENCODER_A);
  if (a != a_prev) {
    delta++;
    encoder_count++;
    a_prev = a;
  }
}

//================================================================================================================================================
//                                                              Setup Function

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== NanoFloat Depth-Based Control ===");
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  load_depth();
  Serial.print("Last recorded depth: ");
  Serial.print(current_depth);
  Serial.println(" m");
  
  // Initialize GPIO Pins
  pinMode(PIN_ENCODER_A, INPUT);
  pinMode(PIN_ENCODER_B, INPUT);
  pinMode(PIN_LIMIT_SW, INPUT_PULLDOWN);
  pinMode(PIN_LIMIT_SW_EN, OUTPUT);
  pinMode(PIN_MOTOR_1, OUTPUT);
  pinMode(PIN_MOTOR_2, OUTPUT);
  pinMode(PIN_UNUSED_3, OUTPUT);
  pinMode(PIN_UNUSED_8, OUTPUT);
  
  // Enable limit switch
  digitalWrite(PIN_LIMIT_SW_EN, HIGH);
  
  // Initialize motor to stopped
  piston_stop();
  
  // Attach encoder interrupt (optional, for diagnostics)
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoder_isr, CHANGE);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize pressure sensor
  Serial.println("Initializing pressure sensor...");
  int attempts = 0;
  while (!pressureSensor.init() && attempts < 10) {
    Serial.println("Pressure sensor init failed! Retrying...");
    delay(2000);
    attempts++;
  }
  
  if (attempts >= 10) {
    Serial.println("ERROR: Could not initialize pressure sensor!");
    while(1) { delay(1000); }  // Halt - sensor is critical
  }
  
  pressureSensor.setFluidDensity(997);  // Freshwater (use 1029 for seawater)
  Serial.println("Pressure sensor initialized!");
  
  // Read initial depth
  current_depth = read_depth();
  Serial.print("Current depth: ");
  Serial.print(current_depth);
  Serial.println(" m");
  
//   Initialize WiFi in Access Point mode
//   Serial.println("Starting WiFi Access Point...");
//   WiFi.mode(WIFI_AP);
//   if (WiFi.softAP(AP_SSID, WIFI_PASSWORD, 1, 0, 1)) {
//     Serial.println("Access Point started successfully");
//     Serial.print("AP IP address: ");
//     Serial.println(WiFi.softAPIP());
//   } else {
//     Serial.println("Access Point failed to start");
//   }
  
  Serial.println("\n=== NanoFloat Ready ===");
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║          COMPETITION MODE - TASK 4.1      ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\nAvailable Commands:");
  Serial.println("  competition_mission()  - Run full competition task");
  Serial.println("  vertical_profile(1)    - Test single profile");
  Serial.println("  dive_to_depth(2.5)     - Dive to specific depth");
  Serial.println("  hold_depth(2.5, 30000) - Hold depth for time");
  Serial.println("  surface()              - Return to surface");
  Serial.println("  depth_test(10)         - Test sensor readings");
  Serial.println("  motor_test()           - Test motor control\n");
}

//================================================================================================================================================
//                                                              Main Loop

void loop() {
  // Continuously monitor depth
  static unsigned long lastRead = 0;
  if (millis() - lastRead > 1000) {  // Read every second
    current_depth = read_depth();
    
    Serial.print("Depth: ");
    Serial.print(current_depth, 2);
    Serial.print(" m | Temp: ");
    Serial.print(pressureSensor.temperature(), 1);
    Serial.print(" C | Encoder: ");
    Serial.println(encoder_count);
    
    save_depth();
    lastRead = millis();
  }
  
  // Check for limit switch
  if (digitalRead(PIN_LIMIT_SW) == HIGH) {
    piston_stop();
    Serial.println("WARNING: Limit switch triggered!");
  }
  
  delay(100);
}

//================================================================================================================================================
//                                                              Piston Control Functions

void piston_out() {
  digitalWrite(PIN_MOTOR_1, HIGH);
  digitalWrite(PIN_MOTOR_2, LOW);
}

void piston_in() {
  digitalWrite(PIN_MOTOR_1, LOW);
  digitalWrite(PIN_MOTOR_2, HIGH);
}

void piston_stop() {
  digitalWrite(PIN_MOTOR_1, LOW);
  digitalWrite(PIN_MOTOR_2, LOW);
}

//================================================================================================================================================
//                                                              Depth Reading

float read_depth() {
  pressureSensor.read();
  float depth = pressureSensor.depth();
  
  // Clamp to reasonable values
  if (depth < 0) depth = 0;
  if (depth > MAX_DEPTH) depth = MAX_DEPTH;
  
  return depth;
}

//================================================================================================================================================
//                                                              Depth Management

void save_depth() {
  EEPROM.put(EEPROM_DEPTH_ADDR, current_depth);
  EEPROM.commit();
}

void load_depth() {
  EEPROM.get(EEPROM_DEPTH_ADDR, current_depth);
  if (current_depth < 0 || current_depth > MAX_DEPTH) {
    current_depth = 0.0;
  }
}

//================================================================================================================================================
//                                                              Depth-Based Movement

bool dive_to_depth(float target_depth_m) {
  Serial.print("Diving to ");
  Serial.print(target_depth_m);
  Serial.println(" m...");
  
  // Safety check
  if (target_depth_m > MAX_DEPTH) {
    Serial.println("ERROR: Target depth exceeds maximum safe depth!");
    return false;
  }
  
  if (target_depth_m < MIN_DEPTH) {
    Serial.println("ERROR: Target depth below minimum!");
    return false;
  }
  
  target_depth = target_depth_m;
  unsigned long start_time = millis();
  
  // Determine direction
  current_depth = read_depth();
  
  if (current_depth < target_depth) {
    // Need to go deeper - extend piston
    Serial.println("Extending piston (diving deeper)...");
    piston_out();
    
    while (current_depth < (target_depth - depth_tolerance)) {
      current_depth = read_depth();
      
      // Safety checks
      if (millis() - start_time > MAX_MOTOR_TIME) {
        piston_stop();
        Serial.println("ERROR: Motor timeout!");
        return false;
      }
      
      if (digitalRead(PIN_LIMIT_SW) == HIGH) {
        piston_stop();
        Serial.println("ERROR: Limit switch hit!");
        return false;
      }
      
      Serial.print("Current depth: ");
      Serial.print(current_depth, 2);
      Serial.println(" m");
      delay(500);
    }
    
  } else if (current_depth > target_depth) {
    // Need to go shallower - retract piston
    Serial.println("Retracting piston (ascending)...");
    piston_in();
    
    while (current_depth > (target_depth + depth_tolerance)) {
      current_depth = read_depth();
      
      // Safety checks
      if (millis() - start_time > MAX_MOTOR_TIME) {
        piston_stop();
        Serial.println("ERROR: Motor timeout!");
        return false;
      }
      
      if (digitalRead(PIN_LIMIT_SW) == HIGH) {
        piston_stop();
        Serial.println("ERROR: Limit switch hit!");
        return false;
      }
      
      Serial.print("Current depth: ");
      Serial.print(current_depth, 2);
      Serial.println(" m");
      delay(500);
    }
  }
  
  piston_stop();
  current_depth = read_depth();
  Serial.print("Reached target! Final depth: ");
  Serial.print(current_depth, 2);
  Serial.println(" m");
  
  return true;
}

bool surface() {
  Serial.println("Surfacing...");
  return dive_to_depth(0.0);
}

bool hold_depth(float target_depth_m, unsigned long duration_ms) {
  Serial.print("Holding depth ");
  Serial.print(target_depth_m);
  Serial.print(" m for ");
  Serial.print(duration_ms / 1000);
  Serial.println(" seconds...");
  
  unsigned long start_time = millis();
  
  while (millis() - start_time < duration_ms) {
    current_depth = read_depth();
    
    // Actively maintain depth
    if (current_depth < (target_depth_m - depth_tolerance)) {
      piston_out();
      delay(200);
      piston_stop();
    } else if (current_depth > (target_depth_m + depth_tolerance)) {
      piston_in();
      delay(200);
      piston_stop();
    }
    
    Serial.print("Holding at ");
    Serial.print(current_depth, 2);
    Serial.println(" m");
    delay(1000);
  }
  
  piston_stop();
  Serial.println("Hold complete");
  return true;
}
