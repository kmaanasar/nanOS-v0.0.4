#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "MS5837.h"
#include <EEPROM.h>
#include "config.h"
#include <Adafruit_MCP23X17.h>

//================================================================================================================================================
//                                                              I2C GPIO Expander Setup

// MCP23017 I2C GPIO Expander
Adafruit_MCP23X17 mcp;

// Pin definitions on MCP23017 (0-15)
const int MCP_ENCODER_A = 0;      // Encoder A phase
const int MCP_ENCODER_B = 1;      // Encoder B phase
const int MCP_MOTOR_1 = 2;        // Motor Input 1
const int MCP_MOTOR_2 = 3;        // Motor Input 2
const int MCP_LIMIT_SW = 4;       // Limit Switch input
const int MCP_LIMIT_SW_EN = 5;    // Limit Switch Enable
const int MCP_UNUSED_6 = 6;       // Available for future use
const int MCP_UNUSED_7 = 7;       // Available for future use
const int MCP_UNUSED_8 = 8;       // Available for future use
const int MCP_UNUSED_9 = 9;       // Available for future use
const int MCP_UNUSED_10 = 10;     // Available for future use
const int MCP_UNUSED_11 = 11;     // Available for future use
const int MCP_UNUSED_12 = 12;     // Available for future use
const int MCP_UNUSED_13 = 13;     // Available for future use
const int MCP_UNUSED_14 = 14;     // Available for future use
const int MCP_UNUSED_15 = 15;     // Available for future use

// MCP23017 I2C Address (default is 0x20, check your hardware)
const uint8_t MCP_I2C_ADDRESS = 0x20;

//================================================================================================================================================
//                                                              Global Variables

// Depth control parameters
float current_depth = 0.0;
float target_depth = 0.0;
float depth_tolerance = 0.2;

// Safety limits
const float MAX_DEPTH = 30.0;
const float MIN_DEPTH = 0.0;
const unsigned long MAX_MOTOR_TIME = 120000;

// Encoder tracking (optional, for diagnostics only)
volatile int encoder_count = 0;
volatile int delta = 0;
volatile int a_prev = 0;

// Dive status
bool good_dive = true;
bool dive_again = true;

// Pressure sensor object (direct I2C)
MS5837 pressureSensor;

// EEPROM for storing last known depth
const int EEPROM_DEPTH_ADDR = 0;
const int EEPROM_SIZE = 512;

// Telnet server
WiFiServer telnetServer(23);
WiFiClient telnetClient;
bool telnetConnected = false;

//================================================================================================================================================
//                                                              Function Prototypes

void piston_out();
void piston_in();
void piston_stop();
float read_depth();
void save_depth();
void load_depth();
bool dive_to_depth(float target_depth_m);
bool surface();
bool hold_depth(float target_depth_m, unsigned long duration_ms);
void handleTelnet();
void printBoth(const String &message);
void printlnBoth(const String &message);
void read_encoder();

//================================================================================================================================================
//                                                              Setup Function

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== NanoFloat Depth-Based Control with MCP23017 ===");
  
  // Initialize I2C bus
  Wire.begin();
  
  // Initialize MCP23017 GPIO Expander
  Serial.println("Initializing MCP23017 GPIO expander...");
  if (!mcp.begin_I2C(MCP_I2C_ADDRESS)) {
    Serial.println("ERROR: MCP23017 not found! Check I2C connections and address.");
    while (1) { delay(1000); }  // Halt - expander is critical
  }
  Serial.println("MCP23017 initialized!");
  
  // Configure MCP23017 pins with pull-ups on inputs
  mcp.pinMode(MCP_ENCODER_A, INPUT_PULLUP);
  mcp.pinMode(MCP_ENCODER_B, INPUT_PULLUP);
  mcp.pinMode(MCP_MOTOR_1, OUTPUT);
  mcp.pinMode(MCP_MOTOR_2, OUTPUT);
  mcp.pinMode(MCP_LIMIT_SW, INPUT_PULLUP);
  mcp.pinMode(MCP_LIMIT_SW_EN, OUTPUT);
  
  // Enable limit switch
  mcp.digitalWrite(MCP_LIMIT_SW_EN, HIGH);
  
  // Initialize motor to stopped
  piston_stop();
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  load_depth();
  Serial.print("Last recorded depth: ");
  Serial.print(current_depth);
  Serial.println(" m");
  
  // Initialize pressure sensor
  Serial.println("Initializing MS5837 pressure sensor...");
  int attempts = 0;
  while (!pressureSensor.init() && attempts < 10) {
    Serial.println("Pressure sensor init failed! Retrying...");
    delay(2000);
    attempts++;
  }
  
  if (attempts >= 10) {
    Serial.println("ERROR: Could not initialize pressure sensor!");
    Serial.println("Check connections: White=SDA, Green=SCL");
    while(1) { delay(1000); }  // Halt - sensor is critical
  }
  
  pressureSensor.setFluidDensity(997);  // Freshwater (use 1029 for seawater)
  Serial.println("Pressure sensor initialized!");
  
  // Read initial depth
  current_depth = read_depth();
  Serial.print("Current depth: ");
  Serial.print(current_depth);
  Serial.println(" m");
  
  // Initialize WiFi in Access Point mode
  Serial.println("Starting WiFi Access Point...");
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(AP_SSID, WIFI_PASSWORD, 1, 0, 1)) {
    Serial.println("Access Point started successfully");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Access Point failed to start");
  }
  
  // Start Telnet server
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("\nTelnet server started on port 23");
  Serial.print("Connect via: telnet ");
  Serial.println(WiFi.softAPIP());
  Serial.println("  (Windows: Enable Telnet Client in Windows Features)");
  Serial.println("  (Mac/Linux: telnet 192.168.4.1 23)");
  
  Serial.println("\n=== NanoFloat Ready ===");
  Serial.println("\nAvailable Commands:");
  Serial.println("  dive_to_depth(2.5)     - Dive to specific depth");
  Serial.println("  hold_depth(2.5, 30000) - Hold depth for time");
  Serial.println("  surface()              - Return to surface\n");
}

//================================================================================================================================================
//                                                              Telnet Helper Functions

// Print to both Serial and Telnet
void printBoth(const String &message) {
  Serial.print(message);
  if (telnetConnected && telnetClient.connected()) {
    telnetClient.print(message);
  }
}

void printlnBoth(const String &message) {
  Serial.println(message);
  if (telnetConnected && telnetClient.connected()) {
    telnetClient.println(message);
  }
}

void handleTelnet() {
  // Check for new telnet client
  if (telnetServer.hasClient()) {
    // Disconnect existing client if new one connects
    if (telnetClient && telnetClient.connected()) {
      telnetClient.stop();
    }
    telnetClient = telnetServer.available();
    telnetConnected = true;
    
    // Welcome message
    telnetClient.println("\n╔════════════════════════════════════════════╗");
    telnetClient.println("║    NanoFloat Telnet Console - MCP23017    ║");
    telnetClient.println("╚════════════════════════════════════════════╝\n");
    
    // Show current status
    telnetClient.print("Current depth: ");
    telnetClient.print(current_depth, 2);
    telnetClient.println(" m");
    
    telnetClient.print("Temperature: ");
    telnetClient.print(pressureSensor.temperature(), 1);
    telnetClient.println(" C");
    
    telnetClient.print("Pressure: ");
    telnetClient.print(pressureSensor.pressure(), 2);
    telnetClient.println(" mbar");
    
    telnetClient.print("Encoder count: ");
    telnetClient.println(encoder_count);
    
    telnetClient.println("\nMonitoring pressure sensor data...");
    telnetClient.println("(Updates every second)\n");
  }
  
  // Check if client disconnected
  if (telnetClient && !telnetClient.connected()) {
    telnetClient.stop();
    telnetConnected = false;
    printlnBoth("Telnet client disconnected");
  }
}

//================================================================================================================================================
//                                                              Encoder Reading

// Read encoder state from MCP23017
void read_encoder() {
  static int last_a = 0;
  
  int a = mcp.digitalRead(MCP_ENCODER_A);
  
  if (a != last_a) {
    encoder_count++;
    last_a = a;
  }
}

//================================================================================================================================================
//                                                              Main Loop

void loop() {
  // Handle telnet connections
  handleTelnet();
  
  // Read encoder (since we can't use interrupts with MCP23017 easily)
  read_encoder();
  
  // Continuously monitor depth
  static unsigned long lastRead = 0;
  if (millis() - lastRead > 1000) {  // Read every second
    current_depth = read_depth();
    
    // Print to both Serial and Telnet
    printBoth("Depth: ");
    printBoth(String(current_depth, 2));
    printBoth(" m | Temp: ");
    printBoth(String(pressureSensor.temperature(), 1));
    printBoth(" C | Pressure: ");
    printBoth(String(pressureSensor.pressure(), 2));
    printBoth(" mbar | Encoder: ");
    printlnBoth(String(encoder_count));
    
    save_depth();
    lastRead = millis();
  }
  
  // Check for limit switch
  if (mcp.digitalRead(MCP_LIMIT_SW) == HIGH) {
    piston_stop();
    printlnBoth("WARNING: Limit switch triggered!");
  }
  
  delay(10);  // Small delay for stability
}

//================================================================================================================================================
//                                                              Piston Control Functions

void piston_out() {
  mcp.digitalWrite(MCP_MOTOR_1, HIGH);
  mcp.digitalWrite(MCP_MOTOR_2, LOW);
}

void piston_in() {
  mcp.digitalWrite(MCP_MOTOR_1, LOW);
  mcp.digitalWrite(MCP_MOTOR_2, HIGH);
}

void piston_stop() {
  mcp.digitalWrite(MCP_MOTOR_1, LOW);
  mcp.digitalWrite(MCP_MOTOR_2, LOW);
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
  printBoth("Diving to ");
  printBoth(String(target_depth_m));
  printlnBoth(" m...");
  
  // Safety check
  if (target_depth_m > MAX_DEPTH) {
    printlnBoth("ERROR: Target depth exceeds maximum safe depth!");
    return false;
  }
  
  if (target_depth_m < MIN_DEPTH) {
    printlnBoth("ERROR: Target depth below minimum!");
    return false;
  }
  
  target_depth = target_depth_m;
  unsigned long start_time = millis();
  
  // Determine direction
  current_depth = read_depth();
  
  if (current_depth < target_depth) {
    // Need to go deeper - extend piston
    printlnBoth("Extending piston (diving deeper)...");
    piston_out();
    
    while (current_depth < (target_depth - depth_tolerance)) {
      current_depth = read_depth();
      
      // Safety checks
      if (millis() - start_time > MAX_MOTOR_TIME) {
        piston_stop();
        printlnBoth("ERROR: Motor timeout!");
        return false;
      }
      
      if (mcp.digitalRead(MCP_LIMIT_SW) == HIGH) {
        piston_stop();
        printlnBoth("ERROR: Limit switch hit!");
        return false;
      }
      
      printBoth("Current depth: ");
      printBoth(String(current_depth, 2));
      printlnBoth(" m");
      delay(500);
    }
    
  } else if (current_depth > target_depth) {
    // Need to go shallower - retract piston
    printlnBoth("Retracting piston (ascending)...");
    piston_in();
    
    while (current_depth > (target_depth + depth_tolerance)) {
      current_depth = read_depth();
      
      // Safety checks
      if (millis() - start_time > MAX_MOTOR_TIME) {
        piston_stop();
        printlnBoth("ERROR: Motor timeout!");
        return false;
      }
      
      if (mcp.digitalRead(MCP_LIMIT_SW) == HIGH) {
        piston_stop();
        printlnBoth("ERROR: Limit switch hit!");
        return false;
      }
      
      printBoth("Current depth: ");
      printBoth(String(current_depth, 2));
      printlnBoth(" m");
      delay(500);
    }
  }
  
  piston_stop();
  current_depth = read_depth();
  printBoth("Reached target! Final depth: ");
  printBoth(String(current_depth, 2));
  printlnBoth(" m");
  
  return true;
}

bool surface() {
  printlnBoth("Surfacing...");
  return dive_to_depth(0.0);
}

bool hold_depth(float target_depth_m, unsigned long duration_ms) {
  printBoth("Holding depth ");
  printBoth(String(target_depth_m));
  printBoth(" m for ");
  printBoth(String(duration_ms / 1000));
  printlnBoth(" seconds...");
  
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
    
    printBoth("Holding at ");
    printBoth(String(current_depth, 2));
    printlnBoth(" m");
    delay(1000);
  }
  
  piston_stop();
  printlnBoth("Hold complete");
  return true;
}