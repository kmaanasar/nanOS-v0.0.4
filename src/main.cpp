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

// Competition status
bool good_dive = true;
bool dive_again = true;
bool mission_complete = false;

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
bool vertical_profile(int profile_num);
void competition_mission();
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
  Serial.println("\n\n╔════════════════════════════════════════════╗");
  Serial.println("║   NanoFloat Competition Mode - Task 4.1   ║");
  Serial.println("╚════════════════════════════════════════════╝");
  
  // Initialize I2C bus
  Wire.begin();
  
  // Initialize MCP23017 GPIO Expander
  Serial.println("\nInitializing MCP23017 GPIO expander...");
  if (!mcp.begin_I2C(MCP_I2C_ADDRESS)) {
    Serial.println("ERROR: MCP23017 not found! Check I2C connections and address.");
    while (1) { delay(1000); }  // Halt - expander is critical
  }
  Serial.println("✓ MCP23017 initialized!");
  
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
  Serial.println("\nInitializing MS5837 pressure sensor...");
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
  Serial.println("✓ Pressure sensor initialized!");
  
  // Read initial depth
  current_depth = read_depth();
  Serial.print("Current depth: ");
  Serial.print(current_depth);
  Serial.println(" m");
  
  // Initialize WiFi in Access Point mode
  Serial.println("\nStarting WiFi Access Point...");
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(AP_SSID, WIFI_PASSWORD, 1, 0, 1)) {
    Serial.println("✓ Access Point started successfully");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("✗ Access Point failed to start");
  }
  
  // Start Telnet server
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("\n✓ Telnet server started on port 23");
  Serial.print("Connect via: telnet ");
  Serial.println(WiFi.softAPIP());
  
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║          SYSTEM READY FOR MISSION         ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\nMission will auto-start in 10 seconds...");
  Serial.println("(Or call competition_mission() manually)\n");
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
    telnetClient.println("║    NanoFloat Telnet Console - Task 4.1    ║");
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
    
    telnetClient.print("Mission status: ");
    telnetClient.println(mission_complete ? "COMPLETE" : "IN PROGRESS");
    
    telnetClient.println("\nMonitoring...\n");
  }
  
  // Check if client disconnected
  if (telnetClient && !telnetClient.connected()) {
    telnetClient.stop();
    telnetConnected = false;
    Serial.println("Telnet client disconnected");
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
  
  // Read encoder
  read_encoder();
  
  // Auto-start competition mission after 10 seconds
  static bool mission_started = false;
  if (millis() > 10000 && !mission_started && !mission_complete) {
    mission_started = true;
    competition_mission();
  }
  
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
  
  delay(10);
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
  printBoth("✓ Reached target! Final depth: ");
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
  printlnBoth("✓ Hold complete");
  return true;
}

//================================================================================================================================================
//                                                              Competition Functions

// Single vertical profile for Task 4.1
bool vertical_profile(int profile_num) {
  printlnBoth("");
  printBoth("╔════════════════════════════════════════════╗");
  printlnBoth("");
  printBoth("║       VERTICAL PROFILE ");
  printBoth(String(profile_num));
  printlnBoth("                 ║");
  printBoth("╚════════════════════════════════════════════╝");
  printlnBoth("");
  
  // Phase 1: Dive to 2.5 meters
  printlnBoth("\n[Phase 1] Diving to 2.5 meters...");
  if (!dive_to_depth(2.5)) {
    printlnBoth("✗ ERROR: Failed to reach 2.5m!");
    return false;
  }
  printlnBoth("✓ Reached 2.5m");
  
  // Phase 2: Hold at 2.5 meters for 30 seconds
  printlnBoth("\n[Phase 2] Holding at 2.5 meters for 30 seconds...");
  if (!hold_depth(2.5, 30000)) {
    printlnBoth("✗ ERROR: Failed to hold 2.5m!");
    return false;
  }
  printlnBoth("✓ Held 2.5m for 30 seconds");
  
  // Phase 3: Ascend to 0.4 meters (40 cm)
  printlnBoth("\n[Phase 3] Ascending to 0.4 meters (40 cm)...");
  if (!dive_to_depth(0.4)) {
    printlnBoth("✗ ERROR: Failed to reach 0.4m!");
    return false;
  }
  printlnBoth("✓ Reached 0.4m");
  
  // Phase 4: Hold at 0.4 meters for 30 seconds
  printlnBoth("\n[Phase 4] Holding at 0.4 meters for 30 seconds...");
  if (!hold_depth(0.4, 30000)) {
    printlnBoth("✗ ERROR: Failed to hold 0.4m!");
    return false;
  }
  printlnBoth("✓ Held 0.4m for 30 seconds");
  
  // Check if float broke surface (penalty condition)
  current_depth = read_depth();
  if (current_depth < 0.1) {  // Less than 10cm = broke surface
    printlnBoth("⚠ WARNING: Float may have broken surface!");
  }
  
  printlnBoth("");
  printBoth("✓ VERTICAL PROFILE ");
  printBoth(String(profile_num));
  printlnBoth(" COMPLETE");
  printlnBoth("");
  
  return true;
}

// Complete competition mission for Task 4.1
void competition_mission() {
  printlnBoth("\n\n");
  printlnBoth("╔════════════════════════════════════════════╗");
  printlnBoth("║                                            ║");
  printlnBoth("║   STARTING COMPETITION MISSION - TASK 4.1  ║");
  printlnBoth("║                                            ║");
  printlnBoth("╚════════════════════════════════════════════╝");
  printlnBoth("");
  
  // Step 1: Communicate with station before descending
  printlnBoth("\n[1/3] Attempting to communicate with station...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long startAttempt = millis();
  bool station_connected = false;
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    printBoth(".");
    delay(500);
  }
  printlnBoth("");
  
  if (WiFi.status() == WL_CONNECTED) {
    printlnBoth("✓ Station communication successful!");
    station_connected = true;
  } else {
    printlnBoth("✗ Station communication FAILED!");
    printlnBoth("Aborting mission - cannot proceed without station contact");
    WiFi.mode(WIFI_AP);
    return;
  }
  
  // Disconnect from station, switch back to AP mode
  WiFi.mode(WIFI_AP);
  delay(2000);
  
  // Step 2: Execute Vertical Profile 1
  printlnBoth("\n[2/3] Executing Vertical Profile 1...");
  delay(2000);
  bool profile1_success = vertical_profile(1);
  
  if (profile1_success) {
    printlnBoth("\n✓ VERTICAL PROFILE 1 COMPLETE");
  } else {
    printlnBoth("\n✗ VERTICAL PROFILE 1 FAILED");
  }
  
  // Wait between profiles
  printlnBoth("\nWaiting 10 seconds before Profile 2...");
  delay(10000);
  
  // Step 3: Execute Vertical Profile 2
  printlnBoth("\n[3/3] Executing Vertical Profile 2...");
  delay(2000);
  bool profile2_success = vertical_profile(2);
  
  if (profile2_success) {
    printlnBoth("\n✓ VERTICAL PROFILE 2 COMPLETE");
  } else {
    printlnBoth("\n✗ VERTICAL PROFILE 2 FAILED");
  }
  
  // Mission complete
  printlnBoth("\n\n");
  printlnBoth("╔════════════════════════════════════════════╗");
  printlnBoth("║                                            ║");
  printlnBoth("║      COMPETITION MISSION COMPLETE!        ║");
  printlnBoth("║                                            ║");
  printlnBoth("╚════════════════════════════════════════════╝");
  
  // Summary
  printlnBoth("\n═══════════ MISSION SUMMARY ═══════════");
  printBoth("Station Communication: ");
  printlnBoth(station_connected ? "✓ SUCCESS" : "✗ FAILED");
  printBoth("Vertical Profile 1: ");
  printlnBoth(profile1_success ? "✓ SUCCESS" : "✗ FAILED");
  printBoth("Vertical Profile 2: ");
  printlnBoth(profile2_success ? "✓ SUCCESS" : "✗ FAILED");
  printlnBoth("════════════════════════════════════════");
  printlnBoth("");
  
  mission_complete = true;
}