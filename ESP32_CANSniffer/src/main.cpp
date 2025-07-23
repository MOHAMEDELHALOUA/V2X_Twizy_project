// CAN Bus Simulator ESP32 - Sends realistic test data to Jetson
#include <Arduino.h>

// Define realistic CAN message patterns based on your historical data
struct CANMessage {
  String id;
  int dlc;
  String data;
  int frequency_ms;  // How often to send this message
  unsigned long last_sent;
};

// Function declarations (must be before they're used)
void sendCANMessage(String id, int dlc, String data);
void sendDynamicData();
void sendHistoricalBurst();

// Real CAN messages from your historical data
CANMessage canMessages[] = {
  // SOC data (0x155) - Battery State of Charge - Most important for testing
  {"155", 8, "07 97 CE 54 5D D8 00 6E", 100, 0},  // ~58% SOC
  {"155", 8, "07 97 CD 54 5D D8 00 6E", 100, 0},  // Slight SOC variation
  
  // Speed and Odometer (0x5D7) - Critical for speed/distance testing  
  {"5D7", 7, "00 00 00 55 79 40 00", 200, 0},     // 0 km/h, ~3501 km odometer
  
  // Display data (0x599) - Display speed and range
  {"599", 8, "00 00 0D AD FF 18 00 00", 300, 0},  // Display speed and range data
  
  // Drive status (0x59B) - Gear, accelerator, brake, motor status
  {"59B", 8, "20 00 64 00 40 14 00 90", 250, 0},  // Neutral gear, 0% accelerator
  
  // Motor data (0x19F) - Motor speed/status - Very frequent
  {"19F", 8, "FF FF 7D 0F 38 FF 40 FE", 50, 0},   // Motor status data
  
  // Motor temperature (0x196)
  {"196", 8, "00 FF E7 7F 70 4A 0B 00", 500, 0},  // Motor temperature
  
  // Other periodic messages
  {"436", 6, "00 00 00 75 00 00", 400, 0},        // Unknown but periodic
  {"423", 8, "03 2D FF FF 00 E0 00 E4", 600, 0},  // Unknown periodic
  {"69F", 4, "F3 07 61 80", 800, 0},              // Less frequent message
  {"556", 8, "31 13 11 31 13 11 31 2A", 1000, 0}, // Rare message
  {"597", 8, "00 95 08 41 2D 00 01 53", 1200, 0}  // Very rare message
};

const int NUM_MESSAGES = sizeof(canMessages) / sizeof(canMessages[0]);

// Variables for dynamic data simulation
unsigned long startTime;
int socCounter = 0;
int speedCounter = 0;
bool increasing = true;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  startTime = millis();
  
  // Startup message (will appear once)
  Serial.println("CAN Bus Simulator Ready - Sending Historical Data Patterns");
  Serial.println("-----------------------------------------------------------");
  delay(2000);  // Give time to see the startup message
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check each message type and send if it's time
  for (int i = 0; i < NUM_MESSAGES; i++) {
    if (currentTime - canMessages[i].last_sent >= canMessages[i].frequency_ms) {
      sendCANMessage(canMessages[i].id, canMessages[i].dlc, canMessages[i].data);
      canMessages[i].last_sent = currentTime;
    }
  }
  
  // Send some dynamic data every few seconds to simulate changing values
  if (currentTime - startTime > 5000) {  // Every 5 seconds
    sendDynamicData();
    startTime = currentTime;
  }
  
  delay(10);  // Small delay to prevent overwhelming the serial
}

void sendCANMessage(String id, int dlc, String data) {
  Serial.print("SIM ");
  Serial.print(id);
  Serial.print(" ");
  Serial.print(dlc);
  Serial.print(" ");
  Serial.println(data);
}

void sendDynamicData() {
  // Simulate slowly changing SOC (battery percentage)
  socCounter++;
  if (socCounter > 20) {  // Change SOC every ~100 seconds
    if (increasing) {
      // Increase SOC slightly (simulate charging)
      sendCANMessage("155", 8, "07 97 CF 54 5D D8 00 6E");  // ~59% SOC
      if (socCounter > 40) increasing = false;
    } else {
      // Decrease SOC slightly (simulate usage)
      sendCANMessage("155", 8, "07 97 CC 54 5D D8 00 6E");  // ~57% SOC
      if (socCounter > 60) {
        increasing = true;
        socCounter = 0;
      }
    }
  }
  
  // Simulate occasional speed changes (even when stationary, some noise)
  speedCounter++;
  if (speedCounter % 3 == 0) {
    // Slight variations in speed reading
    sendCANMessage("5D7", 7, "00 01 00 55 79 40 00");  // Tiny speed blip
  } else if (speedCounter % 7 == 0) {
    // Odometer increment (very slow)
    sendCANMessage("5D7", 7, "00 00 00 55 79 41 00");  // Odometer +1
  }
  
  // Simulate gear changes occasionally
  if (speedCounter % 10 == 0) {
    sendCANMessage("59B", 8, "80 00 64 00 40 14 00 90");  // Drive gear
    delay(100);
    sendCANMessage("59B", 8, "20 00 64 00 40 14 00 90");  // Back to neutral
  }
  
  // Reset counter
  if (speedCounter > 100) speedCounter = 0;
}

// Alternative function to send a burst of realistic historical data
void sendHistoricalBurst() {
  // This simulates a real capture sequence
  sendCANMessage("59B", 8, "20 00 64 00 40 14 00 90");
  delay(20);
  sendCANMessage("19F", 8, "FF FF 7D 0F 38 FF 40 FE");
  delay(25);
  sendCANMessage("5D7", 7, "00 00 00 55 79 40 00");
  delay(30);
  sendCANMessage("155", 8, "07 97 CE 54 5D D8 00 6E");
  delay(40);
  sendCANMessage("599", 8, "00 00 0D AD FF 18 00 00");
  delay(35);
  sendCANMessage("196", 8, "00 FF E7 7F 70 4A 0B 00");
  delay(45);
  sendCANMessage("436", 6, "00 00 00 75 00 00");
  delay(50);
}
