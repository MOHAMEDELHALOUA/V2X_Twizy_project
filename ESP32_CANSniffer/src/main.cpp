//Fixed ESP32 code - sending text data to jetson via USB Serial:
#include <Arduino.h>
#include <SPI.h>
#include "mcp2515.h"
#include "can.h"

struct can_frame canMsg;
MCP2515 mcp2515(5);  // CS = GPIO5

void setup() {
  // Serial Monitor for debugging AND sending to Jetson
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  // Initialize CAN
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  // Startup message (will appear once)
  Serial.println("CAN Bus Reader Ready - Sending to Jetson via USB");
  Serial.println("------------------------");
  delay(2000);  // Give time to see the startup message
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    
    // ----------- Send to JETSON via USB Serial -----------
    Serial.print("SIM ");
    
    // Send CAN ID in HEX with leading zeros
    if (canMsg.can_id < 0x100) Serial.print("0");
    if (canMsg.can_id < 0x10) Serial.print("0");
    Serial.print(canMsg.can_id, HEX);
    Serial.print(" ");
    
    // Send DLC
    Serial.print(canMsg.can_dlc, DEC);
    
    // Send data bytes (always 8 bytes, pad with 00 if needed)
    for (int i = 0; i < 8; i++) {
      Serial.print(" ");
      if (i < canMsg.can_dlc) {
        if (canMsg.data[i] < 0x10) Serial.print("0");
        Serial.print(canMsg.data[i], HEX);
      } else {
        Serial.print("00");  // Pad unused bytes
      }
    }
    
    // End the line
    Serial.println();
  }
  
  delay(50);
}
