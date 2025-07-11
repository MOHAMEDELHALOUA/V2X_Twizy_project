#include <Arduino.h>
#include <SPI.h>
#include "mcp2515.h"
#include "can.h"

struct can_frame canMsg;
MCP2515 mcp2515(5);

void setup() {
  Serial.begin(115200);
  
  // Wait for serial to initialize
  while (!Serial) {
    delay(10);
  }
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  Serial.println("CAN Bus Reader Ready");
  Serial.println("Format: CAN_ID DLC DATA_BYTES");
  Serial.println("------------------------");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    
    // Print CAN ID with proper formatting (always 3 digits)
    if (canMsg.can_id < 0x100) Serial.print("0");
    if (canMsg.can_id < 0x10) Serial.print("0");
    Serial.print(canMsg.can_id, HEX);
    Serial.print(" ");
    
    // Print DLC as decimal (clearer for data length)
    Serial.print(canMsg.can_dlc, DEC);
    Serial.print(" ");
    
    // Print data bytes with proper padding
    for (int i = 0; i < canMsg.can_dlc; i++) {
      if (canMsg.data[i] < 0x10) Serial.print("0");
      Serial.print(canMsg.data[i], HEX);
      Serial.print(" ");
    }
    
    Serial.println(); // End line
  }
  
  delay(50); // Reduced delay for faster response
}

