//sending text data to jetson:

#include <Arduino.h>
#include <SPI.h>
#include "mcp2515.h"
#include "can.h"

// Use TX = GPIO17, RX = GPIO16 for UART1
#define UART1_TX 17
#define UART1_RX 16  // not used, but required by Serial1.begin()

struct can_frame canMsg;
MCP2515 mcp2515(5);  // CS = GPIO5

void setup() {
  // Serial Monitor for debugging
  Serial.begin(115200);
  while (!Serial) delay(10);

  // UART1 to JETSON
  Serial1.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);  // Only TX used

  // Initialize CAN
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("CAN Bus Reader Ready");
  Serial.println("Format: CAN_ID DLC DATA_BYTES");
  Serial.println("------------------------");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {

    // ----------- Debug print to Serial Monitor -----------
    if (canMsg.can_id < 0x100) Serial.print("0");
    if (canMsg.can_id < 0x10) Serial.print("0");
    Serial.print(canMsg.can_id, HEX);
    Serial.print(" ");

    Serial.print(canMsg.can_dlc, DEC);
    Serial.print(" ");

    for (int i = 0; i < canMsg.can_dlc; i++) {
      if (canMsg.data[i] < 0x10) Serial.print("0");
      Serial.print(canMsg.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // ----------- Send CAN frame to JETSON via Serial1 UART -----------
    // Format: [4 bytes CAN ID][1 byte DLC][8 bytes data]
    Serial1.write((uint8_t *)&canMsg.can_id, 4);         // Send CAN ID
    Serial1.write((uint8_t *)&canMsg.can_dlc, 1);        // Send DLC
    Serial1.write((uint8_t *)canMsg.data, canMsg.can_dlc); // Send data bytes
  }

  delay(50);  // Reduce delay if faster communication is needed
}

