#include <Arduino.h>
#include <SPI.h>
#include "mcp2515.h"
#include "can.h"

// Define UART2 TX/RX pins (adjust as needed)
#define UART2_TX 35
#define UART2_RX 34

// CAN MCP2515
struct can_frame canMsg;
MCP2515 mcp2515(5); // CS on GPIO5

// Custom struct for UART transmission to STM32
typedef struct {
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} CANFrame;

void setup() {
  Serial.begin(115200);  // Debug UART
  Serial2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);  // UART to STM32

  while (!Serial) delay(10);
  Serial.println("ESP32 (CAN → UART) Starting...");

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("MCP2515 initialized");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Debug output
    Serial.print("ID: ");
    Serial.print(canMsg.can_id, HEX);
    Serial.print(" DLC: ");
    Serial.print(canMsg.can_dlc);
    Serial.print(" Data: ");
    for (uint8_t i = 0; i < canMsg.can_dlc; i++) {
      Serial.printf("%02X ", canMsg.data[i]);
    }
    Serial.println();

    // Prepare and send frame to STM32
    CANFrame frame;
    frame.can_id = canMsg.can_id;
    frame.dlc = canMsg.can_dlc;
    memcpy(frame.data, canMsg.data, 8); // fill even unused bytes

    Serial2.write((uint8_t *)&frame, sizeof(CANFrame));
  }

  delay(20);  // Moderate read rate
}

