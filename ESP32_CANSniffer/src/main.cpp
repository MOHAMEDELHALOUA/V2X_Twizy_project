#include <Arduino.h>

// Define the CAN frame structure
typedef struct {
  uint32_t can_id;  // 32-bit CAN ID
  uint8_t dlc;      // Data Length Code (0-8)
  uint8_t data[8];  // CAN data
} CANFrame;

// Send a simulated CAN frame over USB serial (to Raspberry Pi)
void sendSimulatedCANFrame(uint32_t can_id, uint8_t dlc, uint8_t* data) {
  CANFrame frame;
  frame.can_id = can_id;
  frame.dlc = dlc;
  memcpy(frame.data, data, 8);  // Always copy 8 bytes for consistency

  // Send to Raspberry Pi over USB Serial
  Serial.write((uint8_t*)&frame, sizeof(CANFrame));

  // Debug print to monitor on Serial Monitor (optional)
  Serial.print("SIM ");
  if (can_id < 0x100) Serial.print("0");
  if (can_id < 0x10) Serial.print("0");
  Serial.print(can_id, HEX);
  Serial.print(" ");
  Serial.print(dlc, DEC);
  Serial.print(" ");
  for (int i = 0; i < dlc; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);  // Initialize USB Serial connection
  delay(1000);
  Serial.println("ESP32 CAN USB simulator started");
}

void loop() {
  static uint32_t lastSendTime = 0;

  if (millis() - lastSendTime > 2000) {
    lastSendTime = millis();

    // Simulate a CAN frame
    uint32_t can_id = 0x155;  // Example ID
    uint8_t dlc = 8;          // Data Length Code
    uint8_t data[8] = {0x10, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

    sendSimulatedCANFrame(can_id, dlc, data);
  }
}

