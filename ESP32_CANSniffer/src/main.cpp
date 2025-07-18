#include <Arduino.h>
#include <SPI.h>
#include "mcp2515.h"
#include "can.h"

// Use TX = GPIO17, RX = GPIO16 for UART1
#define UART1_TX 17
#define UART1_RX 16  // not used, but required by Serial1.begin()

// CAN Frame structure that matches STM32 expectations
typedef struct {
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} CANFrame;

struct can_frame canMsg;
MCP2515 mcp2515(5);  // CS = GPIO5

// Simulation variables
uint32_t lastSimTime = 0;
uint16_t simulated_soc = 8000;        // SOC in raw format (80.00%)
float simulated_speed = 0.0;          // Speed in km/h
uint32_t simulated_odometer = 5600000; // Odometer in raw format (3500 km)
uint16_t simulated_display_speed = 0;  // Display speed in raw format
uint32_t simulation_counter = 0;

// Function to create and send simulated CAN frames
void sendSimulatedCANFrame(uint32_t can_id, uint8_t dlc, uint8_t* data) {
    CANFrame frame;
    frame.can_id = can_id;
    frame.dlc = dlc;
    memcpy(frame.data, data, 8);  // Always copy 8 bytes for consistency
    
    // Send to STM32 via UART
    Serial1.write((uint8_t*)&frame, sizeof(CANFrame));
    
    // Debug print
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

void simulateTwizyData() {
    uint8_t data[8] = {0};
    simulation_counter++;
    
    // Simulate varying speed (0-60 km/h in a cycle)
    simulated_speed = 30 + 25 * sin(simulation_counter * 0.1);
    if (simulated_speed < 0) simulated_speed = 0;
    
    // Simulate SOC slowly decreasing
    if (simulation_counter % 50 == 0) {
        simulated_soc -= 5;  // Decrease SOC slowly
        if (simulated_soc < 2000) simulated_soc = 8000;  // Reset when too low
    }
    
    // Simulate odometer increasing
    if (simulation_counter % 20 == 0) {
        simulated_odometer += 32;  // Increase odometer (0.02 km)
    }
    
    // Convert speed for display
    simulated_display_speed = (uint16_t)(simulated_speed * 100);
    
    // 1. Battery SoC and Current (0x155)
    uint16_t current_raw = 2000 + (rand() % 400) - 200;  // Simulate current around 0A
    data[0] = 0x00;
    data[1] = (current_raw >> 8) & 0xFF;
    data[2] = current_raw & 0xFF;
    data[3] = 0x00;
    data[4] = (simulated_soc >> 8) & 0xFF;
    data[5] = simulated_soc & 0xFF;
    data[6] = 0x00;
    data[7] = 0x00;
    sendSimulatedCANFrame(0x155, 8, data);
    
    delay(20);
    
    // 2. Display Speed and Odometer (0x5D7)
    data[0] = (simulated_display_speed >> 8) & 0xFF;
    data[1] = simulated_display_speed & 0xFF;
    data[2] = (simulated_odometer >> 24) & 0xFF;
    data[3] = (simulated_odometer >> 16) & 0xFF;
    data[4] = (simulated_odometer >> 8) & 0xFF;
    data[5] = simulated_odometer & 0xFF;
    data[6] = 0x00;
    data[7] = 0x00;
    sendSimulatedCANFrame(0x5D7, 6, data);
    
    delay(20);
    
    // 3. Motor Speed and RPM (0x19F)
    uint16_t speed_raw = 0x7D0 + (int16_t)(simulated_speed * 7250.0f / 80.0f / 10.0f);
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = (speed_raw >> 4) & 0xFF;
    data[3] = (speed_raw << 4) & 0xF0;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    sendSimulatedCANFrame(0x19F, 4, data);
    
    delay(20);
    
    // 4. System Uptime (0x436)
    uint32_t uptime_minutes = simulation_counter * 2;  // Simulate uptime
    data[0] = (uptime_minutes >> 24) & 0xFF;
    data[1] = (uptime_minutes >> 16) & 0xFF;
    data[2] = (uptime_minutes >> 8) & 0xFF;
    data[3] = uptime_minutes & 0xFF;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    sendSimulatedCANFrame(0x436, 4, data);
    
    delay(20);
    
    // 5. Charger Status (0x423)
    uint8_t charger_status = (simulation_counter % 100 < 10) ? 0x03 : 0x00;  // Charger on 10% of time
    uint16_t counter = simulation_counter & 0xFFFF;
    data[0] = charger_status;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = (counter >> 8) & 0xFF;
    data[7] = counter & 0xFF;
    sendSimulatedCANFrame(0x423, 8, data);
    
    delay(20);
    
    // 6. Power and Battery Health (0x424)
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 20;  // Max regen power / 500W
    data[3] = 30;  // Max drive power / 500W
    data[4] = 0x00;
    data[5] = 95;  // Battery SOH 95%
    data[6] = 0x00;
    data[7] = 0x00;
    sendSimulatedCANFrame(0x424, 6, data);
    
    delay(20);
    
    // 7. Battery Voltage and Energy (0x425)
    uint16_t voltage_raw = (uint16_t)(580 * 10 * 2);  // 58.0V battery
    data[0] = 0x00;
    data[1] = 50;  // Available energy * 10 (5.0 kWh)
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = (voltage_raw >> 8) & 0xFF;
    data[5] = voltage_raw & 0xFF;
    data[6] = 0x00;
    data[7] = 0x00;
    sendSimulatedCANFrame(0x425, 6, data);
    
    delay(20);
    
    // 8. 12V System (0x597)
    uint8_t dc_current = (uint8_t)(15.0 * 5);  // 3.0A * 5
    int charger_temp = 45 + 40;  // 45°C + 40 offset
    uint8_t protocol = 0x41;  // System ON
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = dc_current;
    data[3] = protocol;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = charger_temp;
    sendSimulatedCANFrame(0x597, 8, data);
}

void setup() {
    // Serial Monitor for debugging
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    // UART1 to STM32
    Serial1.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);  // Only TX used
    
    // Initialize CAN (optional - for real CAN reading when available)
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    
    Serial.println("CAN Bus Simulator Ready");
    Serial.println("Sending simulated Twizy CAN data to STM32");
    Serial.println("Format: CAN_ID DLC DATA_BYTES");
    Serial.println("------------------------");
    
    // Seed random number generator
    randomSeed(analogRead(0));
}

void loop() {
    uint32_t current_time = millis();
    
    // Check for real CAN messages first (if available)
    bool real_can_received = false;
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        real_can_received = true;
        
        // ----------- Debug print to Serial Monitor -----------
        Serial.print("REAL ");
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
        
        // ----------- Send real CAN frame to STM32 via Serial1 UART -----------
        CANFrame frame;
        frame.can_id = canMsg.can_id;
        frame.dlc = canMsg.can_dlc;
        memcpy(frame.data, canMsg.data, 8);  // Always copy 8 bytes
        Serial1.write((uint8_t*)&frame, sizeof(CANFrame));
    }
    
    // Send simulated data every 1 second (only if no real CAN data)
    if (!real_can_received && (current_time - lastSimTime >= 1000)) {
        Serial.println("--- Sending simulated CAN cycle ---");
        simulateTwizyData();
        lastSimTime = current_time;
        Serial.println("--- End of cycle ---");
    }
    
    delay(10);  // Small delay for system stability
}
