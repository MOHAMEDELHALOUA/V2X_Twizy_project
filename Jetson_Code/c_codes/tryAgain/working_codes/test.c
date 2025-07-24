#include <stdio.h>
#include "parseCANFrame.h"
#include <string.h>

int main() {
    // Frame 1: BMS Status (0x155)
  // 155,8,07 97 CE 54 5D D8 00 6E,07 97 CE 54 5D D8 00 6E
    CANFrame frame1 = {0x155, 8, {0x07, 0x97, 0xCE, 0x54, 0x5D, 0xD8, 0x00, 0x6E}};
    
//    CANFrame frame1 = {0x155, 8, {0x07, 0x97, 0xCA, 0x54, 0x4C, 0xD0, 0x00, 0x6C}};
    
    // Frame 2: Drive Status (0x59B) 
    CANFrame frame2 = {0x59B, 8, {0x80, 0x00, 0x64, 0x00, 0x40, 0x6c, 0x00, 0x90}};
    
    // Frame 3: Vehicle Data 1 (0x599) - Range and Display Speed
  // {"id":"599","data":[0,0,13,172,255,28,255,255]}
    CANFrame frame3 = {0x599, 8, {0x00, 0x00, 0x0D, 0xAC, 0xFF, 0x10, 0x00, 0x00}};
  // 
//    CANFrame frame3 = {0x599, 8, {0x00, 0x00, 0x0D, 0xAC, 0xFF, 0x10, 0x00, 0x00}};
    
    // Frame 4: Vehicle Data 2 (0x5D7) - PRIMARY Speed + Odometer
    CANFrame frame4 = {0x5D7, 7, {0x00, 0x00, 0x00, 0x55, 0x73, 0x00, 0x00}};
    
    // Frame 5: BMS Status 3 (0x425) - Battery Voltage + Available Energy  
    CANFrame frame5 = {0x425, 8, {0x2A, 0x1C, 0x44, 0xFF, 0xFE, 0x3C, 0x01, 0x1D}};
    
    // Parse each frame separately
    ParsedData result1 = parseCANFrame(&frame1);
    ParsedData result2 = parseCANFrame(&frame2);
    ParsedData result3 = parseCANFrame(&frame3);
    ParsedData result4 = parseCANFrame(&frame4);
    ParsedData result5 = parseCANFrame(&frame5);
    
    // Create a merged result
    ParsedData merged = {0};
    
    // Copy data from frame 1 (0x155 - Battery data)
    if (result1.valid) {
        merged.SOC = result1.SOC;
        merged.batteryCurrent = result1.batteryCurrent;
    }
    
    // Copy data from frame 2 (0x59B - Vehicle control data) 
    if (result2.valid) {
        strcpy(merged.gearPosition, result2.gearPosition);
        merged.motorControllerActive = result2.motorControllerActive;
        merged.acceleratorPercent = result2.acceleratorPercent;
        merged.brakePressed = result2.brakePressed;
        merged.powerTorque = result2.powerTorque;
        merged.capacitorVoltage = result2.capacitorVoltage;
    }
    
    // Copy data from frame 3 (0x599 - Range and backup speed data)
    if (result3.valid) {
        merged.remainingRange = result3.remainingRange;
        // Only use this speed if frame4 doesn't provide better speed
        if (result4.valid == 0) {
            merged.speedKmh = result3.speedKmh;  
        }
    }
    
    // Copy data from frame 4 (0x5D7 - PRIMARY speed and odometer)
    if (result4.valid) {
        merged.speedKmh = result4.speedKmh;      // PRIMARY speed (overrides frame3)
        merged.odometerKm = result4.odometerKm;  // PRIMARY odometer
    }
    
    // Copy data from frame 5 (0x425 - Battery voltage and energy)
    if (result5.valid) {
        merged.batteryVoltage = result5.batteryVoltage;
        merged.availableEnergy = result5.availableEnergy;
        merged.chargingProtocolStatus = result5.chargingProtocolStatus;
    }
    
    merged.valid = 1;
    
    // Display results
    printf("=== Individual Frame Results ===\n");
    printf("Frame 0x155: SOC=%d%%, Current=%.1fA\n", result1.SOC, result1.batteryCurrent);
    printf("Frame 0x59B: Gear=%s, Motor=%s, Accel=%d%%, Brake=%s, CapV=%.1fV\n",
           result2.gearPosition,
           result2.motorControllerActive ? "ON" : "OFF", 
           result2.acceleratorPercent,
           result2.brakePressed ? "Active" : "Inactive",
           result2.capacitorVoltage);
    printf("Frame 0x599: Range=%dkm, DisplaySpeed=%.1fkm/h\n", 
           result3.remainingRange, result3.speedKmh);
    printf("Frame 0x5D7: Speed=%.1fkm/h, Odometer=%.1fkm\n", 
           result4.speedKmh, result4.odometerKm);
    printf("Frame 0x425: Voltage=%.1fV, Energy=%.1fkWh, ChargingStatus=0x%02X\n", 
           result5.batteryVoltage, result5.availableEnergy, result5.chargingProtocolStatus);
    
    printf("\n=== Merged Results ===\n");
    printf("SOC: %d%%\n", merged.SOC);
    printf("Current: %.1fA\n", merged.batteryCurrent);
    printf("Battery Voltage: %.1fV\n", merged.batteryVoltage);
    printf("Available Energy: %.1fkWh\n", merged.availableEnergy);
    printf("Gear: %s\n", merged.gearPosition);
    printf("Motor: %s\n", merged.motorControllerActive ? "Active" : "Offline");
    printf("Accelerator: %d%%\n", merged.acceleratorPercent);
    printf("Speed: %.1fkm/h (Primary from 0x5D7)\n", merged.speedKmh);
    printf("Odometer: %.1fkm\n", merged.odometerKm);
    printf("Range: %dkm\n", merged.remainingRange);
    printf("Capacitor: %.1fV\n", merged.capacitorVoltage);
    printf("Charging Status: 0x%02X\n", merged.chargingProtocolStatus);
    
    printf("\nV2V/V2G Status:\n%s\n", getSystemStatusSummary(&merged));
    
    return 0;
}
