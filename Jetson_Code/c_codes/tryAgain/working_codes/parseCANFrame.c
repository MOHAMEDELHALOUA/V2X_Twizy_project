// parseCANFrame.h
#ifndef PARSECANFRAME_H
#define PARSECANFRAME_H
#include <stdint.h>

typedef struct {
    unsigned int can_id;
    unsigned char dlc;
    unsigned char data[8];
} CANFrame;

typedef struct {
    // Basic vehicle data
    unsigned short SOC;
    float speedKmh;
    float odometerKm;
    float displaySpeed;
    
    // Extended battery data
    float batteryCurrent;
    int motorTemperature;
    int batterySOH;
    float batteryVoltage;
    
    // Vehicle status
    int remainingRange;
    char gearPosition[10];
    int acceleratorPercent;
    
    // Additional drive status (from 0x59B)
    int brakePressed;           // 0=not pressed, 1=pressed
    int motorOn;               // 0=off, 1=on
    int powerTorque;           // Power/Torque value (100=zero point)
    float capacitorVoltage;    // Capacitor voltage in V
    
    // Parsing metadata
    uint32_t parsed_can_id;
    int valid;
    uint8_t MacAddress[6];
} ParsedData;

ParsedData parseCANFrame(CANFrame *frame);

#endif

// parseCANFrame.c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "parseCANFrame.h"

ParsedData parseCANFrame(CANFrame *frame) {
    ParsedData result = {0}; // Initialize all fields to 0
    result.valid = 0; // Default to invalid
    result.parsed_can_id = frame->can_id;
    
    switch (frame->can_id) {
        
        case 0x155: // BMS Status 1 - RELIABLE SOC
            if (frame->dlc >= 8) {
                // SOC calculation (bytes 4+5) - VERIFIED WORKING
                // Example: 5B 60 = 0x5B60 = 23,392 → 23,392/400 = 58.48%
                uint16_t socRaw = (frame->data[4] << 8) | frame->data[5];
                result.SOC = (unsigned short)((socRaw * 100) / 40000); // Convert to percentage
                
                // Note: Current calculation needs debugging - commenting out for now
                // result.batteryCurrent = 0.0; // TODO: Fix current calculation
                
                result.valid = 1;
            }
            break;
            
        case 0x5D7: // Vehicle Data 2 - RELIABLE SPEED & ODOMETER
            if (frame->dlc >= 7) {
                // Speed (bytes 0+1) - VERIFIED: 00 00 = 0 km/h ✓
                uint16_t speedRaw = (frame->data[1] << 8) | frame->data[0];
                result.speedKmh = (float)speedRaw / 100.0;
                
                // Odometer (bytes 2-5) - FIXED: Big Endian byte order
                // 00 55 79 40 = 0x00557940 = 5,601,600 / 1600 = 3,501 km ✓
                uint32_t odometer = (frame->data[2] << 24) | (frame->data[3] << 16) | 
                                   (frame->data[4] << 8) | frame->data[5];
                result.odometerKm = (float)odometer / 1600.0;
                
                result.valid = 1;
            }
            break;
            
        case 0x59B: // Drive Status 1 - COMPREHENSIVE DRIVE DATA
            if (frame->dlc >= 8) {
                // Byte 0: Gear Selection - VERIFIED: 20 = Neutral ✓
                uint8_t gear = frame->data[0];
                switch(gear) {
                    case 0x80: strcpy(result.gearPosition, "Drive"); break;
                    case 0x20: strcpy(result.gearPosition, "Neutral"); break;
                    case 0x08: strcpy(result.gearPosition, "Reverse"); break;
                    default: strcpy(result.gearPosition, "Unknown"); break;
                }
                
                // Byte 1: Brake/Motor Status - NEW
                uint8_t brakeMotor = frame->data[1];
                result.brakePressed = (brakeMotor & 0x01) ? 1 : 0; // Bit 0: Brake pedal
                result.motorOn = (brakeMotor & 0x0C) ? 1 : 0;      // Bits 2+3: Motor on
                
                // Byte 2: Power/Torque - NEW (100 = zero point)
                result.powerTorque = (int)frame->data[2];
                
                // Byte 3: Accelerator Position - VERIFIED: 00 = 0% ✓
                uint8_t accelerator = frame->data[3];
                result.acceleratorPercent = (int)((accelerator * 100) / 0xFD); // FD = max value
                
                // Byte 4: Additional brake/motor status (unused for now)
                
                // Byte 5: Capacitor Voltage - NEW
                result.capacitorVoltage = (float)frame->data[5] / 2.0; // Voltage = Value ÷ 2
                
                result.valid = 1;
            }
            break;
            
        case 0x599: // Vehicle Data 1 - RELIABLE DISPLAY SPEED & RANGE
            if (frame->dlc >= 8) {
                // Display Speed (bytes 6+7) - VERIFIED: 00 00 = 0.0 km/h ✓
                uint16_t speedRaw = (frame->data[7] << 8) | frame->data[6];
                if (speedRaw != 0xFFFF) {
                    result.displaySpeed = (float)speedRaw / 100.0;
                }
                
                // Remaining Range (byte 5) - Shows reasonable values (18-24 km)
                result.remainingRange = (int)frame->data[5];
                
                // Note: Odometer from this message doesn't match display - skip for now
                // result.odometerKm = 0.0; // TODO: Debug odometer calculation
                
                result.valid = 1;
            }
            break;
            
        // TODO: Add more IDs as they are verified
        // case 0x196: // Motor Temperature - needs verification
        // case 0x19F: // Motor Speed - needs debugging (shows high speeds when idle)
        // case 0x424: // Battery SOH - needs verification  
        // case 0x425: // Battery Voltage - needs verification
        
        default:
            // Unknown or unverified CAN ID - return invalid result
            result.valid = 0;
            break;
    }
    
    return result;
}
