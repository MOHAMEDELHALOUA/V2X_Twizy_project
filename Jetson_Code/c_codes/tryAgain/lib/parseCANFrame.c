
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
        case 0x155: // BMS Status 1
            if (frame->dlc >= 8) {
                // SOC calculation (bytes 4+5)
                uint16_t socRaw = (frame->data[5] << 8) | frame->data[4];
                result.SOC = (unsigned short)((socRaw & 0x1FFF) / 400.0 * 100); // Only upper 13 bits
                
                // Battery Current (bytes 2+3)
                uint16_t currentRaw = (frame->data[2] << 8) | frame->data[1];
                int16_t currentDeviation = (int16_t)currentRaw - 0x7D0; // 2000d zero point
                result.batteryCurrent = (float)currentDeviation / 4.0;
                
                result.valid = 1;
            }
            break;
            
        case 0x19F: // Energy Status 3 - Speed
            if (frame->dlc >= 8) {
                // Speed calculation (bytes 3+4)
                uint16_t speedRaw = (frame->data[3] << 8) | frame->data[2];
                int16_t speedDeviation = (int16_t)speedRaw - 0x7D0; // Rest value 2000
                float speedRPM = (float)speedDeviation * 10.0;
                result.speedKmh = speedRPM / 7250.0 * 80.0; // Convert to km/h for T80
                
                result.valid = 1;
            }
            break;
            
        case 0x196: // Energy Status 2 - Motor Temperature
            if (frame->dlc >= 8) {
                result.motorTemperature = (int)frame->data[5] - 40; // Byte 6: Temperature [°C] = Value - 40
                result.valid = 1;
            }
            break;
            
        case 0x424: // BMS Status 2
            if (frame->dlc >= 8) {
                result.batterySOH = (int)frame->data[5]; // Byte 6: Battery State of Health in percent
                result.valid = 1;
            }
            break;
            
        case 0x425: // BMS Status 3 - Battery Voltage
            if (frame->dlc >= 8) {
                // Battery Voltage (bytes 5-6 and 7-8 combined)
                uint16_t voltage1 = (frame->data[5] << 8) | frame->data[4];
                uint16_t voltage2 = (frame->data[7] << 8) | frame->data[6];
                // Use the first voltage reading, bits 9..1 >> 1
                result.batteryVoltage = (float)(voltage1 >> 1) / 10.0;
                result.valid = 1;
            }
            break;
            
        case 0x599: // Vehicle Data 1
            if (frame->dlc >= 8) {
                // Odometer (bytes 1-4)
                uint32_t odometer = (frame->data[3] << 24) | (frame->data[2] << 16) | 
                                   (frame->data[1] << 8) | frame->data[0];
                result.odometerKm = (float)odometer;
                
                // Remaining Range (byte 6)
                result.remainingRange = (int)frame->data[5];
                
                // Display Speed (bytes 7+8)
                uint16_t speedRaw = (frame->data[7] << 8) | frame->data[6];
                if (speedRaw != 0xFFFF) {
                    result.displaySpeed = (float)speedRaw / 100.0;
                }
                
                result.valid = 1;
            }
            break;
            
        case 0x59B: // Drive Status 1
            if (frame->dlc >= 8) {
                // Gear Selection (byte 1)
                uint8_t gear = frame->data[0];
                switch(gear) {
                    case 0x80: strcpy(result.gearPosition, "Drive"); break;
                    case 0x20: strcpy(result.gearPosition, "Neutral"); break;
                    case 0x08: strcpy(result.gearPosition, "Reverse"); break;
                    default: strcpy(result.gearPosition, "Unknown"); break;
                }
                
                // Accelerator Position (byte 4)
                uint8_t accelerator = frame->data[3];
                result.acceleratorPercent = (int)((accelerator * 100) / 253); // 00h=0% ... FDh=100%
                
                result.valid = 1;
            }
            break;
            
        case 0x5D7: // Vehicle Data 2 - Alternative Speed/Odometer
            if (frame->dlc >= 7) {
                // Speed (bytes 1+2)
                uint16_t speedRaw = (frame->data[1] << 8) | frame->data[0];
                result.speedKmh = (float)speedRaw / 100.0;
                
                // Odometer (bytes 3-6)
                uint32_t odometer = (frame->data[5] << 24) | (frame->data[4] << 16) | 
                                   (frame->data[3] << 8) | frame->data[2];
                result.odometerKm = (float)odometer / 1600.0;
                
                result.valid = 1;
            }
            break;
            
        default:
            // Unknown CAN ID - return invalid result
            result.valid = 0;
            break;
    }
    
    return result;
}
