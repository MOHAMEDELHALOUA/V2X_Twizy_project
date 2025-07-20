#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "parseCANFrame.h"

typedef struct {
    unsigned short SOC;
    float speedKmh;
    float odometerKm;
    float displaySpeed;
    // Extended fields based on DBC
    float batteryCurrent;
    int motorTemperature;
    int batterySOH;
    float batteryVoltage;
    int remainingRange;
    char gearPosition[10];
    int acceleratorPercent;
    uint8_t MacAddress[6];
} ParsedData;

// Helper function to get current time in milliseconds
uint32_t get_current_time_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static uint32_t lastParseTime = 0;

ParsedData parseCANFrame(CANFrame *frame) {
    ParsedData result = {0}; // Initialize all fields to 0
    
    uint32_t current_time = get_current_time_ms();
    if (current_time - lastParseTime < 100) {
        return result; // Rate limiting
    }
    lastParseTime = current_time;
    
    switch (frame->can_id) {
        case 0x155: // BMS Status 1
            if (frame->dlc >= 8) {
                // SOC calculation
                uint16_t socRaw = (frame->data[4] << 8) | frame->data[5];
                result.SOC = (unsigned short)(socRaw / 400.0 * 100); // Convert to percentage
                
                // Battery Current
                uint16_t currentRaw = (frame->data[2] << 8) | frame->data[1];
                int16_t currentDeviation = (int16_t)currentRaw - 2000;
                result.batteryCurrent = (float)currentDeviation / 4.0;
                
                printf("Parsed BMS Status 1 - SOC: %u%%, Current: %.1fA\n", 
                       result.SOC, result.batteryCurrent);
            }
            break;
            
        case 0x19F: // Energy Status 3 - Speed
            if (frame->dlc >= 8) {
                uint16_t speedRaw = (frame->data[2] << 8) | frame->data[3];
                int16_t speedDeviation = (int16_t)speedRaw - 0x7D0;
                float speedRPM = (float)speedDeviation * 10.0;
                result.speedKmh = speedRPM / 7250.0 * 80.0;
                
                printf("Parsed Speed - %.1f km/h\n", result.speedKmh);
            }
            break;
            
        case 0x196: // Energy Status 2 - Motor Temperature
            if (frame->dlc >= 8) {
                result.motorTemperature = frame->data[5] - 40;
                printf("Parsed Motor Temperature - %d°C\n", result.motorTemperature);
            }
            break;
            
        case 0x424: // BMS Status 2
            if (frame->dlc >= 8) {
                result.batterySOH = frame->data[5]; // Battery State of Health
                printf("Parsed Battery SOH - %d%%\n", result.batterySOH);
            }
            break;
            
        case 0x425: // BMS Status 3 - Battery Voltage
            if (frame->dlc >= 8) {
                uint16_t voltageRaw = (frame->data[4] << 8) | frame->data[5];
                result.batteryVoltage = (float)(voltageRaw >> 1) / 10.0;
                printf("Parsed Battery Voltage - %.1fV\n", result.batteryVoltage);
            }
            break;
            
        case 0x599: // Vehicle Data 1
            if (frame->dlc >= 8) {
                // Odometer
                uint32_t odometer = (frame->data[0] << 24) | (frame->data[1] << 16) | 
                                   (frame->data[2] << 8) | frame->data[3];
                result.odometerKm = (float)odometer;
                
                // Remaining Range
                result.remainingRange = frame->data[5];
                
                // Display Speed
                uint16_t speedRaw = (frame->data[6] << 8) | frame->data[7];
                if (speedRaw != 0xFFFF) {
                    result.displaySpeed = (float)speedRaw / 100.0;
                }
                
                printf("Parsed Vehicle Data - Odo: %.0fkm, Range: %dkm, Display Speed: %.1fkm/h\n", 
                       result.odometerKm, result.remainingRange, result.displaySpeed);
            }
            break;
            
        case 0x59B: // Drive Status 1
            if (frame->dlc >= 8) {
                // Gear Selection
                uint8_t gear = frame->data[0];
                switch(gear) {
                    case 0x80: strcpy(result.gearPosition, "Drive"); break;
                    case 0x20: strcpy(result.gearPosition, "Neutral"); break;
                    case 0x08: strcpy(result.gearPosition, "Reverse"); break;
                    default: strcpy(result.gearPosition, "Unknown"); break;
                }
                
                // Accelerator Position
                uint8_t accelerator = frame->data[3];
                result.acceleratorPercent = (accelerator * 100) / 253;
                
                printf("Parsed Drive Status - Gear: %s, Accelerator: %d%%\n", 
                       result.gearPosition, result.acceleratorPercent);
            }
            break;
            
        case 0x5D7: // Vehicle Data 2 - Alternative Speed/Odometer
            if (frame->dlc >= 7) {
                // Speed
                uint16_t speedRaw = (frame->data[0] << 8) | frame->data[1];
                result.speedKmh = (float)speedRaw / 100.0;
                
                // Odometer
                uint32_t odometer = (frame->data[2] << 24) | (frame->data[3] << 16) | 
                                   (frame->data[4] << 8) | frame->data[5];
                result.odometerKm = (float)odometer / 1600.0;
                
                printf("Parsed Vehicle Data 2 - Speed: %.1fkm/h, Odo: %.1fkm\n", 
                       result.speedKmh, result.odometerKm);
            }
            break;
            
        default:
            // Unknown CAN ID - return empty result
            break;
    }
    
    return result;
}
