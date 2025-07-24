// parseCANFrame.c - V3 Enhanced for V2V/V2G Applications
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "parseCANFrame.h"

ParsedData parseCANFrame(CANFrame *frame) {
    ParsedData result = {0}; // Initialize all fields to 0
    result.valid = 0;
    result.parsed_can_id = frame->can_id;
    result.timestamp = (uint32_t)time(NULL); // Add timestamp for V2V/V2G coordination
    
    switch (frame->can_id) {
        
        case 0x155: // BMS Status 1 - CRITICAL for V2V/V2G
            if (frame->dlc >= 8) {
                // SOC calculation (bytes 4+5) - CORRECTED FORMULA
                uint16_t socRaw = (frame->data[4] << 8) | frame->data[5];
                result.SOC = (unsigned short)(socRaw / 400);
                
                // Battery Current (bytes 1+2) - Essential for V2G direction detection
                uint16_t currentRaw = (frame->data[1] << 8) | frame->data[2];
                uint16_t currentData = currentRaw & 0x0FFF; // Lower 12 bits
                int16_t currentDeviation = currentData - 0x7D0; // 2000 zero point
                result.batteryCurrent = (float)currentDeviation / 4.0; // Current in Amps
                
                result.data_source_priority = 1; // Primary source
                result.valid = 1;
            }
            break;
            
        case 0x19F: // Energy Status 3 - FIXED SPEED CALCULATION
            if (frame->dlc >= 8) {
                // CORRECTED Speed calculation (bytes 2+3)
                // Formula: Value = (byte2 << 4) | (byte3 & 0x0F)
                uint16_t speedValue = (frame->data[2] << 4) | (frame->data[3] & 0x0F);
                int rpm = (speedValue - 2000) * 10;
                result.speedKmh = (float)rpm / 7250.0 * 80.0;
                
                // Only use if primary speed (0x5D7) is not available
                result.data_source_priority = 3; // Tertiary source
                result.valid = 1;
            }
            break;
            
        case 0x196: // Energy Status 2 - MOTOR TEMPERATURE & POWER
            if (frame->dlc >= 8) {
                // Motor Temperature (byte 5) - Critical for thermal management
                result.motorTemperature = (int)frame->data[5] - 40;
                
                // Power Request (bytes 3+4) - Simplified for reliability
                uint16_t powerRaw = (frame->data[3] << 8) | frame->data[4];
                // Basic power indication (complex formula simplified for reliability)
                result.powerRequest = (float)(powerRaw - 254) * 250.0; // Approximate
                
                result.data_source_priority = 1;
                result.valid = 1;
            }
            break;
            
        case 0x425: // BMS Status 3 - CRITICAL for V2G
            if (frame->dlc >= 8) {
                // Charging Protocol Status (byte 0) - V2V/V2G coordination
                result.chargingProtocolStatus = frame->data[0];
                
                // Available Battery Energy (byte 1) - Essential for V2G planning  
                result.availableEnergy = (float)frame->data[1] / 10.0; // kWh
                
                // Battery Voltage (bytes 4+5) - CORRECT FORMULA CONFIRMED
                // Data bytes 5-6 (frame indices 4-5) hold battery voltage as 16-bit big-endian
                // Formula: voltage = ((data[4] << 8) | data[5]) / 1132.0
                // Example: 0xFE3C = 65,084 / 1132.0 = 57.5V ✅
                uint16_t voltageRaw = (frame->data[4] << 8) | frame->data[5];
                result.batteryVoltage = (float)voltageRaw / 1132.0;
                
                result.data_source_priority = 1;
                result.valid = 1;
            }
            break;
            
        case 0x599: // Vehicle Data 1 - BACKUP SPEED & RANGE
            if (frame->dlc >= 8) {
                // Remaining Range (byte 5) - Trip planning for V2V
                result.remainingRange = (int)frame->data[5];
                
                // Display Speed (bytes 6+7) - Backup speed source
                uint16_t speedRaw = (frame->data[7] << 8) | frame->data[6];
                if (speedRaw != 0xFFFF) {
                    result.speedKmh = (float)speedRaw / 100.0;
                    result.data_source_priority = 2; // Secondary source
                }
                
                result.valid = 1;
            }
            break;
            
        case 0x5D7: // Vehicle Data 2 - PRIMARY SPEED & ODOMETER
            if (frame->dlc >= 7) {
                // PRIMARY Speed (bytes 0+1) - Most reliable for V2V
                uint16_t speedRaw = (frame->data[1] << 8) | frame->data[0];
                result.speedKmh = (float)speedRaw / 100.0;
                
                // PRIMARY Odometer (bytes 2-5) - Vehicle tracking
                uint32_t odometer = (frame->data[2] << 24) | (frame->data[3] << 16) | 
                                   (frame->data[4] << 8) | frame->data[5];
                result.odometerKm = (float)odometer / 1600.0;
                
                result.data_source_priority = 1; // Primary source
                result.valid = 1;
            }
            break;
            
        case 0x59B: // Drive Status 1 - COMPREHENSIVE VEHICLE STATE
            if (frame->dlc >= 8) {
                // Gear Selection (byte 0) - Vehicle state for V2V
                uint8_t gear = frame->data[0];
                switch(gear) {
                    case 0x80: strcpy(result.gearPosition, "Drive"); break;
                    case 0x20: strcpy(result.gearPosition, "Neutral"); break;
                    case 0x08: strcpy(result.gearPosition, "Reverse"); break;
                    default: strcpy(result.gearPosition, "Unknown"); break;
                }
                
                // Motor/Controller Status (byte 1) - Critical for V2V readiness
                uint8_t brakeMotor = frame->data[1];
                result.brakePressed = (brakeMotor & 0x01) ? 1 : 0;
                result.motorControllerActive = (brakeMotor & 0x0C) ? 1 : 0;
                
                // Power/Torque (byte 2) - Vehicle dynamics for V2V
                result.powerTorque = (int)frame->data[2];
                
                // Accelerator Position (byte 3) - Driver intent for V2V
                uint8_t accelerator = frame->data[3];
                result.acceleratorPercent = (int)((accelerator * 100) / 0xFD);
                
                // Capacitor Voltage (byte 5) - System health monitoring
                result.capacitorVoltage = (float)frame->data[5] / 2.0;
                
                result.data_source_priority = 1;
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

// Helper function to determine data source priority
int getDataPriority(unsigned int can_id, const char* data_type) {
    if (strcmp(data_type, "speed") == 0) {
        if (can_id == 0x5D7) return 1;      // Primary
        if (can_id == 0x19F) return 2;      // Secondary  
        if (can_id == 0x599) return 3;      // Tertiary
    }
    else if (strcmp(data_type, "odometer") == 0) {
        if (can_id == 0x5D7) return 1;      // Primary
        if (can_id == 0x599) return 2;      // Secondary
    }
    return 0; // Unknown
}

// Generate system status summary for V2V/V2G applications
const char* getSystemStatusSummary(ParsedData *data) {
    static char summary[512];
    char temp[128];
    
    strcpy(summary, "=== V2V/V2G System Status ===\n");
    
    // Battery Status (always show if SOC available)
    if (data->SOC > 0) {
        snprintf(temp, sizeof(temp), "Battery: SOC=%d%%", data->SOC);
        strcat(summary, temp);
        
        if (data->batteryVoltage > 0) {
            snprintf(temp, sizeof(temp), ", Voltage=%.1fV", data->batteryVoltage);
            strcat(summary, temp);
        }
        
        if (data->availableEnergy > 0) {
            snprintf(temp, sizeof(temp), ", Energy=%.1fkWh", data->availableEnergy);
            strcat(summary, temp);
        }
        strcat(summary, "\n");
    }
    
    // Current Status (show if current data available)
    if (data->batteryCurrent != 0.0) {
        const char* currentFlow;
        if (data->batteryCurrent > 5.0) {
            currentFlow = "High discharge (vehicle in use)";
        } else if (data->batteryCurrent < -5.0) {
            currentFlow = "Charging";
        } else {
            currentFlow = "Minimal flow (standby)";
        }
        
        snprintf(temp, sizeof(temp), "Current: %.1fA (%s)\n", data->batteryCurrent, currentFlow);
        strcat(summary, temp);
    }
    
    // Vehicle Status (show if any vehicle data available)
    if (data->speedKmh > 0 || strlen(data->gearPosition) > 0 || data->remainingRange > 0) {
        strcat(summary, "Vehicle:");
        
        if (data->speedKmh >= 0) {
            snprintf(temp, sizeof(temp), " Speed=%.1fkm/h", data->speedKmh);
            strcat(summary, temp);
        }
        
        if (strlen(data->gearPosition) > 0) {
            snprintf(temp, sizeof(temp), ", Gear=%s", data->gearPosition);
            strcat(summary, temp);
        }
        
        if (data->remainingRange > 0) {
            snprintf(temp, sizeof(temp), ", Range=%dkm", data->remainingRange);
            strcat(summary, temp);
        }
        strcat(summary, "\n");
    }
    
    // Controller Status (show if any control data available)
    if (data->motorControllerActive || data->acceleratorPercent > 0 || data->brakePressed) {
        const char* vehicleReadiness;
        if (data->motorControllerActive && data->speedKmh > 1.0) {
            vehicleReadiness = "Active and moving";
        } else if (data->motorControllerActive) {
            vehicleReadiness = "Ready but stationary";
        } else {
            vehicleReadiness = "Offline";
        }
        
        snprintf(temp, sizeof(temp), "Controller: %s", vehicleReadiness);
        strcat(summary, temp);
        
        if (data->acceleratorPercent >= 0) {
            snprintf(temp, sizeof(temp), ", Accel=%d%%", data->acceleratorPercent);
            strcat(summary, temp);
        }
        
        snprintf(temp, sizeof(temp), ", Brake=%s", data->brakePressed ? "Active" : "Inactive");
        strcat(summary, temp);
        strcat(summary, "\n");
    }
    
    // Thermal Status (show if any thermal data available)
    if (data->motorTemperature != 0 || data->capacitorVoltage > 0) {
        strcat(summary, "Thermal:");
        
        if (data->motorTemperature != 0) {
            snprintf(temp, sizeof(temp), " Motor=%d°C", data->motorTemperature);
            strcat(summary, temp);
        }
        
        if (data->capacitorVoltage > 0) {
            snprintf(temp, sizeof(temp), ", Capacitor=%.1fV", data->capacitorVoltage);
            strcat(summary, temp);
        }
        strcat(summary, "\n");
    }
    
    return summary;
}
