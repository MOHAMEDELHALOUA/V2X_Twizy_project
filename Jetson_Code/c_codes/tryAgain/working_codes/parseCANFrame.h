// parseCANFrame.h - V3 Enhanced for V2V/V2G Applications
#ifndef PARSECANFRAME_H
#define PARSECANFRAME_H
#include <stdint.h>

typedef struct {
    unsigned int can_id;
    unsigned char dlc;
    unsigned char data[8];
} CANFrame;

typedef struct {
    // PRIORITY 1: Battery Status (Critical for V2V/V2G)
    unsigned short SOC;              // State of Charge (0x155) - Primary data
    float batteryVoltage;            // Battery Voltage (0x425) - V2G essential
    float availableEnergy;           // Available Energy in kWh (0x425) - V2G planning
    float batteryCurrent;            // Current flow (0x155) - Charge/discharge detection
    uint8_t chargingProtocolStatus;  // Charging status (0x425) - V2G coordination
    
    // PRIORITY 2: Vehicle Dynamics (V2V Safety)
    float speedKmh;                  // Primary: 0x5D7, Backup: 0x19F, 0x599
    float odometerKm;                // Primary: 0x5D7, Backup: 0x599
    int remainingRange;              // Range estimation (0x599) - Trip planning
    
    // PRIORITY 3: Vehicle Control Status (V2V Awareness)
    char gearPosition[10];           // Current gear (0x59B) - Vehicle state
    int motorControllerActive;       // Motor/Controller status (0x59B) - Vehicle readiness
    int acceleratorPercent;          // Accelerator position (0x59B) - Driver intent
    int brakePressed;                // Brake status (0x59B) - Safety critical
    int powerTorque;                 // Power/Torque level (0x59B) - Vehicle dynamics
    float capacitorVoltage;          // Capacitor voltage (0x59B) - System health
    
    // PRIORITY 4: Thermal Management (System Health)
    int motorTemperature;            // Motor temp (0x196) - Thermal limits
    float powerRequest;              // Power request (0x196) - Load monitoring
    
    // Data Quality & Source Tracking
    uint32_t parsed_can_id;          // Which CAN ID provided this data
    int valid;                       // Data validity flag
    uint8_t data_source_priority;    // 1=Primary source, 2=Backup source
    uint32_t timestamp;              // When data was parsed (for V2V/V2G timing)
} ParsedData;

// Function prototypes
ParsedData parseCANFrame(CANFrame *frame);
int getDataPriority(unsigned int can_id, const char* data_type);
const char* getSystemStatusSummary(ParsedData *data);

#endif
