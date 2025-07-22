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
    
    // Parsing metadata
    uint32_t parsed_can_id;
    int valid;
    uint8_t MacAddress[6];
} ParsedData;

ParsedData parseCANFrame(CANFrame *frame);

#endif
