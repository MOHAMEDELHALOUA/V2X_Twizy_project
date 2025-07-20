#ifndef PARSECANFRAME_H
#define PARSECANFRAME_H

#include <stdint.h>

typedef struct {
    unsigned int can_id;
    unsigned char dlc;
    unsigned char data[8];
} CANFrame;

typedef struct {
    unsigned short SOC;
    float speedKmh;
    float odometerKm;
    float displaySpeed;
    float batteryCurrent;
    int motorTemperature;
    int batterySOH;
    float batteryVoltage;
    int remainingRange;
    char gearPosition[10];
    int acceleratorPercent;
    uint8_t MacAddress[6];
} ParsedData;

ParsedData parseCANFrame(CANFrame *frame);

#endif
