#ifndef ORBIS_BR
#define ORBIS_BR

#include <Arduino.h>
#include "SteeringEncoderInterface.h"

// Basic Commands
//  Position offset setting 'Z' (0x5A)               4
//  Multiturn counter setting 'M' (0x4D)             4
//  Baud rate setting 'B' (0x42)                     4
//  Continuous-response setting 'T' (0x54)           4
//  Continuous-response start 'S' (0x53)             0
//  Continuous-response stop 'P' (0x50)              0
//  Configuration parameters save 'c' (0x63)         0
//  Configuration parameters reset 'r' (0x72)        0

// Definitions
const bool     ORBIS_BR_FLIP_DIRECTION                    = true;
const int      ORBIS_BR_DEFAULT_BAUD_RATE                 = 115200;
const float    ORBIS_BR_SCALE                             = 0.0219726562; // 360 degrees per 14 bits (14 bit resolution)

// Error Definitions
const uint16_t ORBIS_BR_BITMASK_GENERAL_WARNING           = (0b1 << 0);   // error if low, position valid, some operating conditions are close to limits : 0b00000001
const uint16_t ORBIS_BR_BITMASK_GENERAL_ERROR             = (0b1 << 1);   // warning if low, position invalid : 0b00000010

const uint16_t ORBIS_BR_BITMASK_DETAILED_COUNTER_ERROR    = (0b1 << 3); // 0b00001000, errors if high
const uint16_t ORBIS_BR_BITMASK_DETAILED_SPEED_HIGH       = (0b1 << 4); // 0b00010000, errors if high
const uint16_t ORBIS_BR_BITMASK_DETAILED_TEMP_RANGE       = (0b1 << 5); // 0b00100000, errors if high
const uint16_t ORBIS_BR_BITMASK_DETAILED_DIST_FAR         = (0b1 << 6); // 0b01000000, errors if high
const uint16_t ORBIS_BR_BITMASK_DETAILED_DIST_NEAR        = (0b1 << 7); // 0b10000000, errors if high

class OrbisBR : public SteeringEncoderInterface
{
public:
// Constructors
    OrbisBR(HardwareSerial* serial, int serialSpeed);
// Functions
    void init() override;
    void sample() override;
    SteeringEncoderConversion_s convert() override;
    void setOffset(float newOffset) override;
    void timedSample(uint32_t intervalMs = 500);

private:
// Data
    HardwareSerial* _serial;
    int _serialSpeed;
    int _position_data;
    float _offset;
    SteeringEncoderConversion_s _lastConversion;
    unsigned long _lastSampleTime = 0;

    void decodeErrors(uint8_t general, uint8_t detailed);
};

#endif /* ORBIS_BR */