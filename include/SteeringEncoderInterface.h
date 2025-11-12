#ifndef __UPPERSTEERINGSENSOR_H__
#define __UPPERSTEERINGSENSOR_H__

#include <Arduino.h>

enum class SteeringEncoderStatus_e
{
    STEERING_ENCODER_NOMINAL = 0,
    STEERING_ENCODER_ERROR = 1,
};

struct EncoderErrorFlags_s
{
    bool calibrationTimeout       = false;  // Ring did not make complete during 10 seconds
    bool calibrationParameter     = false;  // Mechanical installation outside tolerance
    bool generalError             = false;  // Position data invalid (bit=0 means error)
    bool generalWarning           = false;  // Position data valid, operating conditons near limits (bit=0 means warning)
    bool counterError             = false;  // Multiturn counter error (bit=1 means error)
    bool speedHigh                = false;  // Speed too high (bit=1 means error)
    bool tempRange                = false;  // Temperature out of range (bit=1 means error)
    bool distFar                  = false;  // Dist b/w readhead and ring too far (bit=1 means error)
    bool distNear                 = false;  // Dist b/w readhead and ring too close (bit=1 means error)
    bool noData                   = false;  // No data received
};

struct SteeringEncoderConversion_s // what's seen in foxglove?
{
    float angle = 0.0f;
    int raw = 0;
    SteeringEncoderStatus_e status = SteeringEncoderStatus_e::STEERING_ENCODER_NOMINAL;
    EncoderErrorFlags_s errors;
};

class SteeringEncoderInterface
{
public:
// Functions
    virtual void init() = 0;
    /// @brief Commands the underlying steering sensor to sample and hold the result
    virtual void sample() = 0;
    /// @brief Calculate steering angle and whether result is in sensor's defined bounds. DOES NOT SAMPLE.
    /// @return Calculated steering angle in degrees, upperSteeringStatus_s
    virtual SteeringEncoderConversion_s convert() = 0;
};


#endif /* __UPPERSTEERINGSENSOR_H__ */
