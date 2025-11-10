#include "ORBIS_BR.h"

OrbisBR::OrbisBR(HardwareSerial* serial, int serialSpeed = ORBIS_BR_DEFAULT_BAUD_RATE)
: _serial(serial)
, _serialSpeed(serialSpeed)
, _position_data(0)
, _offset(0.0f)
{
   _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
}

void OrbisBR::init() // all initialization (calibration and configuration)
{
    _serial->begin(_serialSpeed);

    // Unlock encoder sequence
    _serial->write(0xCD); 
    _serial->write(0xEF);
    _serial->write(0x89);
    _serial->write(0xAB); // may need delay(1)

    _serial->write(0x41);             // self-calibration start
    _serial->write(0x69); delay(5);   // self-calibration status request

    SteeringEncoderConversion_s calibStatus;
    
    if (_serial->available() >= 2)   // Calibration checker loop
    {
        uint8_t echo = _serial->read();     // echo byte
        uint8_t status = _serial->read();   // status byte

        calibStatus.errors.calibrationTimeout   = status & 0b00000010;
        calibStatus.errors.calibrationParameter = status & 0b00000100;
    } 
    else
    {
        calibStatus.errors.noData = true;
        // no response
    }

    _lastConversion = calibStatus;

    //Continous-Response Setting ('T')
    // Using auto-start, and short position request, period = 1000 µs, should have delay(1)
    _serial->write(0x54);       // 'T' command
    _serial->write(0x01);       // Auto-start enabled after power-on
    _serial->write(0x33);       // Command '3' = short position response
    _serial->write(0x00);       // Period high byte (0x00)
    _serial->write(0x3E8);      // Period low byte (0x3E8 = 1000 µs)


    //Continous-Response Start ('S')
    _serial->write(0x53); delay(1); 


    //Save Config ('c')
    _serial->write(0x63); 

    //encoder should auto-start on future power-ups
}


void OrbisBR::sample()
{
    _lastConversion = SteeringEncoderConversion_s();
    _serial->write(0x64); // position request + detailed status: 1 byte echo, 2 byte position, 1 byte detailed status
    
    if (_serial->available() < 4) // check if received all 4 bytes
    { 
        calibStatus.errors.noData = true;
        _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
        return;
    }   

    uint8_t echo = _serial->read();
    uint8_t general1 = _serial->read();
    uint8_t general2 = _serial->read();
    uint8_t detailed = _serial->read();

    if (echo != 0x64)
    {
        calibStatus.errors.noData = true;
        _lastConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
        return;
    }
    
    uint16_t raw_position_data = ((uint16_t) general2 << 8) | (uint16_t) general1;
    _position_data = raw_position_data >> 2;
        
    // Errors
    bool anyError = 
    (
        _lastConversion.errors.generalError   ||
        _lastConversion.errors.counterError   ||
        _lastConversion.errors.speedHigh      ||
        _lastConversion.errors.tempRange      ||
        _lastConversion.errors.distFar        ||
        _lastConversion.errors.distNear       ||
        _lastConversion.errors.noData
    );
    
    _lastConversion.status = anyError
        ? SteeringEncoderStatus_e::STEERING_ENCODER_ERROR
        : SteeringEncoderStatus_e::STEERING_ENCODER_NOMINAL;
}

void OrbisBR::decodeErrors(uint8_t general, uint8_t detailed)
{
    // General bits error low (0)
    _lastConversion.errors.generalError     = !(general & ORBIS_BR_BITMASK_GENERAL_ERROR);
    _lastConversion.errors.generalWarning   = !(general & ORBIS_BR_BITMASK_GENERAL_WARNING);

    // Detailed bits are error high (1)
    _lastConversion.errors.counterError     = detailed & ORBIS_BR_BITMASK_DETAILED_COUNTER_ERROR;
    _lastConversion.errors.speedHigh        = detailed & ORBIS_BR_BITMASK_DETAILED_SPEED_HIGH;
    _lastConversion.errors.tempRange        = detailed & ORBIS_BR_BITMASK_DETAILED_TEMP_RANGE;
    _lastConversion.errors.distFar          = detailed & ORBIS_BR_BITMASK_DETAILED_DIST_FAR;
    _lastConversion.errors.distNear         = detailed & ORBIS_BR_BITMASK_DETAILED_DIST_NEAR;
}


void OrbisBR::timedSample(uint32_t intervalMs)
{
    if (millis() - _lastSampleTime >= intervalMs)
    {
        _lastSampleTime = millis();
        sample();   // call your existing sampling logic
    }
}


SteeringEncoderConversion_s OrbisBR::convert() // convert raw binary code, to a this struct. Readable for the car. 
{
    _lastConversion_.raw = _position_data;
    _lastConversion_.angle = (_position_data * ORBIS_BR_SCALE) + _offset;

    // Apply modulo
    while (_lastConversion.angle < -180.0f)
        _lastConversion.angle += 360.0f;

    while (_lastConversion.angle > 180.0f)
        _lastConversion.angle -= 360.0f;

    // Apply wraparound fix
    if (ORBIS_BR_FLIP_DIRECTION)
        _lastConversion.angle *= -1.0f;

    return _lastConversion;
}

void OrbisBR::setOffset(float newOffset)
{
    _offset = newOffset;
}
