#include "ORBIS_BR10.h"

OrbisBR10::OrbisBR10(HardwareSerial* serial, int serialSpeed)
: serial_(serial)
, serialSpeed_(serialSpeed)
{
    data_ = 0;
    offset_ = 0.0;
    // Initialize with all errors/warnings set
    status_ = ORBIS_BR10_BITMASK_GENERAL | ORBIS_BR10_BITMASK_DETAILED;    
}

OrbisBR10::OrbisBR10(HardwareSerial* serial)
: OrbisBR10(serial, ORBIS_BR10_DEFAULT_SERIAL_SPEED)
{

}

uint8_t OrbisBR10::check_calibration_status() {
    uint8_t status_flags = 0;
    serial_->write(0x69);
    
    if (serial_->available() && serial_->read() == 0x69) {
        status_flags = serial_->read();
    } else {
        return -1;
    }

    
    return status_flags;

}

//Unlock sequence for self-calibration, must be followed by programming command byte
void OrbisBR10::send_command_sequence_() {
    // delay(1);
    serial_->write(0xCD);
    // delay(1);
    serial_->write(0xEF);
    // delay(1);
    serial_->write(0x89);
    // delay(1);
    serial_->write(0xAB);
    // delay(1);
    
}

void OrbisBR10::set_offset_(uint32_t pos)
{
    // Serial.println(pos);
    send_command_sequence_();
    //0x5A is programming command byte for position offset setting
    serial_->write(0x5A);
    // delay(1);
    serial_->write((pos & 0xFF000000) >> 24);
    //serial_->write(0);
    // delay(1);
    serial_->write((pos & 0xFF0000) >> 16);
    //serial_->write(0);
    // delay(1);
    serial_->write((pos & 0xFF00) >> 8);
    //serial_->write(0x2D);
    // delay(1);
    serial_->write(pos & 0xFF);
    //serial_->write(0x12);
    // delay(1);
}

void OrbisBR10::save_parameters_() {
    send_command_sequence_();
    //0x63 programming command byte for configuration parameters save
    serial_->write(0x63);
}

void OrbisBR10::calibrate_sensor()
{

    // set_zero_position(0)
    set_offset_(6200);
    // delay(200);
    // sets data_ with the raw current position
    // sample();
    // Serial.println(data_);
    // set_offset_(data_);
    save_parameters_();

    // auto status_flags = check_calibration_status();
    // if (status_flags & 0b01000000) {
    //     Serial.println("already calibrated");
        
    // }
    // if (status_flags & 0b00001000) {
    //     Serial.println("bad tolerance");
        
    // }
    // if (status_flags & 0b0001100) {
    //     Serial.println("errors");
    // }

    // save_parameters_();
}

void OrbisBR10::init()
{
    serial_->begin(serialSpeed_);
}

void OrbisBR10::sample()
{
    uint8_t readByte[3] = {0};

    serial_->write(0x64);
    if (serial_->available() >= 4 && serial_->read() == 0x64) {
        readByte[0] = serial_->read(); //lower half of steering
        readByte[1] = serial_->read(); //upper half of steering
        readByte[2] = serial_->read(); //error stream 
        data_ = (int16_t)(((uint16_t)readByte[0]) << 6) | ((uint16_t)readByte[1] >> 2);
        status_ = ~readByte[1] & ORBIS_BR10_BITMASK_GENERAL;
        status_ |= readByte[2] & ORBIS_BR10_BITMASK_DETAILED;
    }
    else
    {
        // set the error flag
        status_ = ORBIS_BR10_BITMASK_GENERAL_ERROR;
    }
}

SteeringEncoderConversion_s OrbisBR10::convert()
{
    SteeringEncoderConversion_s returnConversion;
    returnConversion.raw = data_;
    returnConversion.angle = (data_ * ORBIS_BR10_SCALE) + offset_;

    // Apply modulo
    while (returnConversion.angle < 0.0f)
        returnConversion.angle += 360.0f;

    while (returnConversion.angle > 360.0f)
        returnConversion.angle -= 360.0f;

    // Apply wraparound fix
    if (returnConversion.angle > 180.0f)
        returnConversion.angle = returnConversion.angle - 360.0f;
    
    if (ORBIS_BR10_FLIP_DIRECTION)
        returnConversion.angle *= -1.0f;

    if (status_ & (ORBIS_BR10_BITMASK_GENERAL_ERROR | ORBIS_BR10_BITMASK_DETAILED_COUNTER_ERROR))
    {
        returnConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_ERROR;
    }
    else if (status_ == 0)
    {
        returnConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_NOMINAL;
    }
    else
    {
        returnConversion.status = SteeringEncoderStatus_e::STEERING_ENCODER_MARGINAL;
    }
    return returnConversion;
}

void OrbisBR10::setOffset(float newOffset)
{
    offset_ = newOffset;
}
