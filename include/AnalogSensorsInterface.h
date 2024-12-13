#ifndef ANALOGSENSORSINTERFACE
#define ANALOGSENSORSINTERFACE

#include <tuple>
#include <algorithm>
#include "SharedFirmwareTypes.h"

/**
 * The AnalogChannel class represents one individual "channel" of an ADC. Each Channel
 * has its own scale, offsets, and clamp. An AnalogMultiSensor has several AnalogChannels.
 * To use an AnalogChannel properly, you need to first sample the data and place the raw
 * conversion into lastSample. Then, you can call convert() to retrieve the data.
 */
class AnalogChannel
{
public:
    /* Data */
    float scale;
    float offset;
    bool clamp;
    float clampLow;
    float clampHigh;
    int lastSample;

    /* Constructors */
    AnalogChannel(float scale_, float offset_, bool clamp_, float clampLow_, float clampHigh_)
    : scale(scale_),
      offset(offset_),
      clamp(clamp_),
      clampLow(clampLow_),
      clampHigh(clampHigh_) {}
    AnalogChannel(float scale_, float offset_)
    : AnalogChannel(scale_, offset_, false, __FLT_MIN__, __FLT_MAX__) {}
    AnalogChannel()
    : AnalogChannel(1.0, 0.0, false, __FLT_MIN__, __FLT_MAX__) {}
    
    /* Functions*/
    /**
     * Calculates sensor output and whether result is in the sensor's defined bounds. This DOES NOT SAMPLE.
     * @return This AnalogChannel's output AnalogConversion_s (raw reading, real value, and status).
     */
    AnalogConversion_s convert()
    {
        float conversion = (lastSample + offset) * scale;
        float clampedConversion = std::min(std::max(conversion, clampLow), clampHigh);
        AnalogSensorStatus_e returnStatus = AnalogSensorStatus_e::ANALOG_SENSOR_GOOD;
        if (clamp && (conversion > clampHigh || conversion < clampLow))
            returnStatus = AnalogSensorStatus_e::ANALOG_SENSOR_CLAMPED;
        return {
            lastSample,
            clamp ? clampedConversion : conversion,
            returnStatus
        };
    }
};

/**
 * The AnalogMultiSensor is the base class for specific analog sensor interfaces. Each specific
 * analog sensor (the MCP, Teensy, etc) are children of this class. To use this class, you need
 * to call sample() and convert().
 */
template <int N>
class AnalogMultiSensor
{
private:
protected:
    AnalogChannel channels_[N];
public:
    AnalogConversionPacket_s<N> data;

    /* Functions */

    /**
     * The tick() function in each subclass should call sample() and then convert() to update the data field.
     */
    void tick(unsigned long curr_millis);

    /**
     * Used by systems to retrieve the data field.
     */
    const AnalogConversionPacket_s<N>& get()
    {
        return data;
    }

    /**
     * Sets the scale of the internal conversion channel.
     */
    void setChannelScale(int channel, float scale)
    {
        if (channel < N)
            channels_[channel].scale = scale;
    }

    /**
     * Sets the offset of the internal conversion channel.
     */
    void setChannelOffset(int channel, float offset)
    {
        if (channel < N)
            channels_[channel].offset = offset;
    }

    /**
     * Sets the clamp of the internal conversion channel. The min and max values (to clamp to)
     * are in actual (post-conversion) units. Calling this function will also set the channel's
     * clamp boolean to true to enable clamping. There is no way to programmatically disable clamping.
     */
    void setChannelClamp(int channel, float clampLow, float clampHigh)
    {
        if (channel < N) {
            channels_[channel].clampLow = clampLow;
            channels_[channel].clampHigh = clampHigh;
            channels_[channel].clamp = true;
        }
    }

    /**
     * Sets both the scale and offset of the given channel by calling setChannelOffset and setChannelClamp.
     */
    void setChannelScaleAndOffset(int channel, float scale, float offset)
    {
        setChannelScale(channel, scale);
        setChannelOffset(channel, offset);
    }

    /**
     * Performs unit conversions on all channels.
     * @post The data field will be updated with the new conversions of each channel.
     */
    void convert()
    {
        for (int i = 0; i < N; i++)
        {
            data.conversions[i] = channels_[i].convert();
        }
    }

    /**
     * Commands the underlying device to sample all channels and internall store the results.
     * @post The lastSample field of each AnalogChannel in channels_ must contain the new value.
     */
    void sample();

};

#endif /* ANALOGSENSORSINTERFACE */
