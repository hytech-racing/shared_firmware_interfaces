#ifndef ANALOGSENSORSINTERFACE
#define ANALOGSENSORSINTERFACE

#include <tuple>
#include <algorithm>

enum class AnalogSensorStatus_e
{
    ANALOG_SENSOR_GOOD = 0,
    ANALOG_SENSOR_CLAMPED = 1,
};

struct AnalogConversion_s
{
    int raw;
    float conversion;
    AnalogSensorStatus_e status;
};

template <int N>
struct AnalogConversionPacket_s
{
    AnalogConversion_s conversions[N];
};

class AnalogChannel
{
public:
// Data
    float scale;
    float offset;
    bool clamp;
    float clampLow;
    float clampHigh;
    int lastSample;

// Constructors
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
    
// Functions
    /// @brief Calculate sensor output and whether result is in sensor's defined bounds. DOES NOT SAMPLE.
    /// @return Sensor's calculated output in real units, whether the result was clamped (AnalogSensorStatus_s)
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

template <int N>
class AnalogMultiSensor
{
private:
protected:
    AnalogChannel channels_[N];
public:
    AnalogConversionPacket_s<N> data;
// Functions
    /// @brief Called by the main loop. Allows AnalogMultiSensor devices not owned by a single system to self-actualize sampling & conversion.
    /// @param tick 
    void tick(unsigned long curr_millis);

    /// @brief Used by systems to get data out of this device when it's self-actualizing sampling & conversion.
    /// @return Const ref to last data conversion.
    const AnalogConversionPacket_s<N>& get()
    {
        return data;
    }

    /// @brief Set the scale of an internal conversion channel
    void setChannelScale(int channel, float scale)
    {
        if (channel < N)
            channels_[channel].scale = scale;
    }

    /// @brief Set the offset of an internal conversion channel
    void setChannelOffset(int channel, float offset)
    {
        if (channel < N)
            channels_[channel].offset = offset;
    }

    void setChannelClamp(int channel, float clampLow, float clampHigh)
    {
        if (channel < N) {
            channels_[channel].clampLow = clampLow;
            channels_[channel].clampHigh = clampHigh;
            channels_[channel].clamp = true;
        }
    }

    /// @brief Performs unit conversions on all channels
    void convert()
    {
        for (int i = 0; i < N; i++)
        {
            data.conversions[i] = channels_[i].convert();
        }
    }

    /// @brief Commands the underlying device to sample all channels and internally store the results
    void sample();
};

#endif /* ANALOGSENSORSINTERFACE */
