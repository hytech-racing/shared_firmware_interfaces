#include "MCP_ADC.h"
#include <SPI.h>
#include <array>

template <int MCP_ADC_NUM_CHANNELS>
MCP_ADC<MCP_ADC_NUM_CHANNELS>::MCP_ADC(const int spiPinCS, const int spiPinSDI, const int spiPinSDO, const int spiPinCLK, const int spiSpeed, const float scales[MCP_ADC_NUM_CHANNELS], const float offsets[MCP_ADC_NUM_CHANNELS])
: _spiPinCS(spiPinCS)
, _spiPinSDI(spiPinSDI)
, _spiPinSDO(spiPinSDO)
, _spiPinCLK(spiPinCLK)
, _spiSpeed(spiSpeed)
, _currentChannel(0)
{
    for (int i = 0; i < MCP_ADC_NUM_CHANNELS; i++)
    {
        MCP_ADC<MCP_ADC_NUM_CHANNELS>::_channels[i] = AnalogChannel();
        this->setChannelScaleAndOffset(i, scales[i], offsets[i]);
    }
}

template <int MCP_ADC_NUM_CHANNELS>
void MCP_ADC<MCP_ADC_NUM_CHANNELS>::init()
{
    pinMode(_spiPinCS, OUTPUT);
    digitalWrite(_spiPinCS, HIGH);

    _dma_busy = false;

    _spi_event.setContext(this);
    _spi_event.attachImmediate([](EventResponderRef ref)
    {
        static_cast<MCP_ADC*>(ref.getContext())->_dma_callback();
    });

    _rx_buf.fill(0);
    _tx_buf.fill(0);
}

template <int MCP_ADC_NUM_CHANNELS>
void MCP_ADC<MCP_ADC_NUM_CHANNELS>::tick()
{
    if (_dma_busy)
    {
        return;
    }
    _sample();
}

template <int MCP_ADC_NUM_CHANNELS>
void MCP_ADC<MCP_ADC_NUM_CHANNELS>::_dma_callback()
{
    digitalWrite(_spiPinCS, HIGH);
    SPI.endTransaction();

    uint8_t b0 = _rx_buf[0];
    uint8_t b1 = _rx_buf[1];
    uint8_t b2 = _rx_buf[2];
    uint16_t value = (b0 & 0x01) << 11 | (b1 & 0xFF) << 3 | (b2 & 0xE0) >> 5;

    MCP_ADC<MCP_ADC_NUM_CHANNELS>::_channels[_currentChannel].lastSample = (value & 0x0FFF);

    _currentChannel = (_currentChannel+1) % MCP_ADC_NUM_CHANNELS;

    _dma_busy = false;

    this->_convert();
}

template <int MCP_ADC_NUM_CHANNELS>
void MCP_ADC<MCP_ADC_NUM_CHANNELS>::_sample()
{
    // uint16_t command = (
    //     (0b1 << 15) |    // start bit
    //     (0b1 << 14)      // single ended mode
    // );
    byte command;

    command = ((0x01 << 7) |                    // start bit
            (0x01 << 6) |                    // single or differential
            ((_currentChannel & 0x07) << 3));   // channel number
    // initialize SPI bus. REQUIRED: call SPI.begin() before this
    SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));

    digitalWrite(_spiPinCS, LOW);
    
    _tx_buf[0] = command;
    SPI.transfer(_tx_buf.data(), _rx_buf.data(), buffer_size, _spi_event);

    _dma_busy = true;
}