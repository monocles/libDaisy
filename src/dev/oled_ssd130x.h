#pragma once
#ifndef SA_OLED_SSD130X_H
#define SA_OLED_SSD130X_H /**< & */

#include "per/i2c.h"
#include "per/spi.h"
#include "per/gpio.h"
#include "sys/system.h"

namespace daisy
{
/**
 * I2C Transport for SSD1306 / SSD1309 OLED display devices
 */
class SSD130xI2CTransport
{
  public:
    struct Config
    {
        I2CHandle::Config i2c_config;
        uint8_t           i2c_address;
        void              Defaults()
        {
            i2c_config.periph = I2CHandle::Config::Peripheral::I2C_1;
            i2c_config.speed  = I2CHandle::Config::Speed::I2C_1MHZ;

            i2c_config.pin_config.scl.Init(Pin::Port::DSY_GPIOB, 8);
            i2c_config.pin_config.sda.Init(Pin::Port::DSY_GPIOB, 9);
            i2c_address = 0x3C;
        }
    };
    void Init(const Config& config)
    {
        i2c_address_ = config.i2c_address;
        i2c_.Init(config.i2c_config);
    };
    void SendCommand(uint8_t cmd)
    {
        uint8_t buf[2] = {0X00, cmd};
        i2c_.TransmitBlocking(i2c_address_, buf, 2, 1000);
    };

    void SendData(uint8_t* buff, size_t size)
    {
        for(size_t i = 0; i < size; i++)
        {
            uint8_t buf[2] = {0X40, buff[i]};
            i2c_.TransmitBlocking(i2c_address_, buf, 2, 1000);
        }
    };

  private:
    daisy::I2CHandle i2c_;
    uint8_t          i2c_address_;
};

/**
 * 4 Wire SPI Transport for SSD1306 / SSD1309 OLED display devices
 */
class SSD130x4WireSpiTransport
{
  public:
    struct Config
    {
        struct
        {
            Pin dc;    /**< & */
            Pin reset; /**< & */
        } pin_config;
        void Defaults()
        {
            pin_config.dc.Init(Pin::Port::DSY_GPIOB, 4);
            pin_config.reset.Init(Pin::Port::DSY_GPIOB, 15);
        }
    };
    void Init(const Config& config)
    {
        // Initialize both GPIO
        GPIO::Config gpio_conf;
        gpio_conf.mode = GPIO::Config::Mode::OUTPUT_PP;
        gpio_conf.pin  = config.pin_config.dc;
        pin_dc_.Init(gpio_conf);

        gpio_conf.pin = config.pin_config.reset;
        pin_reset_.Init(gpio_conf);
        // Initialize SPI
        spi_.Init();

        // Reset and Configure OLED.
        pin_reset_.Write(0);
        System::Delay(10);
        pin_reset_.Write(1);
        System::Delay(10);
    };
    void SendCommand(uint8_t cmd)
    {
        pin_dc_.Write(0);
        spi_.BlockingTransmit(&cmd, 1);
    };

    void SendData(uint8_t* buff, size_t size)
    {
        pin_dc_.Write(1);
        spi_.BlockingTransmit(buff, size);
    };

  private:
    SpiHandle spi_;
    GPIO      pin_reset_;
    GPIO      pin_dc_;
};


/**
 * A driver implementation for the SSD1306/SSD1309
 */
template <size_t width, size_t height, typename Transport>
class SSD130xDriver
{
  public:
    struct Config
    {
        typename Transport::Config transport_config;
    };

    void Init(Config config)
    {
        transport_.Init(config.transport_config);

        // Init routine...

        // Display Off
        transport_.SendCommand(0xaE);
        // Dimension dependent commands...
        switch(height)
        {
            case 16:
                // Display Clock Divide Ratio
                transport_.SendCommand(0xD5);
                transport_.SendCommand(0x60);
                // Multiplex Ratio
                transport_.SendCommand(0xA8);
                transport_.SendCommand(0x0F);
                // COM Pins
                transport_.SendCommand(0xDA);
                transport_.SendCommand(0x02);
                break;
            case 32:
                // Display Clock Divide Ratio
                transport_.SendCommand(0xD5);
                transport_.SendCommand(0x80);
                // Multiplex Ratio
                transport_.SendCommand(0xA8);
                transport_.SendCommand(0x1F);
                // COM Pins
                transport_.SendCommand(0xDA);
                if(width == 64)
                {
                    transport_.SendCommand(0x12);
                }
                else
                {
                    transport_.SendCommand(0x02);
                }

                break;
            case 48:
                // Display Clock Divide Ratio
                transport_.SendCommand(0xD5);
                transport_.SendCommand(0x80);
                // Multiplex Ratio
                transport_.SendCommand(0xA8);
                transport_.SendCommand(0x2F);
                // COM Pins
                transport_.SendCommand(0xDA);
                transport_.SendCommand(0x12);
                break;
            default: // 128
                // Display Clock Divide Ratio
                transport_.SendCommand(0xD5);
                transport_.SendCommand(0x80);
                // Multiplex Ratio
                transport_.SendCommand(0xA8);
                transport_.SendCommand(0x3F);
                // COM Pins
                transport_.SendCommand(0xDA);
                transport_.SendCommand(0x12);
                break;
        }

        // Display Offset
        transport_.SendCommand(0xD3);
        transport_.SendCommand(0x00);
        // Start Line Address
        transport_.SendCommand(0x40);
        // Normal Display
        transport_.SendCommand(0xA6);
        // All On Resume
        transport_.SendCommand(0xA4);
        // Charge Pump
        transport_.SendCommand(0x8D);
        transport_.SendCommand(0x14);
        // Set Segment Remap
        transport_.SendCommand(0xA1);
        // COM Output Scan Direction
        transport_.SendCommand(0xC8);
        // Contrast Control
        transport_.SendCommand(0x81);
        transport_.SendCommand(0x8F);
        // Pre Charge
        transport_.SendCommand(0xD9);
        transport_.SendCommand(0x25);
        // VCOM Detect
        transport_.SendCommand(0xDB);
        transport_.SendCommand(0x34);


        // Display On
        transport_.SendCommand(0xAF); //--turn on oled panel
    };

    size_t Width() { return width; };
    size_t Height() { return height; };
    size_t CurrentX() { return current_x_; };
    size_t CurrentY() { return current_y_; };

    void SetCursor(uint8_t x, uint8_t y)
    {
        current_x_ = x;
        current_y_ = y;
    }

    void DrawPixel(uint_fast8_t x, uint_fast8_t y, bool on)
    {
        if(x >= width || y >= height)
            return;
        if(on)
            buffer_[x + (y / 8) * width] |= (1 << (y % 8));
        else
            buffer_[x + (y / 8) * width] &= ~(1 << (y % 8));
    }

    void Fill(bool on)
    {
        for(size_t i = 0; i < sizeof(buffer_); i++)
        {
            buffer_[i] = on ? 0xff : 0x00;
        }
    };

    /**
     * Update the display 
    */
    void Update()
    {
        uint8_t i;
        uint8_t high_column_addr;
        switch(height)
        {
            case 32: high_column_addr = 0x12; break;

            default: high_column_addr = 0x10; break;
        }
        for(i = 0; i < (height / 8); i++)
        {
            transport_.SendCommand(0xB0 + i);
            transport_.SendCommand(0x00);
            transport_.SendCommand(high_column_addr);
            transport_.SendData(&buffer_[width * i], width);
        }
    };

  private:
    uint16_t  current_x_;
    uint16_t  current_y_;
    Transport transport_;
    uint8_t   buffer_[width * height / 8];
};

/**
 * A driver for the SSD1306/SSD1309 128x64 OLED displays connected via 4 wire SPI  
 */
using SSD130x4WireSpi128x64Driver
    = daisy::SSD130xDriver<128, 64, SSD130x4WireSpiTransport>;

/**
 * A driver for the SSD1306/SSD1309 128x32 OLED displays connected via 4 wire SPI  
 */
using SSD130x4WireSpi128x32Driver
    = daisy::SSD130xDriver<128, 32, SSD130x4WireSpiTransport>;

/**
 * A driver for the SSD1306/SSD1309 98x16 OLED displays connected via 4 wire SPI  
 */
using SSD130x4WireSpi98x16Driver
    = daisy::SSD130xDriver<98, 16, SSD130x4WireSpiTransport>;

/**
 * A driver for the SSD1306/SSD1309 64x48 OLED displays connected via 4 wire SPI  
 */
using SSD130x4WireSpi64x48Driver
    = daisy::SSD130xDriver<64, 48, SSD130x4WireSpiTransport>;

/**
 * A driver for the SSD1306/SSD1309 64x32 OLED displays connected via 4 wire SPI  
 */
using SSD130x4WireSpi64x32Driver
    = daisy::SSD130xDriver<64, 32, SSD130x4WireSpiTransport>;

/**
 * A driver for the SSD1306/SSD1309 128x64 OLED displays connected via I2C  
 */
using SSD130xI2c128x64Driver
    = daisy::SSD130xDriver<128, 64, SSD130xI2CTransport>;

/**
 * A driver for the SSD1306/SSD1309 128x32 OLED displays connected via I2C  
 */
using SSD130xI2c128x32Driver
    = daisy::SSD130xDriver<128, 32, SSD130xI2CTransport>;

/**
 * A driver for the SSD1306/SSD1309 98x16 OLED displays connected via I2C  
 */
using SSD130xI2c98x16Driver = daisy::SSD130xDriver<98, 16, SSD130xI2CTransport>;

/**
 * A driver for the SSD1306/SSD1309 64x48 OLED displays connected via I2C  
 */
using SSD130xI2c64x48Driver = daisy::SSD130xDriver<64, 48, SSD130xI2CTransport>;

/**
 * A driver for the SSD1306/SSD1309 64x32 OLED displays connected via I2C  
 */
using SSD130xI2c64x32Driver = daisy::SSD130xDriver<64, 32, SSD130xI2CTransport>;

}; // namespace daisy


#endif