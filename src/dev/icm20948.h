#pragma once
#ifndef DSY_ICM20948_H
#define DSY_ICM20948_H

// Misc configuration macros
#define I2C_MASTER_RESETS_BEFORE_FAIL \
    5 ///< The number of times to try resetting a stuck I2C master before giving up
#define NUM_FINISHED_CHECKS \
    100 ///< How many times to poll I2C_SLV4_DONE before giving up and resetting

// Bank 0
#define ICM20X_B0_WHOAMI 0x00         ///< Chip ID register
#define ICM20X_B0_USER_CTRL 0x03      ///< User Control Reg. Includes I2C Master
#define ICM20X_B0_LP_CONFIG 0x05      ///< Low Power config
#define ICM20X_B0_REG_INT_PIN_CFG 0xF ///< Interrupt config register
#define ICM20X_B0_REG_INT_ENABLE 0x10 ///< Interrupt enable register 0
#define ICM20X_B0_REG_INT_ENABLE_1 0x11 ///< Interrupt enable register 1
#define ICM20X_B0_I2C_MST_STATUS \
    0x17 ///< Records if I2C master bus data is finished
#define ICM20X_B0_REG_BANK_SEL 0x7F ///< register bank selection register
#define ICM20X_B0_PWR_MGMT_1 0x06   ///< primary power management register
#define ICM20X_B0_ACCEL_XOUT_H 0x2D ///< first byte of accel data
#define ICM20X_B0_GYRO_XOUT_H 0x33  ///< first byte of accel data

// Bank 2
#define ICM20X_B2_GYRO_SMPLRT_DIV 0x00    ///< Gyroscope data rate divisor
#define ICM20X_B2_GYRO_CONFIG_1 0x01      ///< Gyro config for range setting
#define ICM20X_B2_ACCEL_SMPLRT_DIV_1 0x10 ///< Accel data rate divisor MSByte
#define ICM20X_B2_ACCEL_SMPLRT_DIV_2 0x11 ///< Accel data rate divisor LSByte
#define ICM20X_B2_ACCEL_CONFIG_1 0x14     ///< Accel config for setting range

// Bank 3
#define ICM20X_B3_I2C_MST_ODR_CONFIG 0x0 ///< Sets ODR for I2C master bus
#define ICM20X_B3_I2C_MST_CTRL 0x1       ///< I2C master bus config
#define ICM20X_B3_I2C_MST_DELAY_CTRL 0x2 ///< I2C master bus config
#define ICM20X_B3_I2C_SLV0_ADDR \
    0x3 ///< Sets I2C address for I2C master bus slave 0
#define ICM20X_B3_I2C_SLV0_REG \
    0x4 ///< Sets register address for I2C master bus slave 0
#define ICM20X_B3_I2C_SLV0_CTRL 0x5 ///< Controls for I2C master bus slave 0
#define ICM20X_B3_I2C_SLV0_DO 0x6   ///< Sets I2C master bus slave 0 data out

#define ICM20X_B3_I2C_SLV4_ADDR \
    0x13 ///< Sets I2C address for I2C master bus slave 4
#define ICM20X_B3_I2C_SLV4_REG \
    0x14 ///< Sets register address for I2C master bus slave 4
#define ICM20X_B3_I2C_SLV4_CTRL 0x15 ///< Controls for I2C master bus slave 4
#define ICM20X_B3_I2C_SLV4_DO 0x16   ///< Sets I2C master bus slave 4 data out
#define ICM20X_B3_I2C_SLV4_DI 0x17   ///< Sets I2C master bus slave 4 data in

#define ICM20948_CHIP_ID 0xEA ///< ICM20948 default device id from WHOAMI

#define ICM20948_I2CADDR_DEFAULT 0x69 ///< ICM20948 default i2c address
#define ICM20948_MAG_ID 0x09          ///< The chip ID for the magnetometer

#define ICM20948_UT_PER_LSB 0.15 ///< mag data LSB value (fixed)

#define AK09916_WIA2 0x01  ///< Magnetometer
#define AK09916_ST1 0x10   ///< Magnetometer
#define AK09916_HXL 0x11   ///< Magnetometer
#define AK09916_HXH 0x12   ///< Magnetometer
#define AK09916_HYL 0x13   ///< Magnetometer
#define AK09916_HYH 0x14   ///< Magnetometer
#define AK09916_HZL 0x15   ///< Magnetometer
#define AK09916_HZH 0x16   ///< Magnetometer
#define AK09916_ST2 0x18   ///< Magnetometer
#define AK09916_CNTL2 0x31 ///< Magnetometer
#define AK09916_CNTL3 0x32 ///< Magnetometer

namespace daisy
{
/** @addtogroup external 
    @{ 
*/

/** I2C Transport for Icm20948 */
class Icm20948I2CTransport
{
  public:
    Icm20948I2CTransport() {}
    ~Icm20948I2CTransport() {}

    struct Config
    {
        I2CHandle::Config::Peripheral periph;
        I2CHandle::Config::Speed      speed;
        Pin                           scl;
        Pin                           sda;

        uint8_t address;

        Config()
        {
            address = ;

            periph = I2CHandle::Config::Peripheral::I2C_1;
            speed  = I2CHandle::Config::Speed::I2C_400KHZ;

            scl = Pin(PORTB, 8);
            sda = Pin(PORTB, 9);
        }
    };

    inline void Init(Config config)
    {
        config_ = config;

        I2CHandle::Config i2c_config;
        i2c_config.mode   = I2CHandle::Config::Mode::I2C_MASTER;
        i2c_config.periph = config.periph;
        i2c_config.speed  = config.speed;

        i2c_config.pin_config.scl = config.scl;
        i2c_config.pin_config.sda = config.sda;

        i2c_.Init(i2c_config);
    }

    void Write(uint8_t *data, uint16_t size)
    {
        error_ |= I2CHandle::Result::OK
                  != i2c_.TransmitBlocking(config_.address, data, size, 10);
    }

    void Read(uint8_t *data, uint16_t size)
    {
        error_ |= I2CHandle::Result::OK
                  != i2c_.ReceiveBlocking(config_.address, data, size, 10);
    }

    /**  Writes an 8 bit value
        \param reg the register address to write to
        \param value the value to write to the register
    */
    void Write8(uint8_t reg, uint8_t value)
    {
        uint8_t buffer[2];

        buffer[0] = reg;
        buffer[1] = value;

        Write(buffer, 2);
    }

    /** Read from a reg address a defined number of bytes */
    void ReadReg(uint8_t reg, uint8_t *buff, uint8_t size)
    {
        Write(&reg, 1);
        Read(buff, size);
    }

    /**  Reads an 8 bit value
        \param reg the register address to read from
        \return the 16 bit data value read from the device
    */
    uint8_t Read8(uint8_t reg)
    {
        uint8_t buffer;
        ReadReg(reg, &buffer, 1);
        return buffer;
    }

    bool GetError()
    {
        bool tmp = error_;
        error_   = false;
        return tmp;
    }

  private:
    I2CHandle i2c_;
    Config    config_;

    // true if error has occured since last check
    bool error_;
};

/** SPI Transport for Icm20948 */
class Icm20948SpiTransport
{
  public:
    Icm20948SpiTransport() {}
    ~Icm20948SpiTransport() {}

    struct Config
    {
        SpiHandle::Config::Peripheral periph;
        Pin                           sclk;
        Pin                           miso;
        Pin                           mosi;
        Pin                           nss;

        Config()
        {
            periph = SpiHandle::Config::Peripheral::SPI_1;
            sclk   = Pin(PORTG, 11);
            miso   = Pin(PORTB, 4);
            mosi   = Pin(PORTB, 5);
            nss    = Pin(PORTG, 10);
        }
    };

    inline void Init(Config config)
    {
        SpiHandle::Config spi_conf;
        spi_conf.mode           = SpiHandle::Config::Mode::MASTER;
        spi_conf.direction      = SpiHandle::Config::Direction::TWO_LINES;
        spi_conf.clock_polarity = SpiHandle::Config::ClockPolarity::LOW;
        spi_conf.clock_phase    = SpiHandle::Config::ClockPhase::ONE_EDGE;
        spi_conf.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_2;
        spi_conf.nss            = SpiHandle::Config::NSS::SOFT;

        spi_conf.periph          = config.periph;
        spi_conf.pin_config.sclk = config.sclk;
        spi_conf.pin_config.miso = config.miso;
        spi_conf.pin_config.mosi = config.mosi;
        spi_conf.pin_config.nss  = config.nss;

        spi_.Init(spi_conf);
    }

    void Write(uint8_t *data, uint16_t size)
    {
        error_ |= SpiHandle::Result::OK != spi_.BlockingTransmit(data, size);
    }

    void Read(uint8_t *data, uint16_t size)
    {
        error_ |= SpiHandle::Result::OK != spi_.BlockingReceive(data, size, 10);
    }

    /**  Writes an 8 bit value
        \param reg the register address to write to
        \param value the value to write to the register
    */
    void Write8(uint8_t reg, uint8_t value)
    {
        uint8_t buffer[2];

        buffer[0] = reg & ~0x80;
        buffer[1] = value;

        Write(buffer, 2);
    }

    /** Read from a reg address a defined number of bytes */
    void ReadReg(uint8_t reg, uint8_t *buff, uint8_t size)
    {
        reg = uint8_t(reg | 0x80);
        Write(&reg, 1);
        Read(buff, size);
    }


    /**  Reads an 8 bit value
        \param reg the register address to read from
        \return the data uint8_t read from the device
    */
    uint8_t Read8(uint8_t reg)
    {
        uint8_t buffer;
        ReadReg(reg, &buffer, 1);
        return buffer;
    }

    bool GetError()
    {
        bool tmp = error_;
        error_   = false;
        return tmp;
    }

  private:
    SpiHandle spi_;
    bool      error_;
};

/** \brief Device support for ICM20948 TOF sensor
    @author beserge
    @date December 2021
*/
template <typename Transport>
class Icm20948
{
  public:
    Icm20948() {}
    ~Icm20948() {}

    struct Config
    {
        typename Transport::Config transport_config;

        Config() {}
    };

    /** The accelerometer data range */
    enum icm20948_accel_range_t
    {
        ICM20948_ACCEL_RANGE_2_G,
        ICM20948_ACCEL_RANGE_4_G,
        ICM20948_ACCEL_RANGE_8_G,
        ICM20948_ACCEL_RANGE_16_G,
    };

    /** The gyro data range */
    enum icm20948_gyro_range_t
    {
        ICM20948_GYRO_RANGE_250_DPS,
        ICM20948_GYRO_RANGE_500_DPS,
        ICM20948_GYRO_RANGE_1000_DPS,
        ICM20948_GYRO_RANGE_2000_DPS,
    };

    /** Data rates/modes for the embedded AsahiKASEI AK09916 3-axis magnetometer */
    enum ak09916_data_rate_t
    {
        AK09916_MAG_DATARATE_SHUTDOWN = 0x0, ///< Stops measurement updates
        AK09916_MAG_DATARATE_SINGLE
        = 0x1, ///< Takes a single measurement then switches to
               ///< AK09916_MAG_DATARATE_SHUTDOWN
        AK09916_MAG_DATARATE_10_HZ  = 0x2, ///< updates at 10Hz
        AK09916_MAG_DATARATE_20_HZ  = 0x4, ///< updates at 20Hz
        AK09916_MAG_DATARATE_50_HZ  = 0x6, ///< updates at 50Hz
        AK09916_MAG_DATARATE_100_HZ = 0x8, ///< updates at 100Hz
    };

    enum Result
    {
        OK = 0,
        ERR
    };

    /** Initialize the Icm20948 device
        \param config Configuration settings
    */
    Result Init(Config config)
    {
        config_ = config;

        transport_.Init(config_.transport_config);

        SetBank(0);

        uint8_t chip_id = Read8(ICM20X_B0_WHOAMI);

        if(chip_id != ICM20948_CHIP_ID)
        {
            return ERR;
        }

        _sensorid_accel = sensor_id;
        _sensorid_gyro  = sensor_id + 1;
        _sensorid_mag   = sensor_id + 2;
        _sensorid_temp  = sensor_id + 3;

        Reset();

        // take out of default sleep state
        WriteBits(ICM20X_B0_PWR_MGMT_1, 0, 1, 6);

        // 3 will be the largest range for either sensor
        WriteGyroRange(3);
        WriteAccelRange(3);

        // 1100Hz/(1+10) = 100Hz
        SetGyroRateDivisor(10);

        // # 1125Hz/(1+20) = 53.57Hz
        SetAccelRateDivisor(20);

        temp_sensor  = new Adafruit_ICM20X_Temp(this);
        accel_sensor = new Adafruit_ICM20X_Accelerometer(this);
        gyro_sensor  = new Adafruit_ICM20X_Gyro(this);
        mag_sensor   = new Adafruit_ICM20X_Magnetometer(this);

        System::Delay(20);

        return GetTransportError();
    }

    /** Reset the internal registers and restores the default settings */
    void Reset()
    {
        SetBank(0);

        WriteBits(ICM20X_B0_PWR_MGMT_1, 1, 1, 7);
        System::Delay(20);

        while(reset_bit.read())
        {
            System::Delay(10);
        };

        System::Delay(50);
    }


    uint8_t GetMagId()
    {
        // verify the magnetometer id
        return ReadExternalRegister(0x8C, 0x01);
    }

    Result SetupMag()
    {
        uint8_t buffer[2];

        SetI2CBypass(false);

        ConfigureI2CMaster();

        EnableI2CMaster(true);

        if(AuxI2CBusSetupFailed())
        {
            return ERR;
        }

        // set mag data rate
        if(!SetMagDataRate(AK09916_MAG_DATARATE_100_HZ))
        {
            // Serial.println("Error setting magnetometer data rate on external bus");
            return ERR;
        }

        // TODO: extract method
        // Set up Slave0 to proxy Mag readings
        SetBank(3);

        // set up slave0 to proxy reads to mag
        Write8(ICM20X_B3_I2C_SLV0_ADDR, 0x8C);
        if(GetTransportError() != OK)
        {
            return ERR;
        }

        Write8(ICM20X_B3_I2C_SLV0_REG, 0x10);
        if(GetTransportError() != OK)
        {
            return ERR;
        }

        // enable, read 9 bytes
        Write8(ICM20X_B3_I2C_SLV0_CTRL, 0x89);
        if(GetTransportError() != OK)
        {
            return ERR;
        }

        return OK;
    }

    /**
        \param slv_addr
        \param mag_reg_addr
        \param num_finished_checks
        \return uint8_t
    */
    uint8_t ReadMagRegister(uint8_t mag_reg_addr)
    {
        return ReadExternalRegister(0x8C, mag_reg_addr);
    }

    bool WriteMagRegister(uint8_t mag_reg_addr, uint8_t value)
    {
        return WriteExternalRegister(0x0C, mag_reg_addr, value);
    }

    void ScaleValues()
    {
        icm20948_gyro_range_t gyro_range
            = (icm20948_gyro_range_t)current_gyro_range_;
        icm20948_accel_range_t accel_range
            = (icm20948_accel_range_t)current_accel_range_;

        float accel_scale = 1.0;
        float gyro_scale  = 1.0;

        if(gyro_range == ICM20948_GYRO_RANGE_250_DPS)
            gyro_scale = 131.0;
        if(gyro_range == ICM20948_GYRO_RANGE_500_DPS)
            gyro_scale = 65.5;
        if(gyro_range == ICM20948_GYRO_RANGE_1000_DPS)
            gyro_scale = 32.8;
        if(gyro_range == ICM20948_GYRO_RANGE_2000_DPS)
            gyro_scale = 16.4;

        if(accel_range == ICM20948_ACCEL_RANGE_2_G)
            accel_scale = 16384.0;
        if(accel_range == ICM20948_ACCEL_RANGE_4_G)
            accel_scale = 8192.0;
        if(accel_range == ICM20948_ACCEL_RANGE_8_G)
            accel_scale = 4096.0;
        if(accel_range == ICM20948_ACCEL_RANGE_16_G)
            accel_scale = 2048.0;

        gyroX = rawGyroX / gyro_scale;
        gyroY = rawGyroY / gyro_scale;
        gyroZ = rawGyroZ / gyro_scale;

        accX = rawAccX / accel_scale;
        accY = rawAccY / accel_scale;
        accZ = rawAccZ / accel_scale;

        magX = rawMagX * ICM20948_UT_PER_LSB;
        magY = rawMagY * ICM20948_UT_PER_LSB;
        magZ = rawMagZ * ICM20948_UT_PER_LSB;
    }

    /** Get the accelerometer's measurement range.
        \return The accelerometer's measurement range (`icm20948_accel_range_t`).
    */
    icm20948_accel_range_t GetAccelRange()
    {
        return (icm20948_accel_range_t)ReadAccelRange();
    }

    /** Get the accelerometer's measurement range.
        \return The accelerometer's measurement range (`icm20x_accel_range_t`).
    */
    uint8_t ReadAccelRange()
    {
        SetBank(2);
        uint8_t range = ReadBits(ICM20X_B2_ACCEL_CONFIG_1, 2, 1);
        SetBank(0);
        return range;
    }

    /** Sets the accelerometer's measurement range.
        \param  new_accel_range Measurement range to be set. Must be an `icm20x_accel_range_t`.
    */
    void WriteAccelRange(uint8_t new_accel_range)
    {
        SetBank(2);
        WriteBits(ICM20X_B2_ACCEL_CONFIG_1, new_accel_range, 2, 1);
        current_accel_range_ = new_accel_range;
        SetBank(0);
    }

    /** Sets the accelerometer's measurement range.
        \param  new_accel_range Measurement range to be set. Must be an `icm20948_accel_range_t`.
    */
    void SetAccelRange(icm20948_accel_range_t new_accel_range)
    {
        WriteAccelRange((uint8_t)new_accel_range);
    }

    /** Get the gyro's measurement range.
        \return The gyro's measurement range (`icm20948_gyro_range_t`).
    */
    icm20948_gyro_range_t GetGyroRange()
    {
        return (icm20948_gyro_range_t)ReadGyroRange();
    }

    /** Sets the gyro's measurement range.
        \param  new_gyro_range Measurement range to be set. Must be an `icm20948_gyro_range_t`.
    */
    void SetGyroRange(icm20948_gyro_range_t new_gyro_range)
    {
        WriteGyroRange((uint8_t)new_gyro_range);
    }

    /** Sets the gyro's measurement range.
        \param  new_gyro_range Measurement range to be set. Must be an  `icm20x_gyro_range_t`.
    */
    void WriteGyroRange(uint8_t new_gyro_range)
    {
        SetBank(2);
        WriteBits(ICM20X_B2_GYRO_CONFIG_1, new_gyro_range, 2, 1);
        current_gyro_range_ = new_gyro_range;
        SetBank(0);
    }


    /** Get the gyro's measurement range.
        \return The gyro's measurement range (`icm20x_gyro_range_t`).
    */
    uint8_t ReadGyroRange()
    {
        SetBank(2);
        uint8_t range = ReadBits(ICM20X_B2_GYRO_CONFIG_1, 2, 1);
        SetBank(0);
        return range;
    }


    /** Get the current magnetometer measurement rate
        \return ak09916_data_rate_t the current rate
    */
    ak09916_data_rate_t GetMagDataRate()
    {
        uint8_t raw_mag_rate = ReadMagRegister(AK09916_CNTL2);
        return (ak09916_data_rate_t)(raw_mag_rate);
    }

    /** Set the magnetometer measurement rate
        \param rate The rate to set.
        \return true: success false: failure
    */
    bool SetMagDataRate(ak09916_data_rate_t rate)
    {
        /* Following the datasheet, the sensor will be set to
        * AK09916_MAG_DATARATE_SHUTDOWN followed by a 100ms delay, followed by
        * setting the new data rate.
        *
        * See page 9 of https://www.y-ic.es/datasheet/78/SMDSW.020-2OZ.pdf */

        // don't need to read/mask because there's nothing else in the register and
        // it's right justified
        bool success
            = WriteMagRegister(AK09916_CNTL2, AK09916_MAG_DATARATE_SHUTDOWN);
        System::Delay(1);
        return WriteMagRegister(AK09916_CNTL2, rate) && success;
    }

    GetMagId()
    {
        // verify the magnetometer id
        return ReadExternalRegister(0x8C, 0x01);
    }

    /** Sets register bank.
        \param bank_number The bank to set to active
    */
    void SetBank(uint8_t bank_number)
    {
        Write8(ICM20X_B0_REG_BANK_SEL, (bank_number & 0b11) << 4);
    }


    /** Read a single byte from a given register address for an I2C slave device on the auxiliary I2C bus
        \param slv_addr the 7-bit I2C address of the slave device
        \param reg_addr the register address to read from
        \return the requested register value
    */
    uint8_t ReadExternalRegister(uint8_t slv_addr, uint8_t reg_addr)
    {
        return AuxillaryRegisterTransaction(true, slv_addr, reg_addr);
    }

    /** Write a single byte to a given register address for an I2C slave device on the auxiliary I2C bus
        \param slv_addr the 7-bit I2C address of the slave device
        \param reg_addr the register address to write to
        \param value the value to write
        \return true
        \return false
    */
    bool
    WriteExternalRegister(uint8_t slv_addr, uint8_t reg_addr, uint8_t value)
    {
        return (bool)AuxillaryRegisterTransaction(
            false, slv_addr, reg_addr, value);
    }

    /** Read / Write a single byte to a given register address for an I2C slave device on the auxiliary I2C bus
        \param slv_addr the 7-bit I2C address of the slave device
        \param reg_addr the register address to write to
        \param value the value to write
        \return Read value ( if it's a read operation ), else true or false
    */
    uint8_t AuxillaryRegisterTransaction(bool    read,
                                         uint8_t slv_addr,
                                         uint8_t reg_addr,
                                         uint8_t value)
    {
        SetBank(3);

        if(read)
        {
            // set high bit for read, presumably for multi-byte reads
            slv_addr |= 0x80;
        }
        else
        {
            Write8(ICM20X_B3_I2C_SLV4_DO, value);
            if(GetTransportError() == ERR)
            {
                return (uint8_t) false;
            }
        }

        Write8(ICM20X_B3_I2C_SLV4_ADDR, slv_addr);
        if(GetTransportError() == ERR)
        {
            return (uint8_t) false;
        }

        Write8(ICM20X_B3_I2C_SLV4_REG, reg_addr);
        if(GetTransportError() == ERR)
        {
            return (uint8_t) false;
        }

        Write8(ICM20X_B3_I2C_SLV4_CTRL, 0x80);
        if(GetTransportError() == ERR)
        {
            return (uint8_t) false;
        }

        SetBank(0);
        uint8_t tries = 0;
        // wait until the operation is finished
        while(ReadBits(ICM20X_B0_I2C_MST_STATUS, 1, 6) != true)
        {
            tries++;
            if(tries >= NUM_FINISHED_CHECKS)
            {
                return (uint8_t) false;
            }
        }

        if(read)
        {
            SetBank(3);
            return Read8(ICM20X_B3_I2C_SLV4_DI);
        }

        return (uint8_t) true;
    }

    /**  Writes an 8 bit value
        \param reg the register address to write to
        \param value the value to write to the register
    */
    void Write8(uint8_t reg, uint8_t value)
    {
        return transport_.Write8(reg, value);
    }

    /** Read from a reg address a defined number of bytes */
    void ReadReg(uint8_t reg, uint8_t *buff, uint8_t size)
    {
        return transport_.ReadReg(reg, buff, size);
    }

    uint8_t ReadBits(uint8_t reg, uint8_t bits, uint8_t shift)
    {
        uint8_t val = Read8(reg);
        val >>= shift;
        return val & ((1 << (bits)) - 1);
    }

    void WriteBits(uint8_t reg, uint8_t data, uint8_t bits, uint8_t shift)
    {
        uint8_t val = Read8(reg);

        // mask off the data before writing
        uint8_t mask = (1 << (bits)) - 1;
        data &= mask;

        mask <<= shift;
        val &= ~mask;         // remove the current data at that spot
        val |= data << shift; // and add in the new data

        Write8(reg, val);
    }

    /**  Reads an 8 bit value
        \param reg the register address to read from
        \return the data uint8_t read from the device
    */
    uint8_t Read8(uint8_t reg) { return transport_.Read8(reg); }

    /** Get and reset the transport error flag
        \return Whether the transport has errored since the last check
    */
    Result GetTransportError() { return transport_.GetError() ? ERR : OK; }

  private:
    Config    config_;
    Transport transport_;

    uint8_t current_accel_range_; ///< accelerometer range cache
    uint8_t current_gyro_range_;  ///< gyro range cache
};

/** @} */

using Icm20948I2C = Icm20948<Icm20948I2CTransport>;
using Icm20948Spi = Icm20948<Icm20948SpiTransport>;
} // namespace daisy
#endif