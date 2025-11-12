/* =========================================================================
I2Cbus library is placed under the MIT License
Copyright 2017 Natanael Josue Rabello. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
 ========================================================================= */

#ifndef _I2CBUS_HPP_
#define _I2CBUS_HPP_

#include <stdint.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <vector>
#include <utility>


/* ^^^^^^
 * I2Cbus
 * ^^^^^^ */
namespace i2cbus {
constexpr uint32_t kDefaultClockSpeed = 100000;  /*!< Clock speed in Hz, default: 100KHz */
constexpr uint32_t kDefaultTimeout = 1000;       /*!< Timeout in milliseconds, default: 1000ms */
class I2C;
}  // namespace i2cbus

// I2Cbus type
using I2C_t = i2cbus::I2C;

// Default Objects
extern I2C_t i2c0;        /*!< port: I2C_NUM_0 */
extern I2C_t i2c1;        /*!< port: I2C_NUM_1 */


// I2C class definition
namespace i2cbus {
class I2C {
 private:
    i2c_port_t port;            /*!< I2C port: I2C_NUM_0 or I2C_NUM_1 */
    uint32_t ticksToWait;       /*!< Timeout in ticks for read and write */
    i2c_master_bus_config_t m_i2c_mst_config = {};  /*!< I2C master bus configuration */
    i2c_master_bus_handle_t m_i2c_mst_handle;       /*!< I2C master bus handle */
    uint32_t m_default_clk_speed = kDefaultClockSpeed;
    std::vector<std::pair<i2c_master_dev_handle_t, uint8_t>> m_devices;
    uint8_t findAddr(i2c_master_dev_handle_t h) const;
    i2c_master_dev_handle_t getDeviceHandle(uint8_t devAddr);
    esp_err_t addDevice(i2c_device_config_t devConfig, i2c_master_dev_handle_t* deviceHandle);
    esp_err_t removeDevice(i2c_master_dev_handle_t deviceHandle);

public:
    explicit I2C(i2c_port_t port);
    ~I2C();

    /** *** I2C Begin ***
     * @brief  Config I2C bus and Install Driver
     * @param  sda_io_num    [GPIO number for SDA line]
     * @param  scl_io_num    [GPIO number for SCL line]
     * @param  pullup_en     [Enable internal pullups on SDA and SCL lines]
     * @param  clk_speed     [I2C clock frequency for master mode, (no higher than 1MHz for now), Default 100KHz]
     *                       @see "driver/i2c_master.h"
     * @return               - ESP_OK   Success
     *                       - ESP_ERR_INVALID_ARG Parameter error
     *                       - ESP_FAIL Driver install error
     */
    esp_err_t begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed = kDefaultClockSpeed);

    esp_err_t begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num,
                    gpio_pullup_t pullup_en,
                    uint32_t clk_speed = kDefaultClockSpeed);

    /**
     * Stop I2C bus and unninstall driver
     */
    esp_err_t close();

    /**
     * Timeout read and write in milliseconds
     */
    void setTimeout(uint32_t ms);

    /**
     * Device management interface for new i2c bus handling
     */
    esp_err_t addDevice(i2c_device_config_t devConfig);
    esp_err_t removeDevice(uint8_t devAddr);

    /**
     * *** WRITING interface ***
     * @brief  I2C commands for writing to a 8-bit slave device register.
     *         All of them returns standard esp_err_t codes. So it can be used
     *         with ESP_ERROR_CHECK();
     * @param  devAddr   [I2C slave device address]
     * @param  regAddr   [I2C slave device register]
     * @param  bitNum    [Bit position number to write to (bit 7~0)]
     * @param  bitStart  [Start bit number when writing a bit-sequence (MSB)]
     * @param  data      [Value(s) to be write to the register]
     * @param  length    [Number of bytes to write (should be within the data buffer size)]
     *                   [writeBits() -> Number of bits after bitStart (including)]
     * @param  timeout   [Custom timeout for the particular call]
     * @return  - ESP_OK Success
     *          - ESP_ERR_INVALID_ARG Parameter error
     *          - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
     *          - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
     *          - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
     */
    esp_err_t writeReg(uint8_t devAddr, uint8_t regAddr, uint8_t val, int32_t timeout = -1);
    esp_err_t writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data, int32_t timeout = -1);
    esp_err_t writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data, int32_t timeout = -1);
    esp_err_t writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data, int32_t timeout = -1);
    esp_err_t writeBytes(uint8_t devAddr, uint8_t regAddr, size_t length, const uint8_t *data, int32_t timeout = -1);

    /**
     * *** READING interface ***
     * @breif  I2C commands for reading a 8-bit slave device register.
     *         All of them returns standard esp_err_t codes.So it can be used
     *         with ESP_ERROR_CHECK();
     * @param  devAddr   [I2C slave device address]
     * @param  regAddr   [I2C slave device register]
     * @param  bitNum    [Bit position number to write to (bit 7~0)]
     * @param  bitStart  [Start bit number when writing a bit-sequence (MSB)]
     * @param  data      [Buffer to store the read value(s)]
     * @param  length    [Number of bytes to read (should be within the data buffer size)]
     * @param  timeout   [Custom timeout for the particular call]
     * @return  - ESP_OK Success
     *          - ESP_ERR_INVALID_ARG Parameter error
     *          - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
     *          - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
     *          - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.]
     */
    esp_err_t readReg(uint8_t devAddr, uint8_t regAddr, uint8_t* val, int32_t timeout = -1);
    esp_err_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, int32_t timeout = -1);
    esp_err_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, int32_t timeout = -1);
    esp_err_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, int32_t timeout = -1);
    esp_err_t readBytes(uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *data, int32_t timeout = -1);

    // ---------- 16-bit register variants ----------
    esp_err_t writeReg16(uint8_t devAddr, uint16_t regAddr, const uint8_t* data, size_t len, int32_t timeout = -1);
    esp_err_t readReg16(uint8_t devAddr, uint16_t regAddr, uint8_t* data, size_t len, int32_t timeout = -1);

    // ---------- Raw stream interface (explicit) ----------
    esp_err_t rawWrite(uint8_t devAddr, const uint8_t* data, size_t len, int32_t timeout = -1);
    esp_err_t rawRead (uint8_t devAddr, uint8_t* data, size_t len, int32_t timeout = -1);
    esp_err_t rawWriteRead(uint8_t devAddr,
                             const uint8_t* writeData, size_t writeLen,
                             uint8_t* readData, size_t readLen,
                             int32_t timeout = -1);

    /**
     * @brief  Quick check to see if a slave device responds.
     * @param  devAddr   [I2C slave device register]
     * @param  timeout   [Custom timeout for the particular call]
     * @return  - ESP_OK Success
     *          - ESP_ERR_INVALID_ARG Parameter error
     *          - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
     *          - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
     *          - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.]
     */
    esp_err_t testConnection(uint8_t devAddr, int32_t timeout = -1);

    /**
     * I2C scanner utility, prints out all device addresses found on this I2C bus.
     */
    void scanner();
};

}  // namespace i2cbus


/* Get default objects */
constexpr I2C_t& getI2C(i2c_port_t port) {
    return port == 0 ? i2c0 : i2c1;
}



#endif /* end of include guard: _I2CBUS_H_ */