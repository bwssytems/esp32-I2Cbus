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

#include "I2Cbus.hpp"
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <cstring>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#if defined CONFIG_I2CBUS_LOG_RW_LEVEL_INFO
#define I2CBUS_LOG_RW(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#elif defined CONFIG_I2CBUS_LOG_RW_LEVEL_DEBUG
#define I2CBUS_LOG_RW(format, ...) ESP_LOGD(TAG, format, ##__VA_ARGS__)
#elif defined CONFIG_I2CBUS_LOG_RW_LEVEL_VERBOSE
#define I2CBUS_LOG_RW(format, ...) ESP_LOGV(TAG, format, ##__VA_ARGS__)
#endif
#define I2CBUS_LOGE(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)

#define I2C_MASTER_ACK_EN true   /*!< Enable ack check for master */
#define I2C_MASTER_ACK_DIS false /*!< Disable ack check for master */

static const char *TAG __attribute__((unused)) = "I2Cbus";

/*******************************************************************************
 * OBJECTS
 ******************************************************************************/
I2C_t i2c0 = i2cbus::I2C(I2C_NUM_0);
I2C_t i2c1 = i2cbus::I2C(I2C_NUM_1);

/* ^^^^^^
 * I2Cbus
 * ^^^^^^ */
namespace i2cbus
{
    /*******************************************************************************
     * SETUP
     ******************************************************************************/
    I2C::I2C(i2c_port_t port) : port{port}, timeoutMs{kDefaultTimeout}
    {
    }

    I2C::~I2C()
    {
        close();
    }

    esp_err_t I2C::begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed)
    {
        return begin(sda_io_num, scl_io_num, GPIO_PULLUP_ENABLE, clk_speed);
    }

    esp_err_t I2C::begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, gpio_pullup_t pullup_en,
                         uint32_t clk_speed)
    {
        m_lastBeginCreatedBus = false;
        m_default_clk_speed = clk_speed;
        m_i2c_mst_config = {}; // zero-init
        m_i2c_mst_config.i2c_port = port;
        m_i2c_mst_config.sda_io_num = sda_io_num;
        m_i2c_mst_config.scl_io_num = scl_io_num;
        m_i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
        m_i2c_mst_config.glitch_ignore_cnt = 7;
        m_i2c_mst_config.flags.enable_internal_pullup = (pullup_en == GPIO_PULLUP_ENABLE);

        esp_err_t err = i2c_new_master_bus(&m_i2c_mst_config, &m_i2c_mst_handle);
        if (err == ESP_OK) {
            m_lastBeginCreatedBus = true;
            return ESP_OK;
        }

        if (err == ESP_ERR_INVALID_STATE) {
            err = i2c_master_get_bus_handle(port, &m_i2c_mst_handle);
            if (err == ESP_OK && m_i2c_mst_handle != nullptr) {
                ESP_LOGD(TAG, "I2C bus already initialized.");
                return ESP_OK;
            }
        }

        return err;
    }

    esp_err_t I2C::close()
    {
        if (m_i2c_mst_handle == nullptr)
            return ESP_ERR_INVALID_STATE;
        // Remove any remaining devices to be tidy (optional if you remove explicitly elsewhere)
        for (auto &p : m_devices)
        {
            i2c_master_bus_rm_device(p.first);
        }
        m_devices.clear();
        esp_err_t err = i2c_del_master_bus(m_i2c_mst_handle);
        if (err == ESP_OK)
            m_i2c_mst_handle = nullptr;
        m_lastBeginCreatedBus = false;
        return err;
    }

    void I2C::setTimeout(uint32_t ms)
    {
        timeoutMs = ms;
    }

    // ---------- Bit helpers ----------
    static inline bool _bitsArgsOk(uint8_t bitStart, uint8_t length)
    {
        return (length >= 1 && length <= 8 && bitStart < 8 && length <= (uint8_t)(bitStart + 1));
    }

    /*******************************************************************************
     * WRITING
     ******************************************************************************/
    esp_err_t I2C::writeReg(uint8_t devAddr, uint8_t regAddr, uint8_t val, int32_t timeout)
    {
        uint8_t buf[2] = {regAddr, val};
        return rawWrite(devAddr, buf, 2, timeout);
    }

    esp_err_t I2C::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data, int32_t timeout)
    {
        uint8_t v;
        esp_err_t err = readReg(devAddr, regAddr, &v, timeout);
        if (err)
            return err;
        if (bitNum > 7)
            return ESP_ERR_INVALID_ARG;
        if (data)
            v |= (1u << bitNum);
        else
            v &= ~(1u << bitNum);
        return writeReg(devAddr, regAddr, v, timeout);
    }

    esp_err_t I2C::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data, int32_t timeout)
    {
        if (!_bitsArgsOk(bitStart, length))
            return ESP_ERR_INVALID_ARG;
        uint8_t v;
        esp_err_t err = readReg(devAddr, regAddr, &v, timeout);
        if (err)
            return err;
        uint32_t mask = ((1u << length) - 1u) << (bitStart - length + 1u);
        uint8_t d = (uint8_t)((data << (bitStart - length + 1u)) & mask);
        v = (uint8_t)((v & ~mask) | d);
        return writeReg(devAddr, regAddr, v, timeout);
    }

    esp_err_t I2C::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data, int32_t timeout)
    {
        return writeBytes(devAddr, regAddr, 1, &data, timeout);
    }

    esp_err_t I2C::writeBytes(uint8_t devAddr, uint8_t regAddr, size_t length, const uint8_t *data, int32_t timeout)
    {
        esp_err_t err;
        std::string frame;
        frame.resize(1 + length);
        frame[0] = static_cast<char>(regAddr);
        if (length)
            memcpy(&frame[1], data, length);
        err = rawWrite(devAddr, reinterpret_cast<const uint8_t *>(frame.data()), frame.size(), timeout);
#if defined CONFIG_I2CBUS_LOG_READWRITES
        if (!err)
        {
            std::string hex;
            hex.reserve(length * 5);
            for (size_t i = 0; i < length; i++)
            {
                char tmp[6];
                sprintf(tmp, "0x%02X ", data[i]);
                hex += tmp;
            }
            I2CBUS_LOG_RW("[port:%d, slave:0x%02X] %s %d bytes, data: %s",
                          port, (devAddr == 0xFF ? 0 : devAddr),
                          __FUNCTION__[0] == 'w' ? "Write" : "Read",
                          (int)length, hex.c_str());
        }
#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
        if (err)
        {
            I2CBUS_LOGE("[port:%d, slave:0x%02X] Failed to %s %d bytes, error: 0x%X",
                        port, (devAddr == 0xFF ? 0 : devAddr),
                        __FUNCTION__[0] == 'w' ? "write" : "read",
                        (int)length, err);
        }
#endif
        return err;
    }

    /*******************************************************************************
     * READING
     ******************************************************************************/
    esp_err_t I2C::readReg(uint8_t devAddr, uint8_t regAddr, uint8_t *val, int32_t timeout)
    {
        esp_err_t err = rawWrite(devAddr, &regAddr, 1, timeout);
        if (err != ESP_OK)
            return err;
        return rawRead(devAddr, val, 1, timeout);
    }

    esp_err_t I2C::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, int32_t timeout)
    {
        if (!data || bitNum > 7)
            return ESP_ERR_INVALID_ARG;
        uint8_t v;
        esp_err_t err = readReg(devAddr, regAddr, &v, timeout);
        if (err)
            return err;
        *data = (uint8_t)((v >> bitNum) & 0x1);
        return ESP_OK;
    }

    esp_err_t I2C::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, int32_t timeout)
    {
        if (!data || !_bitsArgsOk(bitStart, length))
            return ESP_ERR_INVALID_ARG;
        uint8_t v;
        esp_err_t err = readReg(devAddr, regAddr, &v, timeout);
        if (err)
            return err;
        uint32_t mask = ((1u << length) - 1u) << (bitStart - length + 1u);
        *data = (uint8_t)((v & mask) >> (bitStart - length + 1u));
        return ESP_OK;
    }

    esp_err_t I2C::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, int32_t timeout)
    {
        return readBytes(devAddr, regAddr, 1, data, timeout);
    }

    esp_err_t I2C::readBytes(uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *data, int32_t timeout)
    {
        esp_err_t err = rawWriteRead(
                devAddr,
                &regAddr,
                1,
                data,
                length,
                timeout
            );
//        esp_err_t err = rawWrite(devAddr, &regAddr, 1, timeout);
//        if (err != ESP_OK)
//            return err;
//        err = rawRead(devAddr, data, length, timeout);
#if defined CONFIG_I2CBUS_LOG_READWRITES
        if (!err)
        {
            std::string hex;
            hex.reserve(length * 5);
            for (size_t i = 0; i < length; i++)
            {
                char tmp[6];
                sprintf(tmp, "0x%02X ", data[i]);
                hex += tmp;
            }

            I2CBUS_LOG_RW("[port:%d, slave:0x%02X] %s %d bytes, data: %s",
                          port, (devAddr == 0xFF ? 0 : devAddr),
                          __FUNCTION__[0] == 'w' ? "Write" : "Read",
                          (int)length, hex.c_str());
        }
#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
        if (err)
        {
            I2CBUS_LOGE("[port:%d, slave:0x%02X] Failed to %s %d bytes, error: 0x%X",
                        port, (devAddr == 0xFF ? 0 : devAddr),
                        __FUNCTION__[0] == 'w' ? "write" : "read",
                        (int)length, err);
        }
#endif
        return err;
    }

    // ---------- 16-bit register variants ----------
    esp_err_t I2C::writeReg16(uint8_t devAddr, uint16_t regAddr, const uint8_t *data, size_t len, int32_t timeout)
    {
        uint8_t addr[2 + len];
        addr[0] = (uint8_t)((regAddr >> 8) & 0xFF);
        addr[1] = (uint8_t)(regAddr & 0xFF);
        if (len)
            memcpy(&addr[2], data, len);
        return rawWrite(devAddr, addr, 2 + len, timeout);
    }

    esp_err_t I2C::readReg16(uint8_t devAddr, uint16_t regAddr, uint8_t *data, size_t len, int32_t timeout)
    {
        uint8_t addr[2] = {(uint8_t)((regAddr >> 8) & 0xFF), (uint8_t)(regAddr & 0xFF)};
        return rawWriteRead(devAddr, addr, 2, data, len, timeout);
    }

    // ---------- Raw stream ----------
    esp_err_t I2C::rawWrite(uint8_t devAddr, const uint8_t *data, size_t len, int32_t timeout)
    {
        i2c_master_dev_handle_t deviceHandle = getDeviceHandle(devAddr);
        if (!deviceHandle) {
            const esp_err_t rebuildErr = rebuildDevice(devAddr);
            if (rebuildErr != ESP_OK) {
                return rebuildErr;
            }
            deviceHandle = getDeviceHandle(devAddr);
            if (!deviceHandle) {
                return ESP_ERR_INVALID_STATE;
            }
        }

        esp_err_t err = i2c_master_transmit(deviceHandle, const_cast<uint8_t *>(data), len,
                                            (timeout < 0 ? static_cast<int>(timeoutMs) : timeout));
        if (err == ESP_OK) {
            return ESP_OK;
        }

        if (err == ESP_ERR_INVALID_STATE || err == ESP_ERR_TIMEOUT) {
            (void)i2c_master_bus_reset(m_i2c_mst_handle);
            if (rebuildDevice(devAddr) == ESP_OK) {
                deviceHandle = getDeviceHandle(devAddr);
                if (deviceHandle) {
                    err = i2c_master_transmit(deviceHandle, const_cast<uint8_t *>(data), len,
                                              (timeout < 0 ? static_cast<int>(timeoutMs) : timeout));
                }
            }
        }

        return err;
    }

    esp_err_t I2C::rawRead(uint8_t devAddr, uint8_t *data, size_t len, int32_t timeout)
    {
        i2c_master_dev_handle_t deviceHandle = getDeviceHandle(devAddr);
        if (!deviceHandle) {
            const esp_err_t rebuildErr = rebuildDevice(devAddr);
            if (rebuildErr != ESP_OK) {
                return rebuildErr;
            }
            deviceHandle = getDeviceHandle(devAddr);
            if (!deviceHandle) {
                return ESP_ERR_INVALID_STATE;
            }
        }

        esp_err_t err = i2c_master_receive(deviceHandle, data, len,
                                           (timeout < 0 ? static_cast<int>(timeoutMs) : timeout));
        if (err == ESP_OK) {
            return ESP_OK;
        }

        if (err == ESP_ERR_INVALID_STATE || err == ESP_ERR_TIMEOUT) {
            (void)i2c_master_bus_reset(m_i2c_mst_handle);
            if (rebuildDevice(devAddr) == ESP_OK) {
                deviceHandle = getDeviceHandle(devAddr);
                if (deviceHandle) {
                    err = i2c_master_receive(deviceHandle, data, len,
                                             (timeout < 0 ? static_cast<int>(timeoutMs) : timeout));
                }
            }
        }

        return err;
    }

    esp_err_t I2C::rawWriteRead(uint8_t devAddr,
                                 const uint8_t* writeData, size_t writeLen,
                                 uint8_t* readData, size_t readLen,
                                 int32_t timeout)
    {
        constexpr int kRawWriteReadRetryCount = CONFIG_I2CBUS_RAW_WRITE_READ_RETRY_COUNT;
        constexpr int kRawWriteReadRetryDelayMs = CONFIG_I2CBUS_RAW_WRITE_READ_RETRY_DELAY_MS;
        esp_err_t err = ESP_ERR_INVALID_STATE;
        for(int i = 0; i < kRawWriteReadRetryCount; i++) {
            i2c_master_dev_handle_t deviceHandle = getDeviceHandle(devAddr);
            if (!deviceHandle) {
                const esp_err_t rebuildErr = rebuildDevice(devAddr);
                if (rebuildErr != ESP_OK) {
                    err = rebuildErr;
                    ++m_rawWriteReadFailureCount;
                    vTaskDelay(pdMS_TO_TICKS(kRawWriteReadRetryDelayMs));
                    continue;
                }
                deviceHandle = getDeviceHandle(devAddr);
                if (!deviceHandle) {
                    err = ESP_ERR_INVALID_STATE;
                    ++m_rawWriteReadFailureCount;
                    vTaskDelay(pdMS_TO_TICKS(kRawWriteReadRetryDelayMs));
                    continue;
                }
            }

            err = i2c_master_transmit_receive(deviceHandle,
                                                        writeData, writeLen,
                                                        readData, readLen,
                                                        (timeout < 0 ? static_cast<int>(timeoutMs) : timeout));
            if (err == ESP_OK) {
                return ESP_OK;
            }
            ++m_rawWriteReadFailureCount;
            i2c_master_bus_reset(m_i2c_mst_handle);
            (void)rebuildDevice(devAddr);
            vTaskDelay(pdMS_TO_TICKS(kRawWriteReadRetryDelayMs));
        }
        return err;
    }

    uint32_t I2C::getRawWriteReadFailureCount() const
    {
        return m_rawWriteReadFailureCount;
    }

    void I2C::resetRawWriteReadFailureCount()
    {
        m_rawWriteReadFailureCount = 0;
    }

    /*******************************************************************************
     * UTILS
     ******************************************************************************/
    esp_err_t I2C::probeBus(uint8_t devAddr, int32_t timeout)
    {
        esp_err_t err = i2c_master_probe(m_i2c_mst_handle, devAddr,
                                         (timeout < 0 ? static_cast<int>(timeoutMs) : timeout));

        return err;
    }

    void I2C::scanner()
    {
        constexpr int32_t scanTimeout = 1000;
        printf(LOG_COLOR_W "\n>> I2C scanning ..." LOG_RESET_COLOR "\n");
        uint8_t count = 0;
        for (size_t i = 0x3; i < 0x78; i++)
        {
            if (probeBus(i, scanTimeout) == ESP_OK)
            {
                printf(LOG_COLOR_W "- Device found at address 0x%X%s", i, LOG_RESET_COLOR "\n");
                count++;
            }
        }
        if (count == 0)
            printf(LOG_COLOR_E "- No I2C devices found!" LOG_RESET_COLOR "\n");
        printf("\n");
    }

    uint8_t I2C::findAddr(i2c_master_dev_handle_t h) const
    {
        for (auto &p : m_devices)
            if (p.first == h)
                return p.second;
        return 0xFF; // unknown
    }

    i2c_master_dev_handle_t I2C::getDeviceHandle(uint8_t devAddr) {
        for (auto it = m_devices.rbegin(); it != m_devices.rend(); ++it) {
            if (it->second == devAddr) {
                return (i2c_master_dev_handle_t)it->first;
            }
        }
        return nullptr;
    }

    esp_err_t I2C::rebuildDevice(uint8_t devAddr)
    {
        if (m_i2c_mst_handle == nullptr) {
            return ESP_ERR_INVALID_STATE;
        }

        for (auto it = m_devices.begin(); it != m_devices.end(); ) {
            if (it->second == devAddr) {
                (void)i2c_master_bus_rm_device(it->first);
                it = m_devices.erase(it);
            } else {
                ++it;
            }
        }

        i2c_device_config_t devConfig = {};
        devConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        devConfig.device_address = devAddr;
        devConfig.scl_speed_hz = m_default_clk_speed;
        devConfig.scl_wait_us = 0;

        i2c_master_dev_handle_t handle = nullptr;
        const esp_err_t err = i2c_master_bus_add_device(m_i2c_mst_handle, &devConfig, &handle);
        if (err == ESP_OK) {
            m_devices.emplace_back(handle, devAddr);
        }
        return err;
    }
    
    esp_err_t I2C::addDevice(i2c_device_config_t devConfig, i2c_master_dev_handle_t *deviceHandle)
    {
        // If caller didn’t set a speed, use default
        if (devConfig.scl_speed_hz == 0)
        {
            devConfig.scl_speed_hz = m_default_clk_speed;
        }
        esp_err_t err = i2c_master_bus_add_device(m_i2c_mst_handle, &devConfig, deviceHandle);
        if (err == ESP_OK)
        {
            m_devices.emplace_back(*deviceHandle, devConfig.device_address);
        }
        return err;
    }

    esp_err_t I2C::removeDevice(i2c_master_dev_handle_t deviceHandle)
    {
        esp_err_t err = i2c_master_bus_rm_device(deviceHandle);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE || err == ESP_ERR_INVALID_ARG)
        {
            for (auto it = m_devices.begin(); it != m_devices.end(); ++it)
            {
                if (it->first == deviceHandle)
                {
                    m_devices.erase(it);
                    break;
                }
            }
        }
        return err;
    }

    esp_err_t I2C::addDevice(i2c_device_config_t devConfig) {
        i2c_master_dev_handle_t handle;
        return addDevice(devConfig, &handle);
    }

    esp_err_t I2C::removeDevice(uint8_t devAddr) {
        std::vector<i2c_master_dev_handle_t> handles;
        handles.reserve(m_devices.size());
        for (const auto &p : m_devices) {
            if (p.second == devAddr) {
                handles.push_back(p.first);
            }
        }

        if (handles.empty()) {
            return ESP_ERR_INVALID_ARG;
        }

        esp_err_t ret = ESP_OK;
        for (auto handle : handles) {
            const esp_err_t err = removeDevice(handle);
            if (err != ESP_OK && err != ESP_ERR_INVALID_STATE && err != ESP_ERR_INVALID_ARG && ret == ESP_OK) {
                ret = err;
            }
        }
        return ret;
    }

    esp_err_t I2C::reset() {
        return i2c_master_bus_reset(m_i2c_mst_handle);
    }
} // namespace i2cbus
