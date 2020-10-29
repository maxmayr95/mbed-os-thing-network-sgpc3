/*
 * Copyright (c) 2017-2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * TODO:
 * - Baseline store/restore
 * - proper init
 * - IAQ levels, getIAQLevel()
 */



#include <string.h>

#include "sensirion_ess.h"
DigitalOut led_red(LED_RED);
DigitalOut led_yellow(LED_YEL);
DigitalOut led_green(LED_GRN);

SensirionESS::SensirionESS(void *interface)
{
    i2c_connection = (I2C*)(interface);
    t = new Timer();
}

int SensirionESS::initSensors()
{

    if (measureRHTInt() != 0) {
        setError("Error communicating with SHTC1");
        return -1;
    }

    if (initSGP() != 0) {
        setError("Error communicating with SGPC3");
        return -2;
    }
    
    t->start();

    mInitialized = true;
    return 0;
}

//////////////////////////////////////////////////////////////////////////////
// SHT

int SensirionESS::measureRHT()
{
    if (!mInitialized) {
        setError("ESS not initialized");
        return -1;
    }

    measureRHTInt();
    return 0;
}

int SensirionESS::measureRHTInt()
{
    uint8_t cmd[CMD_LENGTH] = { 0x7C, 0xA2 };

    if (i2c_write(SHT_I2C_ADDR, cmd, CMD_LENGTH)) {
        setError("I2C write error");
        return -1;
    }

    wait_ms(SHT_MEASURE_DELAY);

    int ret = i2c_read(SHT_I2C_ADDR, mDataBuf, SHT_DATA_LENGTH);
    if (ret == -1) {
        setError("I2C read error");
        return -2;
    }

    // check CRC for both RH and T
    if (crc8(mDataBuf+0, 2) != mDataBuf[2] ||
        crc8(mDataBuf+3, 2) != mDataBuf[5]) {
        setError("CRC mismatch");
        return -3;
    }

    uint16_t val;
    val = (mDataBuf[0] << 8) + mDataBuf[1];
    mTemperature = -45 + 175 * (val / 65535.0);
    val = (mDataBuf[3] << 8) + mDataBuf[4];
    mHumidity = 100 * (val / 65535.0);

    return 0;
}

//////////////////////////////////////////////////////////////////////////////
// SGP

int SensirionESS::getProductType() const
{
    return mProductType;
}

int SensirionESS::getFeatureSetVersion() const
{
    return mFeatureSetVersion;
}

int SensirionESS::measureIAQ()
{
    if (!mInitialized) {
        setError("ESS not initialized");
        return -1;
    }

    // keep track of timing
    mSGPMeasurementTimestamp = t->read_ms();

    uint8_t cmd[CMD_LENGTH] = { 0x20, 0x08 };

    if (i2c_write(SGP_I2C_ADDR, cmd, CMD_LENGTH)) {
        setError("error in i2c_write");
        return -1;
    }

    wait_ms(SGP_MEASURE_DELAY);

    int ret = i2c_read(SGP_I2C_ADDR, mDataBuf, SGP_DATA_LENGTH);
    if (ret == -1) {
        setError("error in i2c_read");
        return -2;
    }

    if (crc8(mDataBuf, 2) != mDataBuf[2]) {
        setError("CRC mismatch");
        return -3;
    }

    // SGPC3 only has TVOC; SGP30 sends [eco2, tvoc]
    if (mProductType == PRODUCT_TYPE_SGP30) {
      mECO2 = (mDataBuf[0] << 8) | mDataBuf[1];
      if (crc8(mDataBuf+3, 2) != mDataBuf[5]) {
          setError("CRC mismatch");
          return -4;
      }
      mTVOC = (mDataBuf[3] << 8) | mDataBuf[4];
  } else {
      mTVOC = (mDataBuf[0] << 8) | mDataBuf[1];;
  }

    return 0;
}

int SensirionESS::readFeatureSetInt()
{
    uint8_t cmd[CMD_LENGTH] = { 0x20, 0x2f };

    if (i2c_write(SGP_I2C_ADDR, cmd, CMD_LENGTH)) {
        setError("error in i2c_write");
        return -1;
    }

    wait_ms(2);

    const uint8_t DATA_LEN = 3;
    uint8_t data[DATA_LEN] = { 0 };
    int ret = i2c_read(SGP_I2C_ADDR, data, DATA_LEN);
    if (ret == -1) {
        setError("I2C read error");
        return -3;
    }

    // check CRC
    if (crc8(data, 2) != data[2]) {
        setError("CRC mismatch");
        return -4;
    }

    // 0 = SGP30, 1 = SGPC3
    mProductType = (data[0] & 0xF0) >> 4;
    mFeatureSetVersion = data[1] & 0xFF;

    return 0;
}

int SensirionESS::initSGP()
{
    int ret = readFeatureSetInt();
    // default: SGP30
    SGP_INTERMEASURE_DELAY = SGP30_INTERMEASURE_DELAY;
    SGP_DATA_LENGTH = SGP30_DATA_LENGTH;
    uint8_t cmd[CMD_LENGTH] = { 0x20, 0x03 };

    if (mProductType == PRODUCT_TYPE_SGPC3) {
      SGP_INTERMEASURE_DELAY = SGPC3_INTERMEASURE_DELAY;
      cmd[1] = 0xae;
    }

    // run init air quality
    if (i2c_write(SGP_I2C_ADDR, cmd, CMD_LENGTH)) {
        setError("error in i2c_write");
        return -1;
    }
    wait_ms(10);

    return ret;
}

//////////////////////////////////////////////////////////////////////////////
// getter for values read earlier
bool SensirionESS::isInitialized()
{
    return mInitialized;
}

float SensirionESS::getTemperature() const
{
    return mTemperature;
}

float SensirionESS::getHumidity() const
{
    return mHumidity;
}

float SensirionESS::getTVOC() const
{
    return mTVOC;
}

float SensirionESS::getECO2() const
{
    return mECO2;
}

void SensirionESS::setLedRYG(int r, int y, int g)
{
    led_red = r;
    led_yellow = y;
    led_green = g;
}

//////////////////////////////////////////////////////////////////////////////
// error handling

inline void SensirionESS::setError(const char* error)
{
    strlcpy(mErrorBuf, error, ERROR_BUF_LENGTH);
}

const char* SensirionESS::getError() const
{
    return mErrorBuf;
}

//////////////////////////////////////////////////////////////////////////////
// helper

int SensirionESS::remainingWaitTimeMS()
{
    unsigned long deltaT = t->read_ms() - mSGPMeasurementTimestamp;
    if (deltaT > SGP_INTERMEASURE_DELAY) {
        // we're already late, don't wait any longer
        return 0;
    }
    return (SGP_INTERMEASURE_DELAY - deltaT);
}

int8_t SensirionESS::i2c_read(uint8_t address, uint8_t* data, uint16_t count)
{
    if (i2c_connection->read(address << 1, (char *) data, count) != 0)
        return -1;
    return 0;
}

int8_t SensirionESS::i2c_write(uint8_t address, const uint8_t* data, uint16_t count)
{
    if (i2c_connection->write(address << 1, (char *) data, count) != 0)
        return -1;
    return 0;
}

uint8_t SensirionESS::crc8(const uint8_t* data, uint8_t len)
{
  // adapted from SHT21 sample code from http://www.sensirion.com/en/products/humidity-temperature/download-center/

  uint8_t crc = 0xff;
  uint8_t byteCtr;
  for (byteCtr = 0; byteCtr < len; ++byteCtr) {
    crc ^= (data[byteCtr]);
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}
