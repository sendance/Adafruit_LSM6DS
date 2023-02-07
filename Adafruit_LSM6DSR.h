/*!
 *  @file Adafruit_LSM6DSL.h
 *
 * 	I2C Driver for the Adafruit LSM6DSL 6-DoF Accelerometer and Gyroscope
 *library
 *
 * 	This is a library for the Adafruit LSM6DSL breakout:
 * 	https://www.adafruit.com/
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_LSM6DSR_H
#define _ADAFRUIT_LSM6DSR_H

#include "Adafruit_LSM6DS.h"

#include <ostream>
#include <sstream>
#include <iomanip>
#include <utility>

#define LSM6DSR_CHIP_ID 0x6B ///< LSM6DSL default device id from WHOAMI

#define LSM6DSR_MASTER_CONFIG 0x1A ///< I2C Master config

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LSM6DS3TRC
 */
class Adafruit_LSM6DSR : public Adafruit_LSM6DS
{
public:
  Adafruit_LSM6DSR();
  ~Adafruit_LSM6DSR(){};

  void enableI2CMasterPullups(bool enable_pullups);
  void enablePedometer(bool enable);

  std::string const &readAll()
  {
    _read();

    std::ostringstream outputString;
    outputString << std::fixed;
    outputString << std::setprecision(4); // maybe set this precision to according resolution of sensor

    outputString << "[" << millis() << ";";
    outputString << accX << "," << accY << "," << accZ << ",";
    outputString << std::setprecision(3); // gyro values have less precision, because of different conversion
    outputString << gyroX << "," << gyroY << "," << gyroZ << "]";

    return std::move(outputString.str());
  }

private:
  bool
  _init(int32_t sensor_id) override;

  // this is the same as in base class, just the conversion differs (acc to mg, gyro to mdeg)
  void _read(void) override
  {
    // get raw readings
    Adafruit_BusIO_Register data_reg = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_OUT_TEMP_L, 14);

    uint8_t buffer[14];
    data_reg.read(buffer, 14);

    rawTemp = buffer[1] << 8 | buffer[0];
    temperature = (rawTemp / temperature_sensitivity) + 25.0;

    rawGyroX = buffer[3] << 8 | buffer[2];
    rawGyroY = buffer[5] << 8 | buffer[4];
    rawGyroZ = buffer[7] << 8 | buffer[6];

    rawAccX = buffer[9] << 8 | buffer[8];
    rawAccY = buffer[11] << 8 | buffer[10];
    rawAccZ = buffer[13] << 8 | buffer[12];

    float gyro_scale = 1; // range is in milli-dps per bit!
    switch (gyroRangeBuffered)
    {
    case ISM330DHCX_GYRO_RANGE_4000_DPS:
      gyro_scale = 140.0;
      break;
    case LSM6DS_GYRO_RANGE_2000_DPS:
      gyro_scale = 70.0;
      break;
    case LSM6DS_GYRO_RANGE_1000_DPS:
      gyro_scale = 35.0;
      break;
    case LSM6DS_GYRO_RANGE_500_DPS:
      gyro_scale = 17.50;
      break;
    case LSM6DS_GYRO_RANGE_250_DPS:
      gyro_scale = 8.75;
      break;
    case LSM6DS_GYRO_RANGE_125_DPS:
      gyro_scale = 4.375;
      break;
    }

    gyroX = rawGyroX * gyro_scale;
    gyroY = rawGyroY * gyro_scale;
    gyroZ = rawGyroZ * gyro_scale;

    float accel_scale = 1; // range is in milli-g per bit!
    switch (accelRangeBuffered)
    {
    case LSM6DS_ACCEL_RANGE_16_G:
      accel_scale = 0.488;
      break;
    case LSM6DS_ACCEL_RANGE_8_G:
      accel_scale = 0.244;
      break;
    case LSM6DS_ACCEL_RANGE_4_G:
      accel_scale = 0.122;
      break;
    case LSM6DS_ACCEL_RANGE_2_G:
      accel_scale = 0.061;
      break;
    }

    accX = rawAccX * accel_scale;
    accY = rawAccY * accel_scale;
    accZ = rawAccZ * accel_scale;
  }
};

#endif
