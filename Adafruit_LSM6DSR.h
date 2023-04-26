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
#include <stdio.h>
#define LSM6DSR_CHIP_ID 0x6B ///< LSM6DSL default device id from WHOAMI

#define LSM6DSR_MASTER_CONFIG 0x1A ///< I2C Master config

// registers
#define LSM6DS_EMB_FUNC_EN_A 0x04
#define LSM6DS_EMB_FUNC_INT1 0x0A
#define LSM6DS_EMB_FUNC_STATUS 0x12
#define LSM6DS_PAGE_RW 0x17

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

  /*void singleTapDetect()
 {
  bool success = false;
     Adafruit_BusIO_Register register1 = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_FUNC_CFG_ACCESS);
     Adafruit_BusIO_RegisterBits funcCfgAccess = Adafruit_BusIO_RegisterBits(&funcCfgAccessRegister, 1, 7);
     success = funcCfgAccess.write(1);
     if (!success)
     {
       return false;
     }

   uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG0);// preserve interrupt existing configuration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG0, temp | 0x0E); // enable tap detection on X, Y, Z




    temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG1);       // preserve interrupt existing configuration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG1, temp | 0x09); //  set x-axis threshold and axes priority
    temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG2);       // preserve interrupt existing configuration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG2, temp | 0x89); // set y-axis threshold and enable interrupt
    temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_THS_6D);     // preserve interrupt existing configuration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_THS_6D, temp | 0x09);   // set z-axis threshold
    temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_INT_DUR2);       // preserve interrupt existing configuration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_INT_DUR2, temp | 0x06); // set quiet and shock time windows
   temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG);         // preserve interrupt existing routing
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG, temp | 0x40);  // add single tap interrupt routing to INT1
 }*/

  bool embeddedFunctionregisterAccess(bool enable)
  {
    Adafruit_BusIO_Register funcCfgAccessRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_FUNC_CFG_ACCESS);
    Adafruit_BusIO_RegisterBits funcCfgAccess = Adafruit_BusIO_RegisterBits(&funcCfgAccessRegister, 1, 7);
    if (enable)
    {
      // Embedded functions registers are accessible when FUNC_CFG_EN is set to '1' in FUNC_CFG_ACCESS (01h).
      // FUNC_CFG_ACCESS (01h) bit 7 FUNC_CFG_ACCESS auf 1
      return funcCfgAccess.write(1);
    }
    else
    {
      // Embedded functions registers are accessible when FUNC_CFG_EN is set to '1' in FUNC_CFG_ACCESS (01h).
      // disable access here again
      return funcCfgAccess.write(0);
    }
  }

  bool enableTiltDetection()
  {
    bool success = false;
    // Embedded functions registers are accessible when FUNC_CFG_EN is set to '1' in FUNC_CFG_ACCESS (01h).
    // FUNC_CFG_ACCESS (01h) bit 7 FUNC_CFG_ACCESS auf 1
    Adafruit_BusIO_Register funcCfgAccessRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_FUNC_CFG_ACCESS);
    Adafruit_BusIO_RegisterBits funcCfgAccess = Adafruit_BusIO_RegisterBits(&funcCfgAccessRegister, 1, 7);
    success = funcCfgAccess.write(1);
    if (!success)
    {
      return false;
    }
    // EMB_FUNC_EN_A (04h)
    //
    Adafruit_BusIO_Register embFuncEnableRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_EMB_FUNC_EN_A);
    Adafruit_BusIO_RegisterBits tiltBitEnable = Adafruit_BusIO_RegisterBits(&embFuncEnableRegister, 1, 4);
    success = tiltBitEnable.write(1);
    if (!success)
    {
      return false;
    }

    // EMB_FUNC_INT1 (0Ah) Routing of significant motion event on INT1
    // INT1_SIG_MOT auf 1 setzen (bit 5)
    Adafruit_BusIO_Register embFuncINT1Register = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_EMB_FUNC_INT1);
    Adafruit_BusIO_RegisterBits INT1SignificantMotionBitRouting = Adafruit_BusIO_RegisterBits(&embFuncINT1Register, 1, 4);
    success = INT1SignificantMotionBitRouting.write(1);
    if (!success)
    {
      return false;
    }

    // enable latched mode
    /*Adafruit_BusIO_Register pageRWRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_PAGE_RW);
    success = pageRWRegister.write(0x80);
    if (!success)
    {
      return false;
    }*/
    // Embedded functions registers are accessible when FUNC_CFG_EN is set to '1' in FUNC_CFG_ACCESS (01h).
    // disable access here again
    success = funcCfgAccess.write(0);
    if (!success)
    {
      return false;
    }

    // register: MD1_CFG (5Eh)
    // INT1_EMB_FUNC (bit 1)
    Adafruit_BusIO_Register MD1CFGRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_MD1_CFG);
    Adafruit_BusIO_RegisterBits INT1EmbFunctionRoutingBit = Adafruit_BusIO_RegisterBits(&MD1CFGRegister, 1, 1);
    success = INT1EmbFunctionRoutingBit.write(1);
    if (!success)
    {
      return false;
    }

    // turn on acc with correct settings, ODR_XL = 26 Hz, FS_XL = ±2 g
    Adafruit_BusIO_Register CTRL1XLRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL1_XL);
    success = CTRL1XLRegister.write(0x20);
    if (!success)
    {
      return false;
    }

    return true;
  }

  bool checkTiltDetection()
  {
    embeddedFunctionregisterAccess(true);

    // EMB_FUNC_STATUS 0x12
    //  bit 5
    Adafruit_BusIO_Register embFuncStatusRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_EMB_FUNC_STATUS);
    uint8_t detected[1];

    bool success = embFuncStatusRegister.read(detected, 1);
    embeddedFunctionregisterAccess(false);

    if (!success)
    {
      Serial.println("error smd status read");
      return false;
    }
    // Adafruit_BusIO_RegisterBits isSigMotionDetectedBit = Adafruit_BusIO_RegisterBits(&embFuncStatusRegister, 1, 5);
    // uint32_t detected = isSigMotionDetectedBit.read();
    Serial.print("emd function status register: ");
    Serial.println(detected[0]);
    return true;
  }

  bool checkSMD()
  {
    // EMB_FUNC_STATUS 0x12
    //  bit 5
    Adafruit_BusIO_Register embFuncStatusRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_EMB_FUNC_STATUS);
    uint8_t detected[1];

    bool success = embFuncStatusRegister.read(detected, 1);
    if (!success)
    {
      Serial.println("error smd status read");
      return false;
    }
    // Adafruit_BusIO_RegisterBits isSigMotionDetectedBit = Adafruit_BusIO_RegisterBits(&embFuncStatusRegister, 1, 5);
    // uint32_t detected = isSigMotionDetectedBit.read();
    Serial.print("smd detected: ");
    Serial.println(detected[0]);
    return true;
  }

  bool enableSignificantMotionDetection()
  {
    bool success = false;
    // Embedded functions registers are accessible when FUNC_CFG_EN is set to '1' in FUNC_CFG_ACCESS (01h).
    // FUNC_CFG_ACCESS (01h) bit 7 FUNC_CFG_ACCESS auf 1
    Adafruit_BusIO_Register funcCfgAccessRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_FUNC_CFG_ACCESS);
    Adafruit_BusIO_RegisterBits funcCfgAccess = Adafruit_BusIO_RegisterBits(&funcCfgAccessRegister, 1, 7);
    success = funcCfgAccess.write(1);
    if (!success)
    {
      return false;
    }
    // EMB_FUNC_EN_A (04h)
    // SIGN_MOTION_EN auf 1 setzen (bit 5)
    Adafruit_BusIO_Register embFuncEnableRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_EMB_FUNC_EN_A);
    Adafruit_BusIO_RegisterBits significantMotionBitEnable = Adafruit_BusIO_RegisterBits(&embFuncEnableRegister, 1, 5);
    success = significantMotionBitEnable.write(1);
    if (!success)
    {
      return false;
    }

    // EMB_FUNC_INT1 (0Ah) Routing of significant motion event on INT1
    // INT1_SIG_MOT auf 1 setzen (bit 5)
    Adafruit_BusIO_Register embFuncINT1Register = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_EMB_FUNC_INT1);
    Adafruit_BusIO_RegisterBits INT1SignificantMotionBitRouting = Adafruit_BusIO_RegisterBits(&embFuncINT1Register, 1, 5);
    success = INT1SignificantMotionBitRouting.write(1);
    if (!success)
    {
      return false;
    }

    // enable latched mode
    Adafruit_BusIO_Register pageRWRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_PAGE_RW);
    success = pageRWRegister.write(0x80);
    if (!success)
    {
      return false;
    }
    // Embedded functions registers are accessible when FUNC_CFG_EN is set to '1' in FUNC_CFG_ACCESS (01h).
    // disable access here again
    // success = funcCfgAccess.write(0);
    // if (!success)
    //{
    // return false;
    //}

    // register: MD1_CFG (5Eh)
    // INT1_EMB_FUNC (bit 1)
    Adafruit_BusIO_Register MD1CFGRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_MD1_CFG);
    Adafruit_BusIO_RegisterBits INT1EmbFunctionRoutingBit = Adafruit_BusIO_RegisterBits(&MD1CFGRegister, 1, 1);
    success = INT1EmbFunctionRoutingBit.write(1);
    if (!success)
    {
      return false;
    }

    // turn on acc with correct settings, ODR_XL = 26 Hz, FS_XL = ±2 g
    Adafruit_BusIO_Register CTRL1XLRegister = Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL1_XL);
    success = CTRL1XLRegister.write(0x20);
    if (!success)
    {
      return false;
    }

    return true;
  }

  // reads temp, acc, and gyro and saves it into member variables
  void read()
  {
    _read();
  }

  void readAll(uint8_t *buffer, uint16_t bufferSize)
  {
    _read();

    snprintf((char *)buffer, 30, "[101;%6.4f,%6.4f,%6.4f]", accX, accY, accZ);
    // buffer[0] = accX;
    // buffer[1] = 3;
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
