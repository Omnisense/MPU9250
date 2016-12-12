/* 
 * @file    MPU9250.h
 * @brief   Device driver - MPU9250 9-axis motion sensor driver
 * @author  David Bartlett
 * @version 1.0
 * Copyright (c) 2016 Omnisense Limited (www.omnisense.co.uk)
 *
 * Licensed under the Apache Licence, Version 2.0 (the "Licence");
 * you may not use this file except in compliance with the Licence.
 * You may obtain a copy of the Licence at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the Licence is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the Licence for the specific language governing permissions and
 * limitations under the Licence.
 *
 * This driver library builds on the work done by Kris Winer
 * https://developer.mbed.org/users/onehorse/code/MPU9250AHRS/
 */
 
#ifndef MPU9250_H
#define MPU9250_H
 
#include "mbed.h"
#include "math.h"
 
//  Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
//  mbed uses the eight-bit device address, so shift seven-bit addresses left by one!
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68<<1  // Device address when ADO = 0
#endif  

//  Constant defines
#define MPU_H_RESET 0x80
#define MPU_SLEEP 0x40
#define MPU_CYCLE 0x20
#define MPU_GYRO_STANDBY 0x10
#define MPU_TEMP_DIS 0x08
#define MPU_ACCEL_DIS 0x38
#define MPU_GYRO_DIS 0x07
#define MPU_ACCEL_FBCHOICE 0x08
#define MPU_INT_ACTL 0x80       // active low
#define MPU_INT_OD 0x40         // open drain
#define MPU_LATCH_INT_EN 0x20   // latched until status cleared
#define MPU_ANYRD_2CLEAR 0x10   // any read clears int status
#define MPU_ACTL_FSYNC 0x08     // FSYNC as INT is active low
#define MPU_FSYNC_INT_MODE 0x04 // use FSYNC as interrupt
#define MPU_BYPASS_EN 0x02      // slave I2C in bypass mode
#define MPU_INT_WOM_EN 0x40     // enable Wake on Motion INT
#define MPU_FIFO_OVFL_INT_EN 0x10   // enable FIFO overflow INT
#define MPU_FSYNC_INT_EN 0x08   // enable FSYNC to INT pin
#define MPU_DRDY_INT_EN 0x01    // data ready interrupt (RAW)
#define MPU_FCHOICE 0x03        // set to disable DLPF

/**
 *  @class MPU9250
 *  @brief API abstraction for the MPU9250 9-axis MEMS motion sensor IC
 */ 
class MPU9250 {
 
public:

    /**
     *  @enum REGISTER
     *  @brief The device register map
     */
    enum REGISTER
    {
        SELF_TEST_X_GYRO    = 0x00,                 
        SELF_TEST_Y_GYRO    = 0x01,                                                                         
        SELF_TEST_Z_GYRO    = 0x02,
        SELF_TEST_X_ACCEL   = 0x0D,
        SELF_TEST_Y_ACCEL   = 0x0E,   
        SELF_TEST_Z_ACCEL   = 0x0F,
        XG_OFFSET_H         = 0x13,     // User-defined trim values for gyroscope
        XG_OFFSET_L         = 0x14,
        YG_OFFSET_H         = 0x15,
        YG_OFFSET_L         = 0x16,
        ZG_OFFSET_H         = 0x17,
        ZG_OFFSET_L         = 0x18,
        SMPLRT_DIV          = 0x19,
        CONFIG              = 0x1A,
        GYRO_CONFIG         = 0x1B,
        ACCEL_CONFIG        = 0x1C,
        ACCEL_CONFIG2       = 0x1D,
        LP_ACCEL_ODR        = 0x1E,  
        WOM_THR             = 0x1F,  
        FIFO_EN             = 0x23,
        I2C_MST_CTRL        = 0x24,  
        I2C_SLV0_ADDR       = 0x25,
        I2C_SLV0_REG        = 0x26,
        I2C_SLV0_CTRL       = 0x27,
        I2C_SLV1_ADDR       = 0x28,
        I2C_SLV1_REG        = 0x29,
        I2C_SLV1_CTRL       = 0x2A,
        I2C_SLV2_ADDR       = 0x2B,
        I2C_SLV2_REG        = 0x2C,
        I2C_SLV2_CTRL       = 0x2D,
        I2C_SLV3_ADDR       = 0x2E,
        I2C_SLV3_REG        = 0x2F,
        I2C_SLV3_CTRL       = 0x30,
        I2C_SLV4_ADDR       = 0x31,
        I2C_SLV4_REG        = 0x32,
        I2C_SLV4_DO         = 0x33,
        I2C_SLV4_CTRL       = 0x34,
        I2C_SLV4_DI         = 0x35,
        I2C_MST_STATUS      = 0x36,
        INT_PIN_CFG         = 0x37,
        INT_ENABLE          = 0x38,
        INT_STATUS          = 0x3A,
        ACCEL_XOUT_H        = 0x3B,
        ACCEL_XOUT_L        = 0x3C,
        ACCEL_YOUT_H        = 0x3D,
        ACCEL_YOUT_L        = 0x3E,
        ACCEL_ZOUT_H        = 0x3F,
        ACCEL_ZOUT_L        = 0x40,
        TEMP_OUT_H          = 0x41,
        TEMP_OUT_L          = 0x42,
        GYRO_XOUT_H         = 0x43,
        GYRO_XOUT_L         = 0x44,
        GYRO_YOUT_H         = 0x45,
        GYRO_YOUT_L         = 0x46,
        GYRO_ZOUT_H         = 0x47,
        GYRO_ZOUT_L         = 0x48,
        EXT_SENS_DATA_00    = 0x49,
        EXT_SENS_DATA_01    = 0x4A,
        EXT_SENS_DATA_02    = 0x4B,
        EXT_SENS_DATA_03    = 0x4C,
        EXT_SENS_DATA_04    = 0x4D,
        EXT_SENS_DATA_05    = 0x4E,
        EXT_SENS_DATA_06    = 0x4F,
        EXT_SENS_DATA_07    = 0x50,
        EXT_SENS_DATA_08    = 0x51,
        EXT_SENS_DATA_09    = 0x52,
        EXT_SENS_DATA_10    = 0x53,
        EXT_SENS_DATA_11    = 0x54,
        EXT_SENS_DATA_12    = 0x55,
        EXT_SENS_DATA_13    = 0x56,
        EXT_SENS_DATA_14    = 0x57,
        EXT_SENS_DATA_15    = 0x58,
        EXT_SENS_DATA_16    = 0x59,
        EXT_SENS_DATA_17    = 0x5A,
        EXT_SENS_DATA_18    = 0x5B,
        EXT_SENS_DATA_19    = 0x5C,
        EXT_SENS_DATA_20    = 0x5D,
        EXT_SENS_DATA_21    = 0x5E,
        EXT_SENS_DATA_22    = 0x5F,
        EXT_SENS_DATA_23    = 0x60,
        I2C_SLV0_DO         = 0x63,
        I2C_SLV1_DO         = 0x64,
        I2C_SLV2_DO         = 0x65,
        I2C_SLV3_DO         = 0x66,
        I2C_MST_DELAY_CTRL  = 0x67,
        SIGNAL_PATH_RESET   = 0x68,
        MOT_DETECT_CTRL     = 0x69,
        USER_CTRL           = 0x6A,     // Bit 7 enable DMP, bit 3 reset DMP
        PWR_MGMT_1          = 0x6B,     // Device defaults to the SLEEP mode
        PWR_MGMT_2          = 0x6C,
        FIFO_COUNTH         = 0x72,
        FIFO_COUNTL         = 0x73,
        FIFO_R_W            = 0x74,
        WHO_AM_I_MPU9250    = 0x75,     // Should return 0x71
        XA_OFFSET_H         = 0x77,
        XA_OFFSET_L         = 0x78,
        YA_OFFSET_H         = 0x7A,
        YA_OFFSET_L         = 0x7B,
        ZA_OFFSET_H         = 0x7D,
        ZA_OFFSET_L         = 0x7E
    };
    
    /**
     *  @enum AK8963_REGISTER
     *  @brief The magnetometer register map
     */
    enum AK8963_REGISTER
    {
        AK8963_ADDRESS      = 0x0C<<1,
        AK8963_WHO_AM_I     = 0x00,     // should return 0x48
        AK8963_INFO         = 0x01,
        AK8963_ST1          = 0x02,     // data ready status bit 0
        AK8963_XOUT_L       = 0x03,     // data
        AK8963_XOUT_H       = 0x04,
        AK8963_YOUT_L       = 0x05,
        AK8963_YOUT_H       = 0x06,
        AK8963_ZOUT_L       = 0x07,
        AK8963_ZOUT_H       = 0x08,
        AK8963_ST2          = 0x09,     // Data overflow bit 3 and data read error status bit 2
        AK8963_CNTL         = 0x0A,     // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
        AK8963_ASTC         = 0x0C,     // Self test control
        AK8963_I2CDIS       = 0x0F,     // I2C disable
        AK8963_ASAX         = 0x10,     // Fuse ROM x-axis sensitivity adjustment value
        AK8963_ASAY         = 0x11,     // Fuse ROM y-axis sensitivity adjustment value
        AK8963_ASAZ         = 0x12      // Fuse ROM z-axis sensitivity adjustment value
    };

    /**
     *  @enum ASCALE    
     *  @brief Accelerometer scale factor mapping
     */
    enum ASCALE {
        AFS_2G = 0x00,
        AFS_4G = 0x08,
        AFS_8G = 0x10,
        AFS_16G = 0x18
    };

    /**
    *   @enum ACCEL_LPF
    *   @brief Filter BW for accelerometer
    */
    enum ACCEL_LPF {
        ACCEL_BW_420 = 0x07,
        ACCEL_BW_218 = 0x01,
        ACCEL_BW_99 = 0x02,
        ACCEL_BW_45 = 0x03,
        ACCEL_BW_21 = 0x04,
        ACCEL_BW_10 = 0x05,
        ACCEL_BW_5 = 0x06,
        ACCEL_BW_NONE = 0x00
    };
    
    /**
    *   @enum DLPF
    *   @brief Filter BW for accelerometer
    */
    enum DLPF {
        DLPF_250 = 0x00,
        DLPF_184 = 0x01,
        DLPF_92 = 0x02,
        DLPF_41 = 0x03,
        DLPF_20 = 0x04,
        DLPF_10 = 0x05,
        DLPF_5 = 0x06,
        DLPF_3600 = 0x07
    };
    
    /**
    *   @enum ACCEL_LPDRATE
    *   @brief Accelerometer data rate in low power mode
    */
    enum ACCEL_LPDRATE {
        ACCEL_DR_00024 = 0x00,
        ACCEL_DR_00049 = 0x01,
        ACCEL_DR_00098 = 0x02,
        ACCEL_DR_00195 = 0x03,
        ACCEL_DR_00391 = 0x04,
        ACCEL_DR_00781 = 0x05,
        ACCEL_DR_01563 = 0x06,
        ACCEL_DR_03125 = 0x07,
        ACCEL_DR_0625 = 0x08,
        ACCEL_DR_125 = 0x09,
        ACCEL_DR_250 = 0x0A,
        ACCEL_DR_500 = 0x0B
    };
    
    /**
     *  @enum GSCALE    
     *  @brief Gyro scale factor mapping
     */
    enum GSCALE {
        GFS_250DPS = 0x00,
        GFS_500DPS = 0x08,
        GFS_1000DPS = 0x10,
        GFS_2000DPS = 0x18
    };

    /**
     *  @enum MSCALE    
     *  @brief Magnetometer scale factor mapping
     */
    enum MSCALE {
        MFS_14BITS = 0x00,     // 0.6 mG per LSB
        MFS_16BITS = 0x10      // 0.15 mG per LSB
    };

    /**
     *  @enum MMODE    
     *  @brief Magnetometer measurement mode
     */
    enum MMODE {
        MFS_PWRNDN = 0x00,     // Power down
        MFS_SINGLE = 0x01,      // Single shot mode
        MFS_CONT1 = 0x02,
        MFS_CONT2 = 0x06,
        MFS_EXTTRIG = 0x04,
        MFS_STEST = 0x08
    };
    
    /**
    * @enum CLKSOURCE
    * @brief Clock source selection for MPU9250
    */
    enum CLKSOURCE {
        CLK_DISABLED = 0x07,
        CLK_INTERNAL = 0x00,
        CLK_AUTO = 0x01
    };
    
   /**
     *  @enum WHO_AM_I_VAL
     *  @brief Device ID's that this class is compatible with
     */ 
    enum WHO_AM_I_VAL
    { 
        I_AM_MPU9250 = 0x71,
    };

    /**
    *   @enum MEMS_MODE
    *   @brief MPU9250 settings by simple mode classes
    */
    enum MEMS_MODE {
        VLP_ACC     = 1,    // accelerometer only very low power
        LP_ACCMAG   = 2,    // low power, accelerometer + magnetometer
        HP_ALL      = 3,    // normal: accelerometer + gyro + magnetometer
        HPP_ALL     = 4     // performance: all sensors
    };
    
    /** Create the MPU9250 object
     *  @param i2c - A defined I2C object
     *  @param intr - A defined InterruptIn object pointer. Default NULL for polling mode
     */ 
    MPU9250(I2C &i2c, InterruptIn* intr = NULL);

    /** Test the Who am I register for valid ID
     *  @return Boolean true if valid device
     */
    bool testWhoAmI(void);

    /** Setup the MPU9250 for the desired operating mode
	 *  @opmode - 1-4 setting by classes
     *  @return status of command
     *
     *  The modes are as follows
     *  1 = very low power, only accelerometer
     *          7.81 Hz
     *  2 = low power, accelerometer and magnetometer only
     *          15.63 Hz accelerometer, single shot magnetometer
     *  3 = high power, accelerometer + gyro + magnetometer
     *          50 Hz, 50 Hz, ~10 Hz
     *  4 = performance mode, accelerometer, gyro, magnetometer
     *          ~250 Hz, 250 Hz, 50 Hz 
     *  
     *  Since the chip has a huge array of different operating modes, a few key settings as chosen
     */
    uint8_t setParameters(MEMS_MODE opmode, ASCALE accelfs, MSCALE magfs, GSCALE gyrofs);

    /** Read Accelerometer data from MPU9250
    *   @param destination - pointer to 3 integer vector into which 16 bit values are written
    *   @return nothing
    */
    void readAccelData(int16_t * destination);

    /** Read Magnetometer data from MPU9250/AK8963
    *   @param destination - pointer to 3 integer vector into which 16 bit values are written
    *   @return measurement status: 0 = good, 1 = no data, 2 = measurement error
    */
    uint8_t readMagData(int16_t * destination);
    
    /** Read Gyro data from MPU9250
    *   @param destination - pointer to 3 integer vector into which 16 bit values are written
    *   @return measurement status: 0 = good, 1 = disabled or error
    */
    uint8_t readGyroData(int16_t * destination);

private:

    I2C         			*_i2c;
    InterruptIn 			*_intr;
    int16_t                 _accelBias[3];
    uint8_t static const 	_i2c_addr = MPU9250_ADDRESS;
    uint8_t static const    _i2c_magaddr = AK8963_ADDRESS;
    MEMS_MODE               _opmode;
    MSCALE                  _magfs;
    
    /** Initialise the device
     *  Set to the power on reset conditions
     *  @return - status of command (0 = success)
     */
    uint8_t init(void);

    /** Write to a register
     *  @param reg - The register to be written
     *  @param data - The data to be written
     *  @param count - number of bytes to send, assumes 1 byte if not specified
     *  @return - status of command
     */
    uint8_t writeRegister(uint8_t reg, uint8_t* data, uint8_t count = 1);
    
    /** Read from a register
     *  @param reg - The register to read from
     *  @param data - buffer of data to be read
     *  @param count - number of bytes to send, assumes 1 byte if not specified
     *  @return - status of command
     */
    uint8_t readRegister(uint8_t reg, uint8_t* data, uint8_t count = 1);

    /** Write to a magnetometer register
     *  @param reg - The register to be written
     *  @param data - The data to be written
     *  @param count - number of bytes to send, assumes 1 byte if not specified
     *  @return - status of command
     */
    uint8_t writeMagRegister(uint8_t reg, uint8_t* data, uint8_t count = 1);
    
    /** Read from a magnetometer register
     *  @param reg - The register to read from
     *  @param data - buffer of data to be read
     *  @param count - number of bytes to send, assumes 1 byte if not specified
     *  @return - status of command
     */
    uint8_t readMagRegister(uint8_t reg, uint8_t* data, uint8_t count = 1);

};

#endif
