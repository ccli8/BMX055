/*
 * mbed library program
 *  BMX055 Small, versatile 9-axis sensor module
 *  by Bosch Sensortec
 *
 * Copyright (c) 2018,'19 Kenji Arai / JH1PJL
 *  http://www.page.sannet.ne.jp/kenjia/index.html
 *  http://mbed.org/users/kenjiArai/
 *      Started:    October   24th, 2018
 *      Revised:    March      3rd, 2019
 */
/*
 *---------------- REFERENCE ---------------------------------------------------
 * Original Information
 *  https://www.bosch-sensortec.com/bst/products/all_products/bmx055
 *  Data Sheet  BST-BMX055-DS000-02 Rev.1.1 November 7th, 2014
 *  Sample software
 *      BMX055 - Combination of bma2x2 + bmg160 + bmm050 APIs
 *          https://github.com/BoschSensortec/BMA2x2_driver
 *          https://github.com/BoschSensortec/BMG160_driver
 *          https://github.com/BoschSensortec/BMM050_driver
 * Aplied Board
 *      AE-BMX055 Module
 *          http://akizukidenshi.com/catalog/g/gK-13010/
 */

// NO Interrupt functions are supported due to no pin on AE-BMX055 Module
// Only supprt normal mode (No sleep and/or standby mode)

#ifndef BMX055_H
#define BMX055_H

#include "mbed.h"

#define AKIZUKI_BOARD

//  BMX055
//  Automatic detection for the address!!
//  AE-BMX055 board default setting (All jumpers are open)
#define BMX055_GYR_CHIP_ADDR      (0x68 << 1)
#define BMX055_ACC_CHIP_ADDR      (0x18 << 1)
#define BMX055_MAG_CHIP_ADDR      (0x10 << 1)

//  ID's
#define I_AM_BMX055_ACC         0xFA    // ACC ID
#define I_AM_BMX055_GYR         0x0F    // GYR ID
#define I_AM_BMX055_MAG         0x40    // MAG ID


////////////// PARAMETER DEFINITION ///////////////////////
//  ACC full scale
#define ACC_2G      3
#define ACC_4G      5
#define ACC_8G      8
#define ACC_16G     12

// Support only "filtered mode"
//  ACC Bandwidth (BW = ODR/2 (ODR = Output Data Rate))
#define ACC_BW7R81Hz    8   // 64 ms
#define ACC_BW15R63Hz   9   // 32 ms
#define ACC_BW31R25Hz   10  // 16 ms
#define ACC_BW62R5Hz    11  // 8 ms
#define ACC_BW125Hz     12  // 4 ms
#define ACC_BW250Hz     13  // 2 ms
#define ACC_BW500Hz     14  // 1 ms
#define ACC_BW1kHz      15  // 0.5 ms

// Gyro Sampling (Data per Second)
#define GYR_2000DPS     0   //  full scal +/- 2000 Deg/s(61.0 mDeg/sec/LSB)
#define GYR_1000DPS     1   //  +/- 1000 Deg/s(30.5 mDeg/sec/LSB)
#define GYR_500DPS      2   //  +/- 500 Deg/s(15.3 mDeg/sec/LSB)
#define GYR_250DPS      3   //  +/- 250 Deg/s(7.6 mDeg/sec/LSB)
#define GYR_125DPS      4   //  +/- 125 Deg/s(3.8 mDeg/sec/LSB)

// Gyro Bandwidth
#define GYR_2000Hz523Hz 0   // 2000 Hz ODR and unfiltered (BW(bandwidth) 523Hz)
#define GYR_2000Hz230Hz 1   // 2000 Hz ODR 230 Hz BW
#define GYR_1000Hz116Hz 2   // 1000 Hz ODR 116 Hz BW
#define GYR_400Hz47Hz   3   // 400 Hz ODR 47 Hz BW
#define GYR_200Hz23Hz   4   // 200 Hz ODR 23 Hz BW
#define GYR_100Hz12Hz   5   // 100 Hz ODR 12 Hz BW
#define GYR_200Hz64Hz   6   // 200 Hz ODR 64 Hz BW
#define GYR_100Hz32Hz   7   // 100 Hz ODR 32 Hz BW

// MAG
#define MAG_ODR10Hz     0   // 10 Hz ODR(default)
#define MAG_ODR2Hz      1   // 2 Hz ODR
#define MAG_ODR6Hz      2   // 6 Hz ODR
#define MAG_ODR8Hz      3   // 8 Hz ODR
#define MAG_ODR15Hz     4   // 15 Hz ODR
#define MAG_ODR20Hz     5   // 20 Hz ODR
#define MAG_ODR25Hz     6   // 25 Hz ODR
#define MAG_ODR30Hz     7   // 30 Hz ODR

////////////// DATA TYPE DEFINITION ///////////////////////
typedef struct {
    // ACC
    uint8_t acc_fs;     // Accelerometer full scale range
    uint8_t acc_bw;     // Accelerometer filtered bandwidth
    // GYR
    uint8_t gyr_fs;     // Gyroscope full scale range
    uint8_t gyr_bw;     // Gyroscope filtered bandwidth
    // MAG
    uint8_t mag_odr;    // Magnetometer Output Data Rate 
} BMX055_TypeDef;

////////////// DEFAULT SETTING ////////////////////////////
// Standard parameter for easy set-up
const BMX055_TypeDef bmx055_std_paramtr = {
    // ACC
    ACC_2G,
    ACC_BW250Hz,
    // GYR
    GYR_125DPS,
    GYR_100Hz32Hz,
    // MAG
    MAG_ODR10Hz
};

////////////// DATA TYPE DEFINITION ///////////////////////
typedef struct {
    uint8_t  acc_addr;
    uint8_t  mag_addr;
    uint8_t  gyr_addr;
} BMX055_ADDR_INF_TypeDef;

typedef struct {
    uint8_t  acc_id;
    uint8_t  mag_id;
    uint8_t  gyr_id;
} BMX055_ID_INF_TypeDef;

typedef struct {
    float x;
    float y;
    float z;
} BMX055_ACCEL_TypeDef;

typedef struct {
    float x;
    float y;
    float z;
} BMX055_GRAVITY_TypeDef;

typedef struct {
    float x;
    float y;
    float z;
} BMX055_GYRO_TypeDef;

typedef struct {
    float x;
    float y;
    float z;
} BMX055_MAGNET_TypeDef;

/** BMX055 Small, versatile 9-axis sensor module by Bosch Sensortec
 * @code
 * #include    "mbed.h"
 * #include    "BMX055.h"
 *
 * Serial pc(USBTX,USBRX);
 * I2C    (I2C_SDA, I2C_SCL);
 * BMX055 imu(i2c);
 *
 * const BMX055_TypeDef bmx055_my_parameters = {
 *   // ACC
 *   ACC_2G,
 *   ACC_BW250Hz,
 *   // GYR
 *   GYR_125DPS,
 *   GYR_200Hz23Hz,
 *   // MAG
 *   MAG_ODR10Hz
 * };
 *
 * int main() {
 *     BMX055_ACCEL_TypeDef  acc;
 *     BMX055_GYRO_TypeDef   gyr;
 *     BMX055_MAGNET_TypeDef mag;
 *
 *     if (imu.imu.chip_ready() == 0){
 *         pc.printf("Bosch BMX055 is NOT avirable!!\r\n");
 *     }
 *     imu.set_parameter(&bmx055_my_parameters);
 *     while(1) {
 *         imu.get_accel(&acc);
 *         pc.printf("//ACC: x=%+3.2f y=%+3.2f z=%+3.2f //",
 *                    acc.x, acc.y, acc.z);
 *         imu.get_gyro(&gyr);
 *         pc.printf("GYR: x=%+3.2f y=%+3.2f z=%+3.2f //",
 *                    gyr.x, gyr.y, gyr.z);
 *         imu.get_magnet(&mag);
 *         pc.printf("MAG: x=%+3.2f y=%+3.2f z=%+3.2f , passed %u sec\r\n",
 *                    mag.x, mag.y, mag.z, n++);
 *         wait(0.5f);
 *     }
 * }
 * @endcode
 */

class BMX055
{
public:
    /** Configure data pin
      * @param data SDA and SCL pins
      * @param Other parameters are set automatically
      */
    BMX055(PinName p_sda, PinName p_scl);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @param Other parameters are set automatically
      */
    BMX055(I2C& p_i2c);

    /** Get accel data
     * @param float type of 3D data address
     */
    void get_accel(BMX055_ACCEL_TypeDef *acc);

    /** Get gyroscope data
     * @param float type of 3D data address
     */
    void get_gyro(BMX055_GYRO_TypeDef *gyr);

    /** Get magnet data
     * @param float type of 3D data address
     */
    void get_magnet(BMX055_MAGNET_TypeDef *mag);

    /** Get Chip temperature data both Acc & Gyro
     * @param none
     * @return temperature data
     */
    float get_chip_temperature(void);

    /** Read BMX055 ID information
      * @param ID information address
      * @return none
      */
    void read_id_inf(BMX055_ID_INF_TypeDef *id);

    /** Check chip is avairable or not
      * @param none
      * @return OK = true, NG = false;
      */
    bool chip_ready(void);

    /** Check chip is avairable or not
      * @param configration parameter
      * @return none
      */
    void set_parameter(const BMX055_TypeDef *bmx055_parameter);

    /** Set I2C clock frequency
      * @param freq.
      * @return none
      */
    void frequency(int hz);

    /** Read register
      * @param register's address
      * @return register data
      */
    uint8_t read_reg(uint8_t addr);

    /** Write register
      * @param register's address
      * @param data
      * @return register data
      */
    uint8_t write_reg(uint8_t addr, uint8_t data);

protected:
    void initialize(void);
    void check_id(void);
    void set_parameters_to_regs(void);

    I2C *_i2c_p;
    I2C &_i2c;

private:
    char     dt[16];      // working buffer
    uint8_t  chip_addr;
    BMX055_ADDR_INF_TypeDef inf_addr;
    BMX055_ID_INF_TypeDef inf_id;

    uint8_t  ready_flag;
    
    
    
    BMX055_TypeDef bmx055_parameters;
    uint8_t  acc_id;
    uint8_t  mag_id;
    uint8_t  gyr_id;
};

#endif      // BMX055_H