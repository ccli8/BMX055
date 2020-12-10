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

#include "mbed.h"
#include "BMX055.h"

#define DEBUG 0

#if MBED_MAJOR_VERSION == 2
#define WAIT_MS(x)       wait_ms(x)
#elif  MBED_MAJOR_VERSION == 5
#define WAIT_MS(x)       Thread::wait(x)
#elif  MBED_MAJOR_VERSION == 6
#define WAIT_MS(x)       ThisThread::sleep_for(x##ms)
#else
#error "Running on Unknown OS"
#endif

BMX055::BMX055 (PinName p_sda, PinName p_scl):
    _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    bmx055_parameters = bmx055_std_paramtr;
    initialize ();
}

BMX055::BMX055 (I2C& p_i2c) :
    _i2c(p_i2c)
{
    initialize ();
}

/////////////// Set parameter /////////////////////////////
void BMX055::set_parameter(const BMX055_TypeDef *bmx055_parameter)
{
    bmx055_parameters = *bmx055_parameter;
    set_parameters_to_regs();
    
}

/////////////// Read data & normalize /////////////////////
void BMX055::get_accel(BMX055_ACCEL_TypeDef *acc)
{
    int16_t x,y,z;
    float factor = 2.0f;

    chip_addr = inf_addr.acc_addr;
    dt[0] = 0x02;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
#if DEBUG
    printf("Read ACC data-> ");
    for (uint32_t i = 0; i < 6; i++){
        printf("i=%d,dt=0x%02x, ", i, dt[i]);
    }
    printf(", all\r\n");
#endif
    x = dt[1] << 8 | (dt[0] & 0xf0);
    y = dt[3] << 8 | (dt[2] & 0xf0);
    z = dt[5] << 8 | (dt[4] & 0xf0);
    switch(bmx055_parameters.acc_fs){
        case ACC_2G:
            factor = 2.0f;
            break;
        case ACC_4G:
            factor = 4.0f;
            break;
        case ACC_8G:
            factor = 8.0f;
            break;
        case ACC_16G:
            factor = 16.0f;
            break;
        default:
            factor = 0;
            break;
    }
    acc->x = (double)x * factor / 2048.0f / 16.0f;
    acc->y = (double)y * factor / 2048.0f / 16.0f;
    acc->z = (double)z * factor / 2048.0f / 16.0f;
}

void BMX055::get_gyro(BMX055_GYRO_TypeDef *gyr)
{
    int16_t x,y,z;
    float factor = 2.0f;

    chip_addr = inf_addr.gyr_addr;
    dt[0] = 0x02;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
#if DEBUG
    printf("Read MAG data-> ");
    for (uint32_t i = 0; i < 6; i++){
        printf("i=%d,dt=0x%02x, ", i, dt[i]);
    }
    printf(", all\r\n");
#endif
    x = dt[1] << 8 | dt[0];
    y = dt[3] << 8 | dt[2];
    z = dt[5] << 8 | dt[4];
#if 0
    switch(bmx055_parameters.gyr_fs){
        case GYR_2000DPS:
            factor = 1998.0f;
            break;
        case GYR_1000DPS:
            factor = 999.0f;
            break;
        case GYR_500DPS:
            factor = 499.5f;
            break;
        case GYR_250DPS:
            factor = 249.75f;
            break;
        case GYR_125DPS:
            factor = 124.87f;
            break;
        default:
            factor = 0;
            break;
    }
    gyr->x = (double)x * factor / 32768.0f / 16.0f;
    gyr->y = (double)y * factor / 32768.0f / 16.0f;
    gyr->z = (double)z * factor / 32768.0f / 16.0f;
#else
    switch(bmx055_parameters.gyr_fs){
        case GYR_2000DPS:
            factor = 61.0f;
            break;
        case GYR_1000DPS:
            factor = 30.5f;
            break;
        case GYR_500DPS:
            factor = 15.3;
            break;
        case GYR_250DPS:
            factor = 7.6f;
            break;
        case GYR_125DPS:
            factor = 3.8f;
            break;
        default:
            factor = 0;
            break;
    }
    gyr->x = (double)x * factor / 1000.0f;
    gyr->y = (double)y * factor / 1000.0f;
    gyr->z = (double)z * factor / 1000.0f;
#endif
}

void BMX055::get_magnet(BMX055_MAGNET_TypeDef *mag)
{
    int16_t x,y,z;

    chip_addr = inf_addr.gyr_addr;
    dt[0] = 0x42;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
#if DEBUG
    printf("Read GYR data-> ");
    for (uint32_t i = 0; i < 6; i++){
        printf("i=%d,dt=0x%02x, ", i, dt[i]);
    }
    printf(", all\r\n");
#endif
    x = (dt[1] << 5 | (dt[0] & 0x1f)) << 3;    // 13bit
    y = (dt[3] << 5 | (dt[2] & 0x1f)) << 3;    // 13bit
    z = (dt[5] << 7 | (dt[4] & 0x7f)) << 1;    // 15bit
    mag->x = (double)x;
    mag->y = (double)y;
    mag->z = (double)z;
}

float BMX055::get_chip_temperature()
{
    chip_addr = inf_addr.acc_addr;
    dt[0] = 0x08;   // chip tempareture reg addr
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    //printf("Temp reg = 0x%02x\r\n", dt[0]);
    return (float)((int8_t)dt[0]) * 0.5f + 23.0f;
}

/////////////// Initialize ////////////////////////////////
void BMX055::initialize (void)
{
    _i2c.frequency(100000);
    // Check Acc & Mag & Gyro are available of not
    check_id();
    if (ready_flag == 0x07){
#if DEBUG
        printf("ACC+GYR+MAG are ready!\r\n");
#endif
    }
    // Set initial data
    set_parameters_to_regs();
}

////// Set initialize data to related registers ///////////
void BMX055::set_parameters_to_regs(void)
{
    // ACC
    chip_addr = inf_addr.acc_addr;
    dt[0] = 0x0f;   // Select PMU_Range register
    dt[1] = bmx055_parameters.acc_fs;
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x10;   // Select PMU_BW register
    dt[1] = bmx055_parameters.acc_bw;
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x11;   // Select PMU_LPW register
    dt[1] = 0x00;   // Normal mode, Sleep duration = 0.5ms
    _i2c.write(chip_addr, dt, 2, false);
    // GYR
    chip_addr = inf_addr.gyr_addr;
    dt[0] = 0x0f;   // Select Range register
    dt[1] = bmx055_parameters.gyr_fs;
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x10;   // Select Bandwidth register
    dt[1] = bmx055_parameters.gyr_bw;
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x11;   // Select LPM1 register
    dt[1] = 0x00;   // Normal mode, Sleep duration = 2ms
    _i2c.write(chip_addr, dt, 2, false);
    // MAG
    chip_addr = inf_addr.mag_addr;
    dt[0] = 0x4b;   // Select Mag register
    dt[1] = 0x83;   // Soft reset
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(10);
    dt[0] = 0x4b;   // Select Mag register
    dt[1] = 0x01;   // Soft reset
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(10);
    dt[0] = 0x4c;   // Select Mag register
    dt[1] = bmx055_parameters.mag_odr;
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x4e;   // Select Mag register
    dt[1] = 0x84;   // X, Y, Z-Axis enabled
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x51;   // Select Mag register
    dt[1] = 0x04;   // No. of Repetitions for X-Y Axis = 9
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x52;   // Select Mag register
    dt[1] = 0x16;   // No. of Repetitions for Z-Axis = 15
    _i2c.write(chip_addr, dt, 2, false);
#if 0
    // ACC
    chip_addr = inf_addr.acc_addr;
    dt[0] = 0x0f;   // Select PMU_Range register
    dt[1] = 0x03;   // Range = +/- 2g
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x10;   // Select PMU_BW register
    dt[1] = 0x08;   // Bandwidth = 7.81 Hz
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x11;   // Select PMU_LPW register
    dt[1] = 0x00;   // Normal mode, Sleep duration = 0.5ms
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    // GYR
    chip_addr = inf_addr.gyr_addr;
    dt[0] = 0x0f;   // Select Range register
    dt[1] = 0x04;   // Full scale = +/- 125 degree/s
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x10;   // Select Bandwidth register
    dt[1] = 0x07;   // ODR = 100 Hz
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x11;   // Select LPM1 register
    dt[1] = 0x00;   // Normal mode, Sleep duration = 2ms
    _i2c.write(chip_addr, dt, 2, false);
    // MAG
    chip_addr = inf_addr.mag_addr;
    dt[0] = 0x4b;   // Select Mag register
    dt[1] = 0x83;   // Soft reset
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(10);
    dt[0] = 0x4b;   // Select Mag register
    dt[1] = 0x01;   // Soft reset
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(10);
    dt[0] = 0x4c;   // Select Mag register
    dt[1] = 0x00;   // Normal Mode, ODR = 10 Hz
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x4e;   // Select Mag register
    dt[1] = 0x84;   // X, Y, Z-Axis enabled
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x51;   // Select Mag register
    dt[1] = 0x04;   // No. of Repetitions for X-Y Axis = 9
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
    dt[0] = 0x52;   // Select Mag register
    dt[1] = 0x16;   // No. of Repetitions for Z-Axis = 15
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1);
#endif
}

/////////////// Check Who am I? ///////////////////////////
void BMX055::check_id(void)
{
    ready_flag = 0;
    // ID ACC
    inf_addr.acc_addr = BMX055_ACC_CHIP_ADDR;
    chip_addr = inf_addr.acc_addr;
    dt[0] = 0x00;   // chip ID reg addr
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    inf_id.acc_id = dt[0];
    if (inf_id.acc_id == I_AM_BMX055_ACC) {
        ready_flag |= 0x01;
    } else {
        inf_addr.acc_addr = (0x18 << 1);
        chip_addr = inf_addr.acc_addr;
        dt[0] = 0x00;   // chip ID reg addr
        _i2c.write(chip_addr, dt, 1, true);
        _i2c.read(chip_addr, dt, 1, false);
        inf_id.acc_id = dt[0];
        if (inf_id.acc_id == I_AM_BMX055_ACC) {
            ready_flag |= 0x01;
        }
    }
    // ID GYRO
    inf_addr.gyr_addr = BMX055_GYR_CHIP_ADDR;
    chip_addr = inf_addr.gyr_addr;
    dt[0] = 0x00;   // chip ID reg addr
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    inf_id.gyr_id = dt[0];
    if (inf_id.gyr_id == I_AM_BMX055_GYR) {
        ready_flag |= 0x02;
    } else {
        inf_addr.gyr_addr = (0x68 << 1);
        chip_addr = inf_addr.gyr_addr;
        dt[0] = 0x00;   // chip ID reg addr
        _i2c.write(chip_addr, dt, 1, true);
        _i2c.read(chip_addr, dt, 1, false);
        inf_id.gyr_id = dt[0];
        if (inf_id.gyr_id == I_AM_BMX055_GYR) {
            ready_flag |= 0x02;
        }
    }
    // ID Mag
    inf_addr.mag_addr = BMX055_MAG_CHIP_ADDR;
    chip_addr = inf_addr.mag_addr;
    dt[0] = 0x4b;   // reg addr
    dt[1] = 0x01;   // control power bit set 1
    _i2c.write(chip_addr, dt, 2, false);
    dt[0] = 0x40;   // chip ID reg addr
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    inf_id.mag_id = dt[0];
    if (inf_id.mag_id == I_AM_BMX055_MAG) {
        ready_flag |= 0x04;
    } else {
        inf_addr.mag_addr = (0x12 << 1);
        chip_addr = inf_addr.mag_addr;
        // control power bit set 1
        dt[0] = 0x4b;
        dt[1] = 0x01;
        _i2c.write(chip_addr, dt, 2, false);
        dt[0] = 0x40;
        _i2c.write(chip_addr, dt, 1, true);
        _i2c.read(chip_addr, dt, 1, false);
        inf_id.mag_id = dt[0];
        if (inf_id.mag_id == I_AM_BMX055_MAG) {
            ready_flag |= 0x04;
        } else {
            inf_addr.mag_addr = (0x11 << 1);
            chip_addr = inf_addr.mag_addr;
            // control power bit set 1
            dt[0] = 0x4b;
            dt[1] = 0x01;
            _i2c.write(chip_addr, dt, 2, false);
            dt[0] = 0x40;
            _i2c.write(chip_addr, dt, 1, true);
            _i2c.read(chip_addr, dt, 1, false);
            inf_id.mag_id = dt[0];
            if (inf_id.mag_id == I_AM_BMX055_MAG) {
                ready_flag |= 0x04;
            } else {
                inf_addr.mag_addr = (0x10 << 1);
                chip_addr = inf_addr.mag_addr;
                // control power bit set 1
                dt[0] = 0x4b;
                dt[1] = 0x01;
                _i2c.write(chip_addr, dt, 2, false);
                dt[0] = 0x40;
                _i2c.write(chip_addr, dt, 1, true);
                _i2c.read(chip_addr, dt, 1, false);
                inf_id.mag_id = dt[0];
                if (inf_id.mag_id == I_AM_BMX055_MAG) {
                    ready_flag |= 0x04;
                }
            }
        }
    }
#if DEBUG
    printf("ACC addr=0x%02x, id=0x%02x\r\n", inf_addr.acc_addr, inf_id.acc_id); 
    printf("GYR addr=0x%02x, id=0x%02x\r\n", inf_addr.gyr_addr, inf_id.gyr_id); 
    printf("MAG addr=0x%02x, id=0x%02x\r\n", inf_addr.mag_addr, inf_id.mag_id); 
    printf("ready_flag = 0x%x\r\n", ready_flag);
#endif
}

void BMX055::read_id_inf(BMX055_ID_INF_TypeDef *id)
{
    id->acc_id = acc_id;
    id->mag_id = mag_id;
    id->gyr_id = gyr_id;
}

/////////////// Check chip ready or not  //////////////////
bool BMX055::chip_ready(void)
{
    if (ready_flag == 0x07) {
        return true;
    }
    return false;
}

/////////////// I2C Freq. /////////////////////////////////
void BMX055::frequency(int hz)
{
    _i2c.frequency(hz);
}

/////////////// Read/Write specific register //////////////
uint8_t BMX055::read_reg(uint8_t addr)
{
    dt[0] = addr;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    return (uint8_t)dt[0];
}

uint8_t BMX055::write_reg(uint8_t addr, uint8_t data)
{
    uint8_t d;

    dt[0] = addr;
    dt[1] = data;
    _i2c.write(chip_addr, dt, 2, false);
    d = dt[0];
    return d;
}
