/*
 * mbed library program
 *  BNO055 Intelligent 9-axis absolute orientation sensor
 *  by Bosch Sensortec
 *
 * Copyright (c) 2015,'17 Kenji Arai / JH1PJL
 *  http://www.page.sannet.ne.jp/kenjia/index.html
 *  http://mbed.org/users/kenjiArai/
 *      Created: March     30th, 2015
 *      Revised: August    23rd, 2017
 */

#include "mbed.h"
#include "BNO055.h"


#if MBED_MAJOR_VERSION == 2
#define WAIT_MS(x)       wait_ms(x)
#elif  MBED_MAJOR_VERSION == 5
#define WAIT_MS(x)       Thread::wait(x)
#else
#error "Running on Unknown OS"
#endif

BNO055::BNO055 (PinName p_sda, PinName p_scl, PinName p_reset, uint8_t addr, uint8_t mode):
    _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p), _res(p_reset)
{
    chip_addr = addr;
    chip_mode = mode;
    initialize ();
}

BNO055::BNO055 (PinName p_sda, PinName p_scl, PinName p_reset) :
    _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p), _res(p_reset)
{
    chip_addr = BNO055_G_CHIP_ADDR;
    chip_mode = MODE_NDOF;
    initialize ();
}

BNO055::BNO055 (I2C& p_i2c, PinName p_reset, uint8_t addr, uint8_t mode) :
    _i2c(p_i2c), _res(p_reset)
{
    chip_addr = addr;
    chip_mode = mode;
    initialize ();
}

BNO055::BNO055 (I2C& p_i2c, PinName p_reset) :
    _i2c(p_i2c), _res(p_reset)
{
    chip_addr = BNO055_G_CHIP_ADDR;
    chip_mode = MODE_NDOF;
    initialize ();
}

/////////////// Read data & normalize /////////////////////
void BNO055::get_Euler_Angles(BNO055_EULER_TypeDef *el)
{
    uint8_t deg_or_rad;
    int16_t h,p,r;

    select_page(0);
    dt[0] = BNO055_UNIT_SEL;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    if (dt[0] & 0x04) {
        deg_or_rad = 1; // Radian
    } else {
        deg_or_rad = 0; // Degree
    }
    dt[0] = BNO055_EULER_H_LSB;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
    h = dt[1] << 8 | dt[0];
    p = dt[3] << 8 | dt[2];
    r = dt[5] << 8 | dt[4];
    if (deg_or_rad) {
        el->h = (double)h / 900;
        el->p = (double)p / 900;
        el->r = (double)r / 900;
    } else {
        el->h = (double)h / 16;
        el->p = (double)p / 16;
        el->r = (double)r / 16;
    }
}

void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *qua)
{
    select_page(0);
    dt[0] = BNO055_QUATERNION_W_LSB;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 8, false);
    qua->w = dt[1] << 8 | dt[0];
    qua->x = dt[3] << 8 | dt[2];
    qua->y = dt[5] << 8 | dt[4];
    qua->z = dt[7] << 8 | dt[6];
}

void BNO055::get_linear_accel(BNO055_LIN_ACC_TypeDef *la)
{
    uint8_t ms2_or_mg;
    int16_t x,y,z;

    select_page(0);
    dt[0] = BNO055_UNIT_SEL;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    if (dt[0] & 0x01) {
        ms2_or_mg = 1; // mg
    } else {
        ms2_or_mg = 0; // m/s*s
    }
    dt[0] = BNO055_LINEAR_ACC_X_LSB;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
    x = dt[1] << 8 | dt[0];
    y = dt[3] << 8 | dt[2];
    z = dt[5] << 8 | dt[4];
    if (ms2_or_mg) {
        la->x = (double)x;
        la->y = (double)y;
        la->z = (double)z;
    } else {
        la->x = (double)x / 100;
        la->y = (double)y / 100;
        la->z = (double)z / 100;
    }
}

void BNO055::get_gravity(BNO055_GRAVITY_TypeDef *gr)
{
    uint8_t ms2_or_mg;
    int16_t x,y,z;

    select_page(0);
    dt[0] = BNO055_UNIT_SEL;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    if (dt[0] & 0x01) {
        ms2_or_mg = 1; // mg
    } else {
        ms2_or_mg = 0; // m/s*s
    }
    dt[0] = BNO055_GRAVITY_X_LSB;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
    x = dt[1] << 8 | dt[0];
    y = dt[3] << 8 | dt[2];
    z = dt[5] << 8 | dt[4];
    if (ms2_or_mg) {
        gr->x = (double)x;
        gr->y = (double)y;
        gr->z = (double)z;
    } else {
        gr->x = (double)x / 100;
        gr->y = (double)y / 100;
        gr->z = (double)z / 100;
    }
}

void BNO055::get_chip_temperature(BNO055_TEMPERATURE_TypeDef *tmp)
{
    uint8_t c_or_f;

    select_page(0);
    dt[0] = BNO055_UNIT_SEL;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    if (dt[0] & 0x10) {
        c_or_f = 1; // Fahrenheit
    } else {
        c_or_f = 0; // degrees Celsius
    }
    dt[0] = BNO055_TEMP_SOURCE;
    dt[1] = 0;
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1); // Do I need to wait?
    dt[0] = BNO055_TEMP;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    if (c_or_f) {
        tmp->acc_chip = (int8_t)dt[0] * 2;
    } else {
        tmp->acc_chip = (int8_t)dt[0];
    }
    dt[0] = BNO055_TEMP_SOURCE;
    dt[1] = 1;
    _i2c.write(chip_addr, dt, 2, false);
    WAIT_MS(1); // Do I need to wait?
    dt[0] = BNO055_TEMP;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    if (c_or_f) {
        tmp->gyr_chip = (int8_t)dt[0] * 2;
    } else {
        tmp->gyr_chip = (int8_t)dt[0];
    }
}

/////////////// Initialize ////////////////////////////////
void BNO055::initialize (void)
{
#if defined(TARGET_STM32L152RE)
    _i2c.frequency(100000);
#else
    _i2c.frequency(400000);
#endif
    page_flag = 0xff;
    select_page(0);
    // Check Acc & Mag & Gyro are available of not
    check_id();
    // Set initial data
    set_initial_dt_to_regs();
    // Unit selection
    unit_selection();
    // Set fusion mode
    change_fusion_mode(chip_mode);
}

void BNO055::unit_selection(void)
{
    select_page(0);
    dt[0] = BNO055_UNIT_SEL;
    dt[1] = UNIT_ORI_WIN + UNIT_ACC_MSS + UNIT_GYR_DPS + UNIT_EULER_DEG + UNIT_TEMP_C;
    _i2c.write(chip_addr, dt, 2, false);
}

uint8_t BNO055::select_page(uint8_t page)
{
    if (page != page_flag){
        dt[0] = BNO055_PAGE_ID;
        if (page == 1) {
            dt[1] = 1;  // select page 1
        } else {
            dt[1] = 0;  // select page 0
        }
        _i2c.write(chip_addr, dt, 2, false);
        dt[0] = BNO055_PAGE_ID;
        _i2c.write(chip_addr, dt, 1, true);
        _i2c.read(chip_addr, dt, 1, false);
        page_flag = dt[0];
    }
    return page_flag;
}

uint8_t BNO055::reset(void)
{
     _res = 0;
     WAIT_MS(1);   // Reset 1mS
     _res = 1;
     WAIT_MS(700); // Need to wait at least 650mS
#if defined(TARGET_STM32L152RE)
    _i2c.frequency(400000);
#else
    _i2c.frequency(400000);
#endif
    _i2c.stop();
    page_flag = 0xff;
    select_page(0);
    check_id();
    if (chip_id != I_AM_BNO055_CHIP){
        return 1;
    } else {
        initialize();
        return 0;
    }
}

////// Set initialize data to related registers ///////////
void BNO055::set_initial_dt_to_regs(void)
{
    // select_page(0);
    // current setting is only used default values
}

/////////////// Check Who am I? ///////////////////////////
void BNO055::check_id(void)
{
    select_page(0);
    // ID
    dt[0] = BNO055_CHIP_ID;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 7, false);
    chip_id = dt[0];
    if (chip_id == I_AM_BNO055_CHIP) {
        ready_flag = 1;
    } else {
        ready_flag = 0;
    }
    acc_id = dt[1];
    if (acc_id == I_AM_BNO055_ACC) {
        ready_flag |= 2;
    }
    mag_id = dt[2];
    if (mag_id == I_AM_BNO055_MAG) {
        ready_flag |= 4;
    }
    gyr_id = dt[3];
    if (mag_id == I_AM_BNO055_MAG) {
        ready_flag |= 8;
    }
    bootldr_rev_id = dt[5]<< 8 | dt[4];
    sw_rev_id = dt[6];
}

void BNO055::read_id_inf(BNO055_ID_INF_TypeDef *id)
{
    id->chip_id = chip_id;
    id->acc_id = acc_id;
    id->mag_id = mag_id;
    id->gyr_id = gyr_id;
    id->bootldr_rev_id = bootldr_rev_id;
    id->sw_rev_id = sw_rev_id;
}

/////////////// Check chip ready or not  //////////////////
uint8_t BNO055::chip_ready(void)
{
    if (ready_flag == 0x0f) {
        return 1;
    }
    return 0;
}

/////////////// Read Calibration status  //////////////////
uint8_t BNO055::read_calib_status(void)
{
    select_page(0);
    dt[0] = BNO055_CALIB_STAT;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    return dt[0];
}

/////////////// Change Fusion mode  ///////////////////////
void BNO055::change_fusion_mode(uint8_t mode)
{
    uint8_t current_mode;

    select_page(0);
    current_mode = check_operating_mode();
    switch (mode) {
        case CONFIGMODE:
            dt[0] = BNO055_OPR_MODE;
            dt[1] = mode;
            _i2c.write(chip_addr, dt, 2, false);
            WAIT_MS(19);    // wait 19mS
            break;
        case MODE_IMU:
        case MODE_COMPASS:
        case MODE_M4G:
        case MODE_NDOF_FMC_OFF:
        case MODE_NDOF:
            if (current_mode != CONFIGMODE) {   // Can we change the mode directry?
                dt[0] = BNO055_OPR_MODE;
                dt[1] = CONFIGMODE;
                _i2c.write(chip_addr, dt, 2, false);
                WAIT_MS(19);    // wait 19mS
            }
            dt[0] = BNO055_OPR_MODE;
            dt[1] = mode;
            _i2c.write(chip_addr, dt, 2, false);
            WAIT_MS(7);    // wait 7mS
            break;
        default:
            break;
    }
}

uint8_t BNO055::check_operating_mode(void)
{
    select_page(0);
    dt[0] = BNO055_OPR_MODE;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    return dt[0];
}

/////////////// Set Mouting position  /////////////////////
void BNO055::set_mounting_position(uint8_t position)
{
    uint8_t remap_config;
    uint8_t remap_sign;
    uint8_t current_mode;

    current_mode = check_operating_mode();
    change_fusion_mode(CONFIGMODE);
    switch (position) {
        case MT_P0:
            remap_config = 0x21;
            remap_sign = 0x04;
            break;
        case MT_P2:
            remap_config = 0x24;
            remap_sign = 0x06;
            break;
        case MT_P3:
            remap_config = 0x21;
            remap_sign = 0x02;
            break;
        case MT_P4:
            remap_config = 0x24;
            remap_sign = 0x03;
            break;
        case MT_P5:
            remap_config = 0x21;
            remap_sign = 0x01;
            break;
        case MT_P6:
            remap_config = 0x21;
            remap_sign = 0x07;
            break;
        case MT_P7:
            remap_config = 0x24;
            remap_sign = 0x05;
            break;
        case MT_P1:
        default:
            remap_config = 0x24;
            remap_sign = 0x00;
            break;
    }
    dt[0] = BNO055_AXIS_MAP_CONFIG;
    dt[1] = remap_config;
    dt[2] = remap_sign;
    _i2c.write(chip_addr, dt, 3, false);
    change_fusion_mode(current_mode);
}

/////////////// I2C Freq. /////////////////////////////////
void BNO055::frequency(int hz)
{
    _i2c.frequency(hz);
}

/////////////// Read/Write specific register //////////////
uint8_t BNO055::read_reg0(uint8_t addr)
{
    select_page(0);
    dt[0] = addr;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    return (uint8_t)dt[0];
}

uint8_t BNO055::write_reg0(uint8_t addr, uint8_t data)
{
    uint8_t current_mode;
    uint8_t d;

    current_mode = check_operating_mode();
    change_fusion_mode(CONFIGMODE);
    dt[0] = addr;
    dt[1] = data;
    _i2c.write(chip_addr, dt, 2, false);
    d = dt[0];
    change_fusion_mode(current_mode);
    return d;
}

uint8_t BNO055::read_reg1(uint8_t addr)
{
    select_page(1);
    dt[0] = addr;
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 1, false);
    return (uint8_t)dt[0];
}

uint8_t BNO055::write_reg1(uint8_t addr, uint8_t data)
{
    uint8_t current_mode;
    uint8_t d;

    current_mode = check_operating_mode();
    change_fusion_mode(CONFIGMODE);
    select_page(1);
    dt[0] = addr;
    dt[1] = data;
    _i2c.write(chip_addr, dt, 2, false);
    d = dt[0];
    change_fusion_mode(current_mode);
    return d;
}


