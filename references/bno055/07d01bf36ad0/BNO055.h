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
/*
 *---------------- REFERENCE ----------------------------------------------------------------------
 * Original Information
 *  https://www.bosch-sensortec.com/en/homepage/products_3/sensor_hubs/iot_solutions/bno055_1/bno055_4
 *  Intelligent 9-axis absolute orientation sensor / Data Sheet  BST_BNO055_DS000_12 Nov. 2014 rev.1.2
 *  Sample software   https://github.com/BoschSensortec/BNO055_driver
 * Sensor board
 *  https://www.rutronik24.com/product/bosch+se/bno055+shuttle+board+mems/6431291.html
 *  http://microcontrollershop.com/product_info.php?products_id=7140&osCsid=10645k86db2crld4tfi0vol5g5
 */

#ifndef BNO055_H
#define BNO055_H

#include "mbed.h"

//  BNO055
//  7bit address = 0b010100x(0x28 or 0x29 depends on COM3)
#define BNO055_G_CHIP_ADDR      (0x28 << 1) // COM3 = GND
#define BNO055_V_CHIP_ADDR      (0x29 << 1) // COM3 = Vdd

// Fusion mode
#define CONFIGMODE              0x00
#define MODE_IMU                0x08
#define MODE_COMPASS            0x09
#define MODE_M4G                0x0a
#define MODE_NDOF_FMC_OFF       0x0b
#define MODE_NDOF               0x0c

//  UNIT
#define UNIT_ACC_MSS            0x00    // acc m/s2
#define UNIT_ACC_MG             0x01    // acc mg
#define UNIT_GYR_DPS            0x00    // gyro Dps
#define UNIT_GYR_RPS            0x02    // gyro Rps
#define UNIT_EULER_DEG          0x00    // euler Degrees
#define UNIT_EULER_RAD          0x04    // euler Radians
#define UNIT_TEMP_C             0x00    // temperature degC
#define UNIT_TEMP_F             0x10    // temperature degF
#define UNIT_ORI_WIN            0x00    // Windows orientation
#define UNIT_ORI_ANDROID        0x80    // Android orientation

//  ID's
#define I_AM_BNO055_CHIP        0xa0    // CHIP ID
#define I_AM_BNO055_ACC         0xfb    // ACC ID
#define I_AM_BNO055_MAG         0x32    // MAG ID
#define I_AM_BNO055_GYR         0x0f    // GYR ID

////////////// DATA TYPE DEFINITION ///////////////////////
typedef struct {
    uint8_t  chip_id;
    uint8_t  acc_id;
    uint8_t  mag_id;
    uint8_t  gyr_id;
    uint8_t  bootldr_rev_id;
    uint16_t sw_rev_id;
} BNO055_ID_INF_TypeDef;

typedef struct {
    double h;
    double r;
    double p;
} BNO055_EULER_TypeDef;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t w;
} BNO055_QUATERNION_TypeDef;

typedef struct {
    double x;
    double y;
    double z;
} BNO055_LIN_ACC_TypeDef;

typedef struct {
    double x;
    double y;
    double z;
} BNO055_GRAVITY_TypeDef;

typedef struct {
    int8_t acc_chip;
    int8_t gyr_chip;
} BNO055_TEMPERATURE_TypeDef;

enum {MT_P0 = 0, MT_P1, MT_P2, MT_P3, MT_P4, MT_P5, MT_P6, MT_P7};

/** Interface for Bosch Sensortec Intelligent 9-axis absolute orientation sensor
 *      Chip: BNO055
 *
 * @code
 * #include    "mbed.h"
 * #include    "BNO055.h"
 *
 * Serial pc(USBTX,USBRX);
 * I2C    i2c(PB_9, PB_8);         // SDA, SCL
 * BNO055 imu(i2c, PA_8);          // Reset
 *
 * BNO055_ID_INF_TypeDef bno055_id_inf;
 * BNO055_EULER_TypeDef  euler_angles;
 *
 * int main() {
 *     pc.printf("Bosch Sensortec BNO055 test program on " __DATE__ "/" __TIME__ "\r\n");
 *     if (imu.chip_ready() == 0){
 *         pc.printf("Bosch BNO055 is NOT avirable!!\r\n");
 *     }
 *     imu.read_id_inf(&bno055_id_inf);
 *     pc.printf("CHIP:0x%02x, ACC:0x%02x, MAG:0x%02x, GYR:0x%02x, , SW:0x%04x, , BL:0x%02x\r\n",
 *                bno055_id_inf.chip_id, bno055_id_inf.acc_id, bno055_id_inf.mag_id,
 *                bno055_id_inf.gyr_id, bno055_id_inf.sw_rev_id, bno055_id_inf.bootldr_rev_id);
 *     while(1) {
 *         imu.get_Euler_Angles(&euler_angles);
 *         pc.printf("Heading:%+6.1f [deg], Roll:%+6.1f [deg], Pich:%+6.1f [deg]\r\n",
 *                    euler_angles.h, euler_angles.r, euler_angles.p);
 *         wait(0.5);
 *     }
 * }
 * @endcode
 */

class BNO055
{
public:
    /** Configure data pin
      * @param data SDA and SCL pins
      * @param device address
      */
    BNO055(PinName p_sda, PinName p_scl, PinName p_reset, uint8_t addr, uint8_t mode);

    /** Configure data pin
      * @param data SDA and SCL pins
      * @param Other parameters are set default data
      */
    BNO055(PinName p_sda, PinName p_scl, PinName p_reset);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @param device address
      */
    BNO055(I2C& p_i2c, PinName p_reset, uint8_t addr, uint8_t mode);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @param Other parameters are set default data
      */
    BNO055(I2C& p_i2c, PinName p_reset);

    /** Get Euler Angles
     * @param double type of 3D data address
     */
    void get_Euler_Angles(BNO055_EULER_TypeDef *el);

    /** Get Quaternion XYZ&W
     * @param int16_t type of 4D data address
     */
    void get_quaternion(BNO055_QUATERNION_TypeDef *qua);

    /** Get Linear accel data
     * @param double type of 3D data address
     */
    void get_linear_accel(BNO055_LIN_ACC_TypeDef *la);

    /** Get Gravity data
     * @param double type of 3D data address
     */
    void get_gravity(BNO055_GRAVITY_TypeDef *gr);

    /** Get Chip temperature data both Acc & Gyro
     * @param int8_t type of data address
     */
    void get_chip_temperature(BNO055_TEMPERATURE_TypeDef *tmp);

    /** Change fusion mode
      * @param fusion mode
      * @return none
      */
    void change_fusion_mode(uint8_t mode);

    /** Set Mouting position
      *  Please make sure your mounting direction of BNO055 chip
      *  refrence: BNO055 data sheet BST-BNO055-DS000-12 3.4 Axis remap
      * @param Set P0 to P7 mounting position data
      * @return none
      */
    void set_mounting_position(uint8_t position);

    /** Read BNO055 ID information
      * @param ID information address
      * @return none
      */
    void read_id_inf(BNO055_ID_INF_TypeDef *id);

    /** Check chip is avairable or not
      * @param none
      * @return OK = 1, NG = 0;
      */
    uint8_t chip_ready(void);

    /** Read calibration status
      * @param none
      * @return SYS(7:6),GYR(5:4),ACC(3:2),MAG(1:0) 3 = Calibrated, 0= not yet
      */
    uint8_t read_calib_status(void);

    /** Reset
      * @param none
      * @return 0 = sucess, 1 = Not available chip
      */
    uint8_t reset(void);

    /** Set I2C clock frequency
      * @param freq.
      * @return none
      */
    void frequency(int hz);

    /** Read page 0 register
      * @param register's address
      * @return register data
      */
    uint8_t read_reg0(uint8_t addr);

    /** Write page 0 register
      * @param register's address
      * @param data
      * @return register data
      */
    uint8_t write_reg0(uint8_t addr, uint8_t data);

    /** Read page 1 register
      * @param register's address
      * @return register data
      */
    uint8_t read_reg1(uint8_t addr);

    /** Write page 1 register
      * @param register's address
      * @param data
      * @return register data
      */
    uint8_t write_reg1(uint8_t addr, uint8_t data);

protected:
    void initialize(void);
    void check_id(void);
    void set_initial_dt_to_regs(void);
    void unit_selection(void);
    uint8_t check_operating_mode(void);
    uint8_t select_page(uint8_t page);

    I2C *_i2c_p;
    I2C &_i2c;
    DigitalOut _res;

private:
    char     dt[10];      // working buffer
    uint8_t  chip_addr;
    uint8_t  chip_mode;
    uint8_t  ready_flag;
    uint8_t  page_flag;

    uint8_t  chip_id;
    uint8_t  acc_id;
    uint8_t  mag_id;
    uint8_t  gyr_id;
    uint8_t  bootldr_rev_id;
    uint16_t sw_rev_id;

};

//---------------------------------------------------------
//----- Register's definition -----------------------------
//---------------------------------------------------------
// Page id register definition
#define BNO055_PAGE_ID          0x07

//----- page0 ---------------------------------------------
#define BNO055_CHIP_ID          0x00
#define BNO055_ACCEL_REV_ID     0x01
#define BNO055_MAG_REV_ID       0x02
#define BNO055_GYRO_REV_ID      0x03
#define BNO055_SW_REV_ID_LSB    0x04
#define BNO055_SW_REV_ID_MSB    0x05
#define BNO055_BL_REV_ID        0x06

// Accel data register*/
#define BNO055_ACC_X_LSB        0x08
#define BNO055_ACC_X_MSB        0x09
#define BNO055_ACC_Y_LSB        0x0a
#define BNO055_ACC_Y_MSB        0x0b
#define BNO055_ACC_Z_LSB        0x0c
#define BNO055_ACC_Z_MSB        0x0d

// Mag data register
#define BNO055_MAG_X_LSB        0x0e
#define BNO055_MAG_X_MSB        0x0f
#define BNO055_MAG_Y_LSB        0x10
#define BNO055_MAG_Y_MSB        0x11
#define BNO055_MAG_Z_LSB        0x12
#define BNO055_MAG_Z_MSB        0x13

// Gyro data registers
#define BNO055_GYR_X_LSB        0x14
#define BNO055_GYR_X_MSB        0x15
#define BNO055_GYR_Y_LSB        0x16
#define BNO055_GYR_Y_MSB        0x17
#define BNO055_GYR_Z_LSB        0x18
#define BNO055_GYR_Z_MSB        0x19

// Euler data registers
#define BNO055_EULER_H_LSB      0x1a
#define BNO055_EULER_H_MSB      0x1b

#define BNO055_EULER_R_LSB      0x1c
#define BNO055_EULER_R_MSB      0x1d

#define BNO055_EULER_P_LSB      0x1e
#define BNO055_EULER_P_MSB      0x1f

// Quaternion data registers
#define BNO055_QUATERNION_W_LSB 0x20
#define BNO055_QUATERNION_W_MSB 0x21
#define BNO055_QUATERNION_X_LSB 0x22
#define BNO055_QUATERNION_X_MSB 0x23
#define BNO055_QUATERNION_Y_LSB 0x24
#define BNO055_QUATERNION_Y_MSB 0x25
#define BNO055_QUATERNION_Z_LSB 0x26
#define BNO055_QUATERNION_Z_MSB 0x27

// Linear acceleration data registers
#define BNO055_LINEAR_ACC_X_LSB 0x28
#define BNO055_LINEAR_ACC_X_MSB 0x29
#define BNO055_LINEAR_ACC_Y_LSB 0x2a
#define BNO055_LINEAR_ACC_Y_MSB 0x2b
#define BNO055_LINEAR_ACC_Z_LSB 0x2c
#define BNO055_LINEAR_ACC_Z_MSB 0x2d

// Gravity data registers
#define BNO055_GRAVITY_X_LSB    0x2e
#define BNO055_GRAVITY_X_MSB    0x2f
#define BNO055_GRAVITY_Y_LSB    0x30
#define BNO055_GRAVITY_Y_MSB    0x31
#define BNO055_GRAVITY_Z_LSB    0x32
#define BNO055_GRAVITY_Z_MSB    0x33

// Temperature data register
#define BNO055_TEMP             0x34

// Status registers
#define BNO055_CALIB_STAT       0x35
#define BNO055_SELFTEST_RESULT  0x36
#define BNO055_INTR_STAT        0x37
#define BNO055_SYS_CLK_STAT     0x38
#define BNO055_SYS_STAT         0x39
#define BNO055_SYS_ERR          0x3a

// Unit selection register
#define BNO055_UNIT_SEL         0x3b
#define BNO055_DATA_SELECT      0x3c

// Mode registers
#define BNO055_OPR_MODE         0x3d
#define BNO055_PWR_MODE         0x3e
#define BNO055_SYS_TRIGGER      0x3f
#define BNO055_TEMP_SOURCE      0x40

// Axis remap registers
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42

// SIC registers
#define BNO055_SIC_MTRX_0_LSB   0x43
#define BNO055_SIC_MTRX_0_MSB   0x44
#define BNO055_SIC_MTRX_1_LSB   0x45
#define BNO055_SIC_MTRX_1_MSB   0x46
#define BNO055_SIC_MTRX_2_LSB   0x47
#define BNO055_SIC_MTRX_2_MSB   0x48
#define BNO055_SIC_MTRX_3_LSB   0x49
#define BNO055_SIC_MTRX_3_MSB   0x4a
#define BNO055_SIC_MTRX_4_LSB   0x4b
#define BNO055_SIC_MTRX_4_MSB   0x4c
#define BNO055_SIC_MTRX_5_LSB   0x4d
#define BNO055_SIC_MTRX_5_MSB   0x4e
#define BNO055_SIC_MTRX_6_LSB   0x4f
#define BNO055_SIC_MTRX_6_MSB   0x50
#define BNO055_SIC_MTRX_7_LSB   0x51
#define BNO055_SIC_MTRX_7_MSB   0x52
#define BNO055_SIC_MTRX_8_LSB   0x53
#define BNO055_SIC_MTRX_8_MSB   0x54

// Accelerometer Offset registers
#define ACCEL_OFFSET_X_LSB      0x55
#define ACCEL_OFFSET_X_MSB      0x56
#define ACCEL_OFFSET_Y_LSB      0x57
#define ACCEL_OFFSET_Y_MSB      0x58
#define ACCEL_OFFSET_Z_LSB      0x59
#define ACCEL_OFFSET_Z_MSB      0x5a

// Magnetometer Offset registers
#define MAG_OFFSET_X_LSB        0x5b
#define MAG_OFFSET_X_MSB        0x5c
#define MAG_OFFSET_Y_LSB        0x5d
#define MAG_OFFSET_Y_MSB        0x5e
#define MAG_OFFSET_Z_LSB        0x5f
#define MAG_OFFSET_Z_MSB        0x60

// Gyroscope Offset registers
#define GYRO_OFFSET_X_LSB       0x61
#define GYRO_OFFSET_X_MSB       0x62
#define GYRO_OFFSET_Y_LSB       0x63
#define GYRO_OFFSET_Y_MSB       0x64
#define GYRO_OFFSET_Z_LSB       0x65
#define GYRO_OFFSET_Z_MSB       0x66

// Radius registers
#define ACCEL_RADIUS_LSB        0x67
#define ACCEL_RADIUS_MSB        0x68
#define MAG_RADIUS_LSB          0x69
#define MAG_RADIUS_MSB          0x6a

//----- page1 ---------------------------------------------
// Configuration registers
#define ACCEL_CONFIG            0x08
#define MAG_CONFIG              0x09
#define GYRO_CONFIG             0x0a
#define GYRO_MODE_CONFIG        0x0b
#define ACCEL_SLEEP_CONFIG      0x0c
#define GYRO_SLEEP_CONFIG       0x0d
#define MAG_SLEEP_CONFIG        0x0e

// Interrupt registers
#define INT_MASK                0x0f
#define INT                     0x10
#define ACCEL_ANY_MOTION_THRES  0x11
#define ACCEL_INTR_SETTINGS     0x12
#define ACCEL_HIGH_G_DURN       0x13
#define ACCEL_HIGH_G_THRES      0x14
#define ACCEL_NO_MOTION_THRES   0x15
#define ACCEL_NO_MOTION_SET     0x16
#define GYRO_INTR_SETING        0x17
#define GYRO_HIGHRATE_X_SET     0x18
#define GYRO_DURN_X             0x19
#define GYRO_HIGHRATE_Y_SET     0x1a
#define GYRO_DURN_Y             0x1b
#define GYRO_HIGHRATE_Z_SET     0x1c
#define GYRO_DURN_Z             0x1d
#define GYRO_ANY_MOTION_THRES   0x1e
#define GYRO_ANY_MOTION_SET     0x1f

//---------------------------------------------------------
//----- Calibration example -------------------------------
//---------------------------------------------------------
#if 0
// Calibration
//  Please refer BNO055 Data sheet 3.10 Calibration & 3.6.4 Sensor calibration data
void bno055_calbration(void){
    uint8_t d;

    pc.printf("------ Enter BNO055 Manual Calibration Mode ------\r\n");
    //---------- Gyroscope Caliblation ------------------------------------------------------------
    // (a) Place the device in a single stable position for a period of few seconds to allow the
    //     gyroscope to calibrate
    pc.printf("Step1) Please wait few seconds\r\n");
    t.start();
    while (t.read() < 10){
        d = imu.read_calib_status();
        pc.printf("Calb dat = 0x%x target  = 0x30(at least)\r\n", d);
        if ((d & 0x30) == 0x30){
            break;
        }
        wait(1.0);
    }
    pc.printf("-> Step1) is done\r\n\r\n");
    //---------- Magnetometer Caliblation ---------------------------------------------------------
    // (a) Make some random movements (for example: writing the number ‘8’ on air) until the
    //     CALIB_STAT register indicates fully calibrated.
    // (b) It takes more calibration movements to get the magnetometer calibrated than in the
    //     NDOF mode.
    pc.printf("Step2) random moving (try to change the BNO055 axis)\r\n");
    t.start();
    while (t.read() < 30){
        d = imu.read_calib_status();
        pc.printf("Calb dat = 0x%x target  = 0x33(at least)\r\n", d);
        if ((d & 0x03) == 0x03){
            break;
        }
        wait(1.0);   
    }
    pc.printf("-> Step2) is done\r\n\r\n");
    //---------- Magnetometer Caliblation ---------------------------------------------------------
    // a) Place the device in 6 different stable positions for a period of few seconds
    //    to allow the accelerometer to calibrate.
    // b) Make sure that there is slow movement between 2 stable positions
    //    The 6 stable positions could be in any direction, but make sure that the device is
    //    lying at least once perpendicular to the x, y and z axis.
    pc.printf("Step3) Change rotation each X,Y,Z axis KEEP SLOWLY!!");
    pc.printf(" Each 90deg stay a 5 sec and set at least 6 position.\r\n");
    pc.printf(" e.g. (1)ACC:X0,Y0,Z-9,(2)ACC:X9,Y0,Z0,(3)ACC:X0,Y0,Z9,");
    pc.printf("(4)ACC:X-9,Y0,Z0,(5)ACC:X0,Y-9,Z0,(6)ACC:X0,Y9,Z0,\r\n");
    pc.printf(" If you will give up, hit any key.\r\n", d);
    t.stop();
    while (true){
        d = imu.read_calib_status();
        imu.get_gravity(&gravity);
        pc.printf("Calb dat = 0x%x target  = 0xff ACC:X %3.0f, Y %3.0f, Z %3.0f\r\n",
                   d, gravity.x, gravity.y, gravity.z);
        if (d == 0xff){     break;}
        if (pc.readable()){ break;}
        wait(1.0);
    }
    if (imu.read_calib_status() == 0xff){
        pc.printf("-> All of Calibration steps are done successfully!\r\n\r\n");
    } else {
        pc.printf("-> Calibration steps are suspended!\r\n\r\n");
    }
    t.stop();
}
#endif

#endif      // BNO055_H
