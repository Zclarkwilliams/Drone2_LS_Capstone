/*

ECE 412-413 Capstone Winter/Spring 2018
Team 32 Drone2 SOC
Ethan Grinnell, Brett Creely, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams

BOSCH BNO055 register offsets

Modified from Adafruit Bosch BNO055 Arduino driver code
*/

/*
Original file header:

**************************************************************************
This is a library for the BNO055 orientation sensor

Designed specifically to work with the Adafruit BNO055 Breakout.

Pick one up today in the adafruit shop!
------> http://www.adafruit.com/products

These sensors use I2C to communicate, 2 pins are required to interface.

Adafruit invests time and resources providing this open source code,
please support Adafruit andopen-source hardware by purchasing products
from Adafruit!

Written by KTOWN for Adafruit Industries.

MIT license, all text above must be included in any redistribution

 ***************************************************************************/


`define BNO055_ADDRESS_A                     8'h28
`define BNO055_ADDRESS_B                     8'h29
`define BNO055_ID                            8'hA0


//PAGE0 REGISTER DEFINITION START
//"All other configuration parameters and output data", all other in Page1
//Page0 is the default configuration page

//GENERAL REGISTERS
`define BNO055_CHIP_ID_ADDR                  8'h00
`define BNO055_ACCEL_REV_ID_ADDR             8'h01
`define BNO055_MAG_REV_ID_ADDR               8'h02
`define BNO055_GYRO_REV_ID_ADDR              8'h03
`define BNO055_SW_REV_ID_LSB_ADDR            8'h04
`define BNO055_SW_REV_ID_MSB_ADDR            8'h05
`define BNO055_BL_REV_ID_ADDR                8'h06
`define BNO055_PAGE_ID_ADDR                  8'h07 //Defaults to page 0 - This register is on page 0 and page 1 at the same location

//ACCEL DATA REGISTER
`define BNO055_ACCEL_DATA_X_LSB_ADDR         8'h08
`define BNO055_ACCEL_DATA_X_MSB_ADDR         8'h09
`define BNO055_ACCEL_DATA_Y_LSB_ADDR         8'h0A
`define BNO055_ACCEL_DATA_Y_MSB_ADDR         8'h0B
`define BNO055_ACCEL_DATA_Z_LSB_ADDR         8'h0C
`define BNO055_ACCEL_DATA_Z_MSB_ADDR         8'h0D

//MAG DATA REGISTER
`define BNO055_MAG_DATA_X_LSB_ADDR           8'h0E
`define BNO055_MAG_DATA_X_MSB_ADDR           8'h0F
`define BNO055_MAG_DATA_Y_LSB_ADDR           8'h10
`define BNO055_MAG_DATA_Y_MSB_ADDR           8'h11
`define BNO055_MAG_DATA_Z_LSB_ADDR           8'h12
`define BNO055_MAG_DATA_Z_MSB_ADDR           8'h13

//GYRO DATA REGISTERS
`define BNO055_GYRO_DATA_X_LSB_ADDR          8'h14
`define BNO055_GYRO_DATA_X_MSB_ADDR          8'h15
`define BNO055_GYRO_DATA_Y_LSB_ADDR          8'h16
`define BNO055_GYRO_DATA_Y_MSB_ADDR          8'h17
`define BNO055_GYRO_DATA_Z_LSB_ADDR          8'h18
`define BNO055_GYRO_DATA_Z_MSB_ADDR          8'h19

//EULER DATA REGISTERS
`define BNO055_EULER_H_LSB_ADDR              8'h1A //  Z-axis, Heading
`define BNO055_EULER_H_MSB_ADDR              8'h1B
`define BNO055_EULER_R_LSB_ADDR              8'h1C //  X-Axis, Roll
`define BNO055_EULER_R_MSB_ADDR              8'h1D
`define BNO055_EULER_P_LSB_ADDR              8'h1E //  Y-Axis, Pitch
`define BNO055_EULER_P_MSB_ADDR              8'h1F

//QUATERNION DATA REGISTERS
`define BNO055_QUATERNION_DATA_W_LSB_ADDR    8'h20
`define BNO055_QUATERNION_DATA_W_MSB_ADDR    8'h21
`define BNO055_QUATERNION_DATA_X_LSB_ADDR    8'h22
`define BNO055_QUATERNION_DATA_X_MSB_ADDR    8'h23
`define BNO055_QUATERNION_DATA_Y_LSB_ADDR    8'h24
`define BNO055_QUATERNION_DATA_Y_MSB_ADDR    8'h25
`define BNO055_QUATERNION_DATA_Z_LSB_ADDR    8'h26
`define BNO055_QUATERNION_DATA_Z_MSB_ADDR    8'h27

//LINEAR ACCELERATION DATA REGISTERS
`define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  8'h28
`define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR  8'h29
`define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR  8'h2A
`define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR  8'h2B
`define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR  8'h2C
`define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR  8'h2D

//GRAVITY DATA REGISTERS
`define BNO055_GRAVITY_DATA_X_LSB_ADDR       8'h2E
`define BNO055_GRAVITY_DATA_X_MSB_ADDR       8'h2F
`define BNO055_GRAVITY_DATA_Y_LSB_ADDR       8'h30
`define BNO055_GRAVITY_DATA_Y_MSB_ADDR       8'h31
`define BNO055_GRAVITY_DATA_Z_LSB_ADDR       8'h32
`define BNO055_GRAVITY_DATA_Z_MSB_ADDR       8'h33

//TEMPERATURE DATA REGISTER
`define BNO055_TEMP_ADDR                     8'h34

//STATUS REGISTERS
`define BNO055_CALIB_STAT_ADDR               8'h35
`define BNO055_SELFTEST_RESULT_ADDR          8'h36
`define BNO055_INTR_STAT_ADDR                8'h37

`define BNO055_SYS_CLK_STAT_ADDR             8'h38
`define BNO055_SYS_STAT_ADDR                 8'h39
`define BNO055_SYS_ERR_ADDR                  8'h3A

//UNIT SELECTION REGISTER
`define BNO055_UNIT_SEL_ADDR                 8'h3B
`define BNO055_DATA_SELECT_ADDR              8'h3C

//MODE REGISTERS
`define BNO055_OPR_MODE_ADDR                 8'h3D
`define BNO055_PWR_MODE_ADDR                 8'h3E

`define BNO055_SYS_TRIGGER_ADDR              8'h3F
`define BNO055_TEMP_SOURCE_ADDR              8'h40

//AXIS REMAP REGISTERS
`define BNO055_AXIS_MAP_CONFIG_ADDR          8'h41
`define BNO055_AXIS_MAP_SIGN_ADDR            8'h42

//SIC REGISTERS
`define BNO055_SIC_MATRIX_0_LSB_ADDR         8'h43
`define BNO055_SIC_MATRIX_0_MSB_ADDR         8'h44
`define BNO055_SIC_MATRIX_1_LSB_ADDR         8'h45
`define BNO055_SIC_MATRIX_1_MSB_ADDR         8'h46
`define BNO055_SIC_MATRIX_2_LSB_ADDR         8'h47
`define BNO055_SIC_MATRIX_2_MSB_ADDR         8'h48
`define BNO055_SIC_MATRIX_3_LSB_ADDR         8'h49
`define BNO055_SIC_MATRIX_3_MSB_ADDR         8'h4A
`define BNO055_SIC_MATRIX_4_LSB_ADDR         8'h4B
`define BNO055_SIC_MATRIX_4_MSB_ADDR         8'h4C
`define BNO055_SIC_MATRIX_5_LSB_ADDR         8'h4D
`define BNO055_SIC_MATRIX_5_MSB_ADDR         8'h4E
`define BNO055_SIC_MATRIX_6_LSB_ADDR         8'h4F
`define BNO055_SIC_MATRIX_6_MSB_ADDR         8'h50
`define BNO055_SIC_MATRIX_7_LSB_ADDR         8'h51
`define BNO055_SIC_MATRIX_7_MSB_ADDR         8'h52
`define BNO055_SIC_MATRIX_8_LSB_ADDR         8'h53
`define BNO055_SIC_MATRIX_8_MSB_ADDR         8'h54

//ACCELEROMETER OFFSET REGISTERS
`define BNO055_ACCEL_OFFSET_X_LSB_ADDR       8'h55

`define BNO055_ACCEL_OFFSET_X_MSB_ADDR       8'h56
`define BNO055_ACCEL_OFFSET_Y_LSB_ADDR       8'h57
`define BNO055_ACCEL_OFFSET_Y_MSB_ADDR       8'h58
`define BNO055_ACCEL_OFFSET_Z_LSB_ADDR       8'h59
`define BNO055_ACCEL_OFFSET_Z_MSB_ADDR       8'h5A

//MAGNETOMETER OFFSET REGISTERS
`define BNO055_MAG_OFFSET_X_LSB_ADDR         8'h5B
`define BNO055_MAG_OFFSET_X_MSB_ADDR         8'h5C
`define BNO055_MAG_OFFSET_Y_LSB_ADDR         8'h5D
`define BNO055_MAG_OFFSET_Y_MSB_ADDR         8'h5E
`define BNO055_MAG_OFFSET_Z_LSB_ADDR         8'h5F
`define BNO055_MAG_OFFSET_Z_MSB_ADDR         8'h60

//GYROSCOPE OFFSET REGISTERS
`define BNO055_GYRO_OFFSET_X_LSB_ADDR        8'h61
`define BNO055_GYRO_OFFSET_X_MSB_ADDR        8'h62
`define BNO055_GYRO_OFFSET_Y_LSB_ADDR        8'h63
`define BNO055_GYRO_OFFSET_Y_MSB_ADDR        8'h64
`define BNO055_GYRO_OFFSET_Z_LSB_ADDR        8'h65
`define BNO055_GYRO_OFFSET_Z_MSB_ADDR        8'h66

//BNO055_REG
`define BNO055_ACCEL_RADIUS_LSB_ADDR         8'h67
`define BNO055_ACCEL_RADIUS_MSB_ADDR         8'h68
`define BNO055_MAG_RADIUS_LSB_ADDR           8'h69
`define BNO055_MAG_RADIUS_MSB_ADDR           8'h6A
//Offsets 8'h6B through 8'h7F are reserved
//PAGE0 REGISTER DEFINITION END*/

//PAGE1 REGISTER DEFINITION START*/
// Sensor specific configuration data
// Offsets 0 - 6 are reserved
`define BNO055_PAGE_ID_ADDR                  8'h07 //Defaults to page 0 - This register is on page 0 and page 1 at the same location
`define BNO055_ACC_CONFIG_REG                8'h08
`define BNO055_MAG_CONFIG                    8'h09
`define BNO055_GYR_CONFIG_0                  8'h0A
`define BNO055_GYR_CONFIG_1                  8'h0B
`define BNO055_ACC_SLEEP_CONFIG              8'h0C
`define BNO055_GYR_SLEEP_CONFIG              8'h0D
`define BNO055_INT_MSK                       8'h0F
`define BNO055_INT_EN                        8'h10
`define BNO055_ACC_AM_THRES                  8'h11
`define BNO055_ACC_INT_SETTINGS              8'h12
`define BNO055_ACC_HG_DURATION               8'h13
`define BNO055_ACC_HG_THRES                  8'h14
`define BNO055_ACC_NM_THRES                  8'h15
`define BNO055_ACC_NM_SET                    8'h16
`define BNO055_GYR_INT_SETTING               8'h17
`define BNO055_GYR_HR_X_SET                  8'h18
`define BNO055_GYR_DUR_X                     8'h19
`define BNO055_GYR_HR_Y_SET                  8'h1A
`define BNO055_GYR_DUR_Y                     8'h1B
`define BNO055_GYR_HR_Z_SET                  8'h1C
`define BNO055_GYR_DUR_Z                     8'h1D
`define BNO055_GYR_AM_THRES                  8'h1E
`define BNO055_GYR_AM_SET                    8'h1F
//Offsets 8'h20 through 8'h4F are reserved
//Offsets 8'h50 - 8'h5F are for "UNIQUE_ID"
//Offsets 8'h60 through 8'h7F are reserved
//PAGE1 REGISTER DEFINITION END*/


//                  CONSTANTS                  //
// BNO055_POWERMODE
`define BNO055_POWER_MODE_NORMAL             8'h00// DEFAULT
`define BNO055_POWER_MODE_LOWPOWER           8'h01
`define BNO055_POWER_MODE_SUSPEND            8'h02

//BNO055_OPMODE
`define BNO055_OPERATION_MODE_CONFIG         8'h00// DEFAULT
`define BNO055_OPERATION_MODE_ACCONLY        8'h01
`define BNO055_OPERATION_MODE_MAGONLY        8'h02
`define BNO055_OPERATION_MODE_GYRONLY        8'h03
`define BNO055_OPERATION_MODE_ACCMAG         8'h04
`define BNO055_OPERATION_MODE_ACCGYRO        8'h05
`define BNO055_OPERATION_MODE_MAGGYRO        8'h06
`define BNO055_OPERATION_MODE_AMG            8'h07
`define BNO055_OPERATION_MODE_IMUPLUS        8'h08
`define BNO055_OPERATION_MODE_COMPASS        8'h09
`define BNO055_OPERATION_MODE_M4G            8'h0A
`define BNO055_OPERATION_MODE_NDOF_FMC_OFF   8'h0B
`define BNO055_OPERATION_MODE_NDOF           8'h0C

//BNO055_AXIS_REMAP_CONFIG
`define BNO055_REMAP_CONFIG_P0               8'h21
`define BNO055_REMAP_CONFIG_P1               8'h24// DEFAULT
`define BNO055_REMAP_CONFIG_P2               8'h24
`define BNO055_REMAP_CONFIG_P3               8'h21
`define BNO055_REMAP_CONFIG_P4               8'h24
`define BNO055_REMAP_CONFIG_P5               8'h21
`define BNO055_REMAP_CONFIG_P6               8'h21
`define BNO055_REMAP_CONFIG_P7               8'h24

//BNO055_AXIS_REMAP_SIGN
`define BNO055_REMAP_SIGN_P0                 8'h04
`define BNO055_REMAP_SIGN_P1                 8'h00// DEFAULT
`define BNO055_REMAP_SIGN_P2                 8'h06
`define BNO055_REMAP_SIGN_P3                 8'h02
`define BNO055_REMAP_SIGN_P4                 8'h03
`define BNO055_REMAP_SIGN_P5                 8'h01
`define BNO055_REMAP_SIGN_P6                 8'h07
`define BNO055_REMAP_SIGN_P7                 8'h05


//Indices of measurement data in rx_data_reg
`define ACC_DATA_X_LSB_INDEX                 0
`define ACC_DATA_X_MSB_INDEX                 1
`define ACC_DATA_Y_LSB_INDEX                 2
`define ACC_DATA_Y_MSB_INDEX                 3
`define ACC_DATA_Z_LSB_INDEX                 4
`define ACC_DATA_Z_MSB_INDEX                 5
`define MAG_DATA_X_LSB_INDEX                 6
`define MAG_DATA_X_MSB_INDEX                 7
`define MAG_DATA_Y_LSB_INDEX                 8
`define MAG_DATA_Y_MSB_INDEX                 9
`define MAG_DATA_Z_LSB_INDEX                 10
`define MAG_DATA_Z_MSB_INDEX                 11
`define GYR_DATA_X_LSB_INDEX                 12
`define GYR_DATA_X_MSB_INDEX                 13
`define GYR_DATA_Y_LSB_INDEX                 14
`define GYR_DATA_Y_MSB_INDEX                 15
`define GYR_DATA_Z_LSB_INDEX                 16
`define GYR_DATA_Z_MSB_INDEX                 17
//  **** NOTE ****
//  These do not follow the same sequence as other registers, instead of X, Y, Z, these are sequenced, Z, X, Y
`define EUL_DATA_Z_LSB_INDEX                 18 //  Yaw/Heading, Z-Axis
`define EUL_DATA_Z_MSB_INDEX                 19
`define EUL_DATA_X_LSB_INDEX                 20 //  Roll, X-Axis
`define EUL_DATA_X_MSB_INDEX                 21
`define EUL_DATA_Y_LSB_INDEX                 22 //  Pitch, Y-Axis
`define EUL_DATA_Y_MSB_INDEX                 23
//  **** /END NOTE ****
`define QUA_DATA_W_LSB_INDEX                 24
`define QUA_DATA_W_MSB_INDEX                 25
`define QUA_DATA_X_LSB_INDEX                 26
`define QUA_DATA_X_MSB_INDEX                 27
`define QUA_DATA_Y_LSB_INDEX                 28
`define QUA_DATA_Y_MSB_INDEX                 29
`define QUA_DATA_Z_LSB_INDEX                 30
`define QUA_DATA_Z_MSB_INDEX                 31
`define LIN_DATA_X_LSB_INDEX                 32
`define LIN_DATA_X_MSB_INDEX                 33
`define LIN_DATA_Y_LSB_INDEX                 34
`define LIN_DATA_Y_MSB_INDEX                 35
`define LIN_DATA_Z_LSB_INDEX                 36
`define LIN_DATA_Z_MSB_INDEX                 37
`define GRA_DATA_X_LSB_INDEX                 38
`define GRA_DATA_X_MSB_INDEX                 39
`define GRA_DATA_Y_LSB_INDEX                 40
`define GRA_DATA_Y_MSB_INDEX                 41
`define GRA_DATA_Z_LSB_INDEX                 42
`define GRA_DATA_Z_MSB_INDEX                 43
`define TEMPERATURE_DATA_INDEX               44
`define CALIBRATION_DATA_INDEX               45
/*

Registers accessed for data collection polling:
Accelerometer Raw Data 3-axis, 6 bytes - Meters/Sec^2, Precision: 1 m/s^2 = 100 LSB
Section 4.3.9  ACC_DATA_X_LSB 0X08 Data Sheet, Page:56
Section 4.3.10 ACC_DATA_X_MSB 0X09 Data Sheet, Page:56
Section 4.3.11 ACC_DATA_Y_LSB 0X0A Data Sheet, Page:56
Section 4.3.12 ACC_DATA_Y_MSB 0X0B Data Sheet, Page:56
Section 4.3.13 ACC_DATA_Z_LSB 0X0C Data Sheet, Page:57
Section 4.3.14 ACC_DATA_Z_MSB 0X0D Data Sheet, Page:57

Magnetometer Raw Data 3-axis, 6 bytes - uT, Precision: 1uT = 16 LSB
Section 4.3.15 MAG_DATA_X_LSB 0X0E Data Sheet, Page:57
Section 4.3.16 MAG_DATA_X_MSB 0X0F Data Sheet, Page:57
Section 4.3.17 MAG_DATA_Y_LSB 0X10 Data Sheet, Page:58
Section 4.3.18 MAG_DATA_Y_MSB 0X11 Data Sheet, Page:58
Section 4.3.19 MAG_DATA_Z_LSB 0X12 Data Sheet, Page:58
Section 4.3.20 MAG_DATA_Z_MSB 0X13 Data Sheet, Page:58

Gyroscope Raw Data 3-axis, 6 bytes - Degrees/Sec, Precision: Dps = 16 LSB
Section 4.3.21 GYR_DATA_X_LSB 0X14 Data Sheet, Page:59
Section 4.3.22 GYR_DATA_X_MSB 0X15 Data Sheet, Page:59
Section 4.3.23 GYR_DATA_Y_LSB 0X16 Data Sheet, Page:59
Section 4.3.24 GYR_DATA_Y_MSB 0X17 Data Sheet, Page:59
Section 4.3.25 GYR_DATA_Z_LSB 0X18 Data Sheet, Page:60
Section 4.3.26 GYR_DATA_Z_MSB 0X19 Data Sheet, Page:60

Fusion calculated Euler angles 3-axis, 6 bytes - Degrees, Precision: Deg = 16 LSB
**** NOTE ****
These do not follow the same sequence as other registers, instead of X, Y, Z, these are sequenced, Z, X, Y
Section 4.3.27 EUL_DATA_Z_LSB 0X1A Data Sheet, Page:60  Yaw/Heading, Z-Axis
Section 4.3.28 EUL_DATA_Z_MSB 0X1B Data Sheet, Page:60
Section 4.3.29 EUL_DATA_X_LSB 0X1C Data Sheet, Page:61  Roll, X-Axis
Section 4.3.30 EUL_DATA_X_MSB 0X1D Data Sheet, Page:61
Section 4.3.31 EUL_DATA_Y_LSB 0X1E Data Sheet, Page:61  Pitch, Y-Axis
Section 4.3.32 EUL_DATA_Y_MSB 0X1F Data Sheet, Page:61

Quaternions, 4-axis, 8 bytes - Quaternion units, Precision: Unit = 2^14 LSB
Section 4.3.33 QUA_DATA_W_LSB 0X20 Data Sheet, Page:62
Section 4.3.34 QUA_DATA_W_MSB 0X21 Data Sheet, Page:62
Section 4.3.35 QUA_DATA_X_LSB 0X22 Data Sheet, Page:62
Section 4.3.36 QUA_DATA_X_MSB 0X23 Data Sheet, Page:62
Section 4.3.37 QUA_DATA_Y_LSB 0X24 Data Sheet, Page:63
Section 4.3.38 QUA_DATA_Y_MSB 0X25 Data Sheet, Page:63
Section 4.3.39 QUA_DATA_Z_LSB 0X26 Data Sheet, Page:63
Section 4.3.40 QUA_DATA_Z_MSB 0X27 Data Sheet, Page:63


Fusion Calculated Linear Acceleration (Excludes gravity vector) 3-axis, 6 bytes = Meters/Sec^2, Precision: 1 m/s^2 = 100 LSB
Section 4.3.41 LIA_DATA_X_LSB 0X28 Data Sheet, Page:64
Section 4.3.42 LIA_DATA_X_MSB 0X29 Data Sheet, Page:64
Section 4.3.43 LIA_DATA_Y_LSB 0X2A Data Sheet, Page:64
Section 4.3.44 LIA_DATA_Y_MSB 0X2B Data Sheet, Page:64
Section 4.3.45 LIA_DATA_Z_LSB 0X2C Data Sheet, Page:65
Section 4.3.46 LIA_DATA_Z_MSB 0X2D Data Sheet, Page:65

Fusion Calculated Gravity Acceleration (Excludes linear acceleration vetor) 3-axis, 6 bytes = Meters/Sec^2, Precision: 1 m/s^2 = 100 LSB
Section 4.3.47 GRV_DATA_X_LSB 0X2E Data Sheet, Page:65
Section 4.3.48 GRV_DATA_X_MSB 0X2F Data Sheet, Page:65
Section 4.3.49 GRV_DATA_Y_LSB 0X30 Data Sheet, Page:66
Section 4.3.50 GRV_DATA_Y_MSB 0X31 Data Sheet, Page:66
Section 4.3.51 GRV_DATA_Z_LSB 0X32 Data Sheet, Page:66
Section 4.3.52 GRV_DATA_Z_MSB 0X33 Data Sheet, Page:66


Temperature - Degrees Celsius
Section 4.3.53 TEMP           0X34 Data Sheet, Page:67
--------------------------------------------------------------------------------------------------------------
Data				Register		Description
Bits				Bits
--------------------------------------------------------------------------------------------------------------
SYS Calib Status	<7:6>		Current system calibration status, depends on status of all sensors, read-only
<0:1>                    		Read: 3 indicates fully calibrated; 0 indicates not calibrated
GYR Calib Status	<5:4>		Current calibration status of Gyroscope, read-only
<0:1>                    		Read: 3 indicates fully calibrated; 0 indicates not calibrated
ACC Calib Status	<3:2>		Current calibration status of Accelerometer, read-only
<0:1>                    		Read: 3 indicates fully calibrated; 0 indicates not calibrated
MAG Calib Status	<1:0>		Current calibration status of Magnetometer, read-only
<0:1>                    		Read: 3 indicates fully calibrated; 0 indicates not calibrated

Section 4.3.54 CALIB_STAT     0X35 Data Sheet, Page:67

Total: 46 bytes

*/

//Indices of calibration data in cal_data_reg
`define ACCEL_OFFSET_X_LSB_INDEX 0
`define ACCEL_OFFSET_X_MSB_INDEX 1
`define ACCEL_OFFSET_Y_LSB_INDEX 2
`define ACCEL_OFFSET_Y_MSB_INDEX 3
`define ACCEL_OFFSET_Z_LSB_INDEX 4
`define ACCEL_OFFSET_Z_MSB_INDEX 5
`define MAG_OFFSET_X_LSB_INDEX   6
`define MAG_OFFSET_X_MSB_INDEX   7
`define MAG_OFFSET_Y_LSB_INDEX   8
`define MAG_OFFSET_Y_MSB_INDEX   9
`define MAG_OFFSET_Z_LSB_INDEX   10
`define MAG_OFFSET_Z_MSB_INDEX   11
`define GYRO_OFFSET_X_LSB_INDEX  12
`define GYRO_OFFSET_X_MSB_INDEX  13
`define GYRO_OFFSET_Y_LSB_INDEX  14
`define GYRO_OFFSET_Y_MSB_INDEX  15
`define GYRO_OFFSET_Z_LSB_INDEX  16
`define GYRO_OFFSET_Z_MSB_INDEX  17
`define ACCEL_RADIUS_LSB_INDEX   18
`define ACCEL_RADIUS_MSB_INDEX   19
`define MAG_RADIUS_LSB_INDEX     20
`define MAG_RADIUS_MSB_INDEX     21


`define BNO055_STATE_BITS      5             //  The number of bits used to represent the current state
`define DATA_RX_BYTE_REG_CNT   46            //  The number of byte registers used to receive all measurement data
`define CAL_DATA_REG_CNT       22            //  The number of byte registers used to store calibration data

// Value aliases
`define GO                      1'b1         //  Go signal to i2c driver is logic high
`define NOT_GO                  1'b0         //  Go signal to i2c driver is logic low
`define SUB_STATE_GO            1'b1         //  Go signal to sub FSM is logic high
`define SUB_STATE_NOT_GO        1'b0         //  Go signal to sub FSM is logic low
`define SUB_STATE_DONE          1'b1         //  Signal that the sub FSM is done
`define SUB_STATE_NOT_DONE      1'b0         //  Signal that the sub FSM is done
`define I2C_READ                1'b0         //  an I2C Read command
`define I2C_WRITE               1'b1         //  An I2C Write command
`define BNO055_CHIP_ID_REG      8'h00        //  Chip ID Register of BNO055 at 0x00
`define BNO055_SLAVE_ADDRESS    7'b01010_00  //  BNO055 SLAVE address 0x28


`define RUN_MS_TIMER            1'b1         //  Flag that starts multi ms timer
`define CLEAR_MS_TIMER          1'b0         //  Flag that stops/clears multi ms timer


// 
//  State Definitions
// 
// Initial default state of IMU FSM


`define BNO055_STATE_RESET               0
`define BNO055_STATE_BOOT                1
`define BNO055_STATE_BOOT_WAIT           2

`define BNO055_STATE_READ_CHIP_ID        3
`define BNO055_STATE_SET_EXT_CRYSTAL     4
`define BNO055_STATE_SET_UNITS           5
`define BNO055_STATE_SET_POWER_MODE      6
`define BNO055_STATE_CAL_RESTORE_DATA    7
`define BNO055_STATE_CAL_RESTORE_START   8
`define BNO055_STATE_CAL_RESTORE_WAIT    9
`define BNO055_STATE_CAL_RESTORE_STOP    10
`define BNO055_STATE_CAL_RESTORE_AGAIN   11
`define BNO055_STATE_SET_RUN_MODE        12
`define BNO055_STATE_WAIT_20MS           13
`define BNO055_STATE_READ_IMU_DATA_BURST 14
`define BNO055_STATE_WAIT_10MS           15


`define BNO055_SUB_STATE_START           16
`define BNO055_SUB_STATE_WAIT_I2C        17
`define BNO055_SUB_STATE_STOP            18