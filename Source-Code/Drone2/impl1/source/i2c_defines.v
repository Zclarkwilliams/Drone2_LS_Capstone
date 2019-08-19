/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell,
 * Brett Creeley,
 * Daniel Christiansen,
 * Kirk Hooper,
 * Zachary Clark-Williams
 */

/**
 *    I2C device driver definitions
 */

// Value aliases
`define GO                                  1'b1         //  Go signal to i2c driver is logic high
`define NOT_GO                              1'b0         //  Go signal to i2c driver is logic low
`define SUB_STATE_GO                        1'b1         //  Go signal to sub FSM is logic high
`define SUB_STATE_NOT_GO                    1'b0         //  Go signal to sub FSM is logic low
`define SUB_STATE_DONE                      1'b1         //  Signal that the sub FSM is done
`define SUB_STATE_NOT_DONE                  1'b0         //  Signal that the sub FSM is done
`define I2C_READ                            1'b0         //  an I2C Read command
`define I2C_WRITE                           1'b1         //  An I2C Write command

`define RUN_MS_TIMER                        1'b1         //  Flag that starts multi ms timer
`define CLEAR_MS_TIMER                      1'b0         //  Flag that stops/clears multi ms timer

// State Definitions
//


`define I2C_STATE_BITS                          5             //  The number of bits used to represent the current state
// Initial default state of IMU FSM
`define I2C_STATE_RESET                         0
// The rest of the startup states
`define I2C_STATE_BOOT                          1
`define I2C_STATE_BOOT_WAIT                     2

// Setup BNO055 and begin reading
`define I2C_BNO055_STATE_READ_CHIP_ID           3
`define I2C_BNO055_STATE_SET_UNITS              4
`define I2C_BNO055_STATE_SET_POWER_MODE         5
`define I2C_BNO055_STATE_CAL_RESTORE_DATA       6
`define I2C_BNO055_STATE_CAL_RESTORE_START      7
`define I2C_BNO055_STATE_CAL_RESTORE_WAIT       8
`define I2C_BNO055_STATE_CAL_RESTORE_STOP       9
`define I2C_BNO055_STATE_CAL_RESTORE_AGAIN      10
`define I2C_BNO055_STATE_SET_EXT_CRYSTAL        11
`define I2C_BNO055_STATE_SET_RUN_MODE           12
`define I2C_BNO055_STATE_WAIT_20MS              13
`define I2C_BNO055_STATE_READ_IMU_DATA_BURST    14
`define I2C_STATE_WAIT_IMU_POLL_TIME            15

// Minor FSM states, repeated for every read or write
`define I2C_SUB_STATE_START                     16
`define I2C_SUB_STATE_WAIT_I2C                  17
`define I2C_SUB_STATE_STOP                      18