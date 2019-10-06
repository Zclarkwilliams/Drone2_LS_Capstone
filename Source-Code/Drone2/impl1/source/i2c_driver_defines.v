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


`define I2C_DRV_STATE_BITS                      5             //  The number of bits used to represent the current state
// Initial default state of IMU FSM
`define I2C_DRV_STATE_RESET                                 'd0
// The rest of the startup states
`define I2C_DRV_STATE_BOOT                                  'd1
`define I2C_DRV_STATE_BOOT_WAIT                             'd2
// Read ST VL53L1X chip ID and poll firmware ready
`define I2C_VL53L1X_STATE_READ_CHIP_ID                      'd3
`define I2C_VL53L1X_STATE_READ_FIRMWARE_READY               'd4
// Read Bosch BNO055 chip ID and begin setup
`define I2C_BNO055_STATE_READ_CHIP_ID                       'd5
`define I2C_BNO055_STATE_SET_UNITS                          'd6
`define I2C_BNO055_STATE_SET_POWER_MODE                     'd7
// Restore calibrations to VL53L1X and BNO055
`define I2C_STATE_CAL_RESTORE_DATA                          'd8
`define I2C_STATE_CAL_RESTORE_NEXT                          'd9
`define I2C_STATE_CAL_RESTORE_AGAIN                         'd10
// Continue Bosch BNO055 setup
`define I2C_BNO055_STATE_SET_EXT_CRYSTAL                    'd11
`define I2C_BNO055_STATE_SET_RUN_MODE                       'd12
`define I2C_STATE_WAIT_20MS                                 'd13
// Continue Configuration of VL53L1X
`define I2C_VL53L1X_STATE_INIT_START_MEASURE                'd14
`define I2C_VL53L1X_STATE_INIT_POLL_DATA_READY              'd15
`define I2C_VL53L1X_STATE_INIT_CLEAR_INTERRUPT              'd16
`define I2C_VL53L1X_STATE_INIT_STOP_MEASURE                 'd17
`define I2C_VL53L1X_STATE_SET_TEMPERATURE_MACRO_LOOP_BOUND  'd18
`define I2C_VL53L1X_STATE_SET_TEMPERATURE_VHV_CONFIG_INIT   'd19
`define I2C_VL53L1X_STATE_SET_MEASUREMENT_PERIOD_RX_OSC_VAL 'd20
`define I2C_VL53L1X_STATE_SET_MEASUREMENT_PERIOD_CALCULATE  'd21
`define I2C_VL53L1X_STATE_SET_MEASUREMENT_PERIOD_TX_PERIOD  'd22
`define I2C_VL53L1X_STATE_START_MEASURE                     'd23
// Begin reading from slaves
`define I2C_BNO055_STATE_READ_DATA_BURST                    'd24
`define I2C_VL53L1X_STATE_POLL_READY                        'd25
`define I2C_VL53L1X_STATE_GET_MEASUREMENT                   'd26
`define I2C_VL53L1X_STATE_CLEAR_INTERRUPT                   'd27
// Wait here for next polling interval
`define I2C_DRV_STATE_WAIT_IMU_POLL_TIME                    'd28

// Minor FSM states, repeated for every read or write
`define I2C_DRV_SUB_STATE_START                             'd29
`define I2C_DRV_SUB_STATE_WAIT_I2C                          'd30
`define I2C_DRV_SUB_STATE_STOP                              'd31