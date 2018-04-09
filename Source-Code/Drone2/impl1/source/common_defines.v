/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

/**
 * common_defines - Contains defines that will be used throughout multiple modules of the
 *					Drone2 project.
 */

// Produce an error if the net type isn't defined
`define default nettype none

// Signal levels
`define HIGH	1'b1
`define LOW 	1'b0

// Conditionals
`define TRUE	1'b1
`define FALSE	1'b0

//  A byte of all zeros
`define BYTE_ALL_ZERO           8'h00

// PWM signal pulse lengths
`define MIN_PWM_TIME_HIGH_US	10'd1000
`define MAX_PWM_TIME_HIGH_US	11'd2000
`define PWM_PERIOD_US			15'd20000

//  Clock dividers and system clock frequency
`define SYS_CLK_FREQ            38_000_000                    //  Frequency of system clock, in Hz
`define I2C_CLK_FREQ            400_000                       //  Frequency of i2c scl clock, in Hz
`define WAIT_MS_DIVIDER         (`SYS_CLK_FREQ/1_000)         //  Number of system clock ticks per ms
`define WAIT_US_DIVIDER         (`SYS_CLK_FREQ/1_000_000)     //  Number of system clock ticks per us
`define WAIT_I2C_DIVIDER        (`I2C_CLK_FREQ/`SYS_CLK_FREQ) //  Number of system clock ticks per i2c scl tick, at i2c rate

// Bit widths
`define PWM_TIME_BIT_WIDTH	4'd11	// Sized to hold the MAX_PWM_TIME_HIGH_US
`define PWM_VALUE_BIT_WIDTH 4'd8	// Sized to hold the mapped pwm value

`define MOTOR_RATE_BIT_WIDTH 4'd8

// Default values (on reset or error) for pwm pulse high values in microseconds
`define THROTTLE_DEFAULT_PULSE_TIME_HIGH_US 11'd1000
`define YAW_DEFAULT_PULSE_TIME_HIGH_US		11'd1500
`define ROLL_DEFAULT_PULSE_TIME_HIGH_US		11'd1500
`define PITCH_DEFAULT_PULSE_TIME_HIGH_US	11'd1500
