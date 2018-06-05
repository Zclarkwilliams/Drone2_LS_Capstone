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
 * common_defines - Contains defines that will be used throughout multiple modules of the
 *					Drone2 project.
 */

// Produce an error if the net type isn't defined
// This was incorrect, uncomment once files are fixed
//`default_nettype none

// Signal levels
`define HIGH	1'b1
`define LOW 	1'b0

// Conditionals
`define TRUE	1'b1
`define FALSE	1'b0

// Mathematical bit addition
`define ONE		1'b1

// MAX MIN fixed point shift in decimal
`define FIXED_POINT_SHIFT		4'sd4
`define OPS_BIT_WIDTH			7'd32
`define SHIFT_OP_BIT_WIDTH		3'd4
`define	BITS_EXTRACT			6'd16
`define PADDING_ZEROS 			16'd0

//  A byte of all zeros
`define BYTE_ALL_ZERO           8'sh00
`define ALL_ZERO_2BYTE          16'sh0000

// PWM signal pulse lengths
`define MIN_PWM_TIME_HIGH_US	16'd1000
`define MAX_PWM_TIME_HIGH_US	16'd2000
`define PWM_PERIOD_US			16'd20000

//  Clock dividers and system clock frequency
`define SYS_CLK_FREQ            38_000_000                    //  Frequency of system clock, in Hz
`define I2C_CLK_FREQ            400_000                       //  Frequency of i2c scl clock, in Hz
`define WAIT_MS_DIVIDER         (`SYS_CLK_FREQ/1_000)         //  Number of system clock ticks per ms
`define WAIT_US_DIVIDER         (`SYS_CLK_FREQ/1_000_000)     //  Number of system clock ticks per us
`define WAIT_I2C_DIVIDER        (`I2C_CLK_FREQ/`SYS_CLK_FREQ) //  Number of system clock ticks per i2c scl tick, at i2c rate

// Bit widths
`define PWM_TIME_BIT_WIDTH		5'd16	// Sized to hold the MAX_PWM_TIME_HIGH_US
`define PWM_VALUE_BIT_WIDTH 	4'd8	// Sized to hold the mapped pwm value

`define MOTOR_RATE_BIT_WIDTH	4'd8

`define REC_VAL_BIT_WIDTH		4'd8	// Sized to hold outputs of receiver
`define PID_RATE_BIT_WIDTH		5'd16	// Sized to hold values from body frame controller
`define	RATE_BIT_WIDTH			5'd16	// Sized to hold values from the angle controller
`define IMU_VAL_BIT_WIDTH		5'd16	// Sized to hold values from the bno055_driver

`define DEBUG_WIRE_BIT_WIDTH	9'd16	// DEBUG LED wire bit width to all modules

// Default values (on reset or error) for pwm pulse high values in microseconds
`define THROTTLE_DEFAULT_PULSE_TIME_HIGH_US  16'd1000
`define YAW_DEFAULT_PULSE_TIME_HIGH_US		 16'd1500
`define ROLL_DEFAULT_PULSE_TIME_HIGH_US		 16'd1500
`define PITCH_DEFAULT_PULSE_TIME_HIGH_US	 16'd1500
`define AUX1_DEFAULT_PULSE_TIME_HIGH_US	 	 16'd1500
`define AUX2_DEFAULT_PULSE_TIME_HIGH_US	 	 16'd1500
`define SWAB_DEFAULT_PULSE_TIME_HIGH_US	 	 16'd1500

/************************************************************ *
 *	Motor_Mixer Module Defines:
 *
 ************************************************************/

//	Min and Max boundary values for to send to ESC's
`define MOTOR_VAL_MIN					16'sh0020
`define MOTOR_VAL_MAX					16'sh0FA0

//	Value to add to value before bit shift rounding to help rounding error
`define MOTOR_RATE_ROUND_UP_VAL 		8'd127