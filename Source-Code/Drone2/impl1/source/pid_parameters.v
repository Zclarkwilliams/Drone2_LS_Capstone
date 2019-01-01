/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell,
 * Brett Creeley,
 * Daniel Christiansen,
 * Kirk Hooper,
 * Zachary Clark-Williams
 */

`include "common_defines.v"

/**
 * pid_parameters.v - File that contains various tuning parameters for
 * angle_controller and the body_frame_controller.
 *
 * PID controller proportional/integral/derivative constant values.
 * These are determined by first multiplying the value by the specific
 * K_* term and then shifting it using the K_*_SHIFT value.
 * Example: value = (value * ROLL_K_P) >>> ROLL_K_P_SHIFT;
 */
 
 /**
 * Defines for the yaw_angle_accumulator
 */
 
//Adjust yaw rate from stick input,
//increasing value increases yaw stick rate, minimum value is 1
`define YAW_ACCUMULATOR_INPUT_MULTIPLIER    16'sd4 

/**
 * Defines for the angle_controller
 */

// Multiplier Scalar Values
`define YAW_OLD_AC_K_P           16'sd60	//Yaw Kp parameter for AC without Yaw Accumulator module
`define YAW_YAAC_AC_K_P          16'sd40 	//Yaw Kp parameter for AC WITH Yaw Accumulator module
`define ROLL_AC_K_P              16'sd36
`define PITCH_AC_K_P             16'sd32
`define THROTTLE_AC_K_P          16'sd1

// Divisor Shift Values
`define YAW_AC_K_P_SHIFT         4'd4
`define PITCH_AC_K_P_SHIFT       4'd4
`define ROLL_AC_K_P_SHIFT        4'd4
`define THROTTLE_AC_K_P_SHIFT    4'd0

/**
 * Defines for the body_frame_controller
 */

// Multiply and shifts to create YAW PID constants
`define YAW_K_P                  16'sd10
`define YAW_K_I                  16'sd0
`define YAW_K_D                  16'sd0
`define YAW_K_P_SHIFT            4'd4
`define YAW_K_I_SHIFT            4'd4
`define YAW_K_D_SHIFT            4'd4

// Multiply and shifts to create ROLL PID constants
`define ROLL_K_P                 16'sd5
`define ROLL_K_I                 16'sd3
`define ROLL_K_D                 16'sd4
`define ROLL_K_P_SHIFT           4'd4
`define ROLL_K_I_SHIFT           4'd4
`define ROLL_K_D_SHIFT           4'd4

// Multiply and shifts to create PITCH PID constants
`define PITCH_K_P                16'sd5
`define PITCH_K_I                16'sd3
`define PITCH_K_D                16'sd4
`define PITCH_K_P_SHIFT          4'd4
`define PITCH_K_I_SHIFT          4'd4
`define PITCH_K_D_SHIFT          4'd4

