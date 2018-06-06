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
 * rec_data_buffer - This buffer controls the reciever to angle_controller bus. The
 * 					 control signal is from the flight_mode module and cycles according
 *					 to swa, and swb from user remote control and internal resets.
 *
 * Outputs:
 * @ yaw_buffer_val:		This will be the yaw values given to angle_controller
 * @ roll_buffer_val:		This will be the roll values given to angle_controller
 * @ pitch_buffer_val:		This will be the pitch values given to angle_controller
 * @ throttle_buffer_val:	This will be the throttle values given to angle_controller
 *
 * Inputs:
 * @ yaw_rec_val:			This is the yaw values from the reciever module 	 8 bit
 * @ roll_rec_val:			This is the roll values from the reciever module 	 8 bit
 * @ pitch_rec_val:			This is the pitch values from the reciever module 	 8 bit
 * @ throttle_rec_val:		This is the throttle values from the reciever module 8 bit 
 * @ curr_motor_rate:		This is the motor_rate output from the motor_mixer 	 8 bit
 * @ rec_data_sel:			This is the data bus output control signal			 4 bit
 * @ us_clk:				clock signal
 */
 
`timescale 1ns / 1ns
`include "common_defines.v"

module rec_data_buffer	(
						 // Output
						 output reg [`PWM_VALUE_BIT_WIDTH-1:0] yaw_val_buff,
						 output reg [`PWM_VALUE_BIT_WIDTH-1:0] roll_val_buff,
						 output reg [`PWM_VALUE_BIT_WIDTH-1:0] pitch_val_buff,
						 output reg [`PWM_VALUE_BIT_WIDTH-1:0] throttle_val_buff,
						 // Input
						 input wire [`PWM_VALUE_BIT_WIDTH-1:0] yaw_rec_val,
						 input wire [`PWM_VALUE_BIT_WIDTH-1:0] roll_rec_val,
						 input wire [`PWM_VALUE_BIT_WIDTH-1:0] pitch_rec_val,
						 input wire [`PWM_VALUE_BIT_WIDTH-1:0] throttle_rec_val,
						 input wire [`PWM_VALUE_BIT_WIDTH-1:0] curr_motor_rate,
						 input wire [`REC_DATA_SEL_BIT_WIDTH-1:0] rec_data_sel,
						 input wire us_clk);
	
	// Parameter for timer counters to increase throttle during takeoff
	localparam TIMER_BIT_WIDTH = 7'd17;
	localparam signed [TIMER_BIT_WIDTH-1:0]
		TIMER_ZERO 		 = 0,		// When Timer is Zero match size to ensure correct
		TAKE_OFF_TIMER_1 = 8695,	// 0   -> 115 in 1    second
		TAKE_OFF_TIMER_2 = 5000,	// 115 -> 165 in 0.25 seconds
		TAKE_OFF_TIMER_3 = 50000;	// 165 -> 185 in 1    seconds
	
	// Parameters for 3 stage throttle ramp during auto-takeoff
	localparam [`REC_VAL_BIT_WIDTH-1:0]
		TAKEOFF_RAMP_THROTTLE_VAL_1 = 115,
		TAKEOFF_RAMP_THROTTLE_VAL_2 = 165,
		TAKEOFF_RAMP_THROTTLE_VAL_3 = 185;
	
	// Timer to set to be able to ramp throttle in 3 stages
	reg signed [16:0] timer_throttle_ramp;
	
	always @(posedge us_clk) begin
		case(rec_data_sel)
			`REC_SEL_OFF:			begin
				throttle_val_buff		<= `BYTE_ALL_ZERO;
				yaw_val_buff			<= `BYTE_ALL_ZERO;
				roll_val_buff			<= `BYTE_ALL_ZERO;
				pitch_val_buff			<= `BYTE_ALL_ZERO;
			end
			`REC_SEL_AUTO_TAKE_OFF:	begin
				// stage 1 throttle 0 -> 115 aka 0 -> 45%
				if (throttle_val_buff < TAKEOFF_RAMP_THROTTLE_VAL_1) begin
					if (timer_throttle_ramp == TIMER_ZERO) begin
						throttle_val_buff	<= curr_motor_rate + `ONE;
						timer_throttle_ramp <= TAKE_OFF_TIMER_1;
						end
					else
						timer_throttle_ramp <= timer_throttle_ramp - `ONE;
				end
				// stage 2 throttle 115 -> 165 aka 45 -> 65%
				else if (throttle_val_buff < TAKEOFF_RAMP_THROTTLE_VAL_2) begin
					if (timer_throttle_ramp == TIMER_ZERO) begin
						throttle_val_buff	<= curr_motor_rate + `ONE;
						timer_throttle_ramp <= TAKE_OFF_TIMER_2;
						end
					else
						timer_throttle_ramp <= timer_throttle_ramp - `ONE;
				end
				// stage 3 throttle 165 -> 185 aka 65 -> 75%
				else if (throttle_val_buff < TAKEOFF_RAMP_THROTTLE_VAL_3) begin
					if (timer_throttle_ramp == TIMER_ZERO) begin
						throttle_val_buff	<= curr_motor_rate + `ONE;
						timer_throttle_ramp <= TAKE_OFF_TIMER_3;
						end
					else
						timer_throttle_ramp <= timer_throttle_ramp - `ONE;
				end
				else //Now that 3rd stage has been reached stay at hover_throttle_val output
					throttle_val_buff	<= `HOVER_THROTTLE_VAL;
				// all other target values held to zero to instill stable flight
				yaw_val_buff			<= `BYTE_ALL_ZERO;
				roll_val_buff			<= `BYTE_ALL_ZERO;
				pitch_val_buff			<= `BYTE_ALL_ZERO;
			end
			`REC_SEL_HOVER:			begin
				throttle_val_buff		<= `HOVER_THROTTLE_VAL;
				yaw_val_buff			<= `BYTE_ALL_ZERO;
				roll_val_buff			<= `BYTE_ALL_ZERO;
				pitch_val_buff			<= `BYTE_ALL_ZERO;
			end
			`REC_SEL_PASS_THROUGH:	begin
				yaw_val_buff			<= yaw_rec_val;
				roll_val_buff			<= roll_rec_val;
				pitch_val_buff			<= pitch_rec_val;
				throttle_val_buff		<= throttle_rec_val;
			end
			`REC_SEL_AUTO_LAND:			begin
				/*if (curr_motor_rate > `MOTOR_VAL_MIN)
					throttle_val_buff	<= curr_motor_rate - `ONE;
				else 
					throttle_val_buff	<= `BYTE_ALL_ZERO;
				yaw_val_buff			<= `BYTE_ALL_ZERO;
				roll_val_buff			<= `BYTE_ALL_ZERO;
				pitch_val_buff			<= `BYTE_ALL_ZERO;
				*/
				//TODO: Get auto-land working but need height sensor or z-axis velocity sensor input
				yaw_val_buff			<= yaw_rec_val;
				roll_val_buff			<= roll_rec_val;
				pitch_val_buff			<= pitch_rec_val;
				throttle_val_buff		<= throttle_rec_val;
			end
			default: 			begin
				// should never reach here! So treat like restart
				yaw_val_buff			<= `BYTE_ALL_ZERO;
				roll_val_buff			<= `BYTE_ALL_ZERO;
				pitch_val_buff			<= `BYTE_ALL_ZERO;
				throttle_val_buff		<= `BYTE_ALL_ZERO;
			end
		endcase
	end
	
endmodule
