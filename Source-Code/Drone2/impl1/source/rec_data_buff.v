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
						 output reg signed 	[`PWM_VALUE_BIT_WIDTH-1:0] 	yaw_val_buff,
						 output reg signed 	[`PWM_VALUE_BIT_WIDTH-1:0] 	roll_val_buff,
						 output reg signed 	[`PWM_VALUE_BIT_WIDTH-1:0] 	pitch_val_buff,
						 output reg 	   	[`PWM_VALUE_BIT_WIDTH-1:0]	throttle_val_buff,
						 // DEBUG Wire
						 output wire 	   	[`DEBUG_WIRE_BIT_WIDTH-1:0]	DEBUG_WIRE,
						 // Input
						 input wire signed 	[`PWM_VALUE_BIT_WIDTH-1:0] 	yaw_rec_val,
						 input wire signed 	[`PWM_VALUE_BIT_WIDTH-1:0] 	roll_rec_val,
						 input wire signed 	[`PWM_VALUE_BIT_WIDTH-1:0] 	pitch_rec_val,
						 input wire		   	[`PWM_VALUE_BIT_WIDTH-1:0] 	throttle_rec_val,
						 input wire		  	[`PWM_VALUE_BIT_WIDTH-1:0] 	curr_motor_rate,
						 input wire 	  	[`REC_DATA_SEL_BIT_WIDTH-1:0]rec_data_sel,
						 input wire 	   	us_clk);
	
	// Parameter for timer counters to increase throttle during takeoff
	localparam TIMER_BIT_WIDTH = 7'd20;
	localparam signed [TIMER_BIT_WIDTH-1:0]
		TIMER_ZERO 		 = 0,		// When Timer is Zero match size to ensure correct
		TAKE_OFF_TIMER_1 = 200000,	// 0   -> 115 in 1    second
		TAKE_OFF_TIMER_2 = 500;		// 115 -> 165 in 0.25 seconds
	reg signed [TIMER_BIT_WIDTH-1:0] timer_throttle_ramp, hover_timer;
	
	// Parameters for 3 stage throttle ramp during auto-takeoff
	localparam [`PWM_VALUE_BIT_WIDTH-1:0]
		TAKEOFF_RAMP_THROTTLE_VAL_1 = 115, // slow motor rise value
		TAKEOFF_RAMP_THROTTLE_VAL_2 = 165, // hover off ground value
		TAKE_OFF_THROTTLE 			= 175; // value to do slow rise till flight_mode changes to hover
	reg	start_takeoff;
	
	// Parameter for timer so that if current throttle has 0.5s to reach hover throttle rate
	localparam REACH_HOVER_TIMERBIT_WIDTH = 7'd20;
	localparam [REACH_HOVER_TIMERBIT_WIDTH-1:0]
		REACH_HOVER_TIMER	= 5000;
		
	// Auto-Landing timer and flag variables
	localparam AUTO_LAND_TIMER_BIT_WIDTH = 7'd20;
	localparam [AUTO_LAND_TIMER_BIT_WIDTH-1:0]
		AUTO_LAND_TIMER		= 500000;
	reg	signed [AUTO_LAND_TIMER_BIT_WIDTH-1:0] landing_timer;
	reg	start_autoland;
	
	assign DEBUG_WIRE = rec_data_sel;
	
	always @(posedge us_clk) begin
		case(rec_data_sel)
			`REC_SEL_OFF: begin
				throttle_val_buff		<= `BYTE_ALL_ZERO;
				yaw_val_buff			<= `BYTE_ALL_ZERO;
				roll_val_buff			<= `BYTE_ALL_ZERO;
				pitch_val_buff			<= `BYTE_ALL_ZERO;
			end
			`REC_SEL_AUTO_TAKE_OFF:	begin
				if (!start_takeoff) begin // if this is the first run through ge the current motor_rate
					start_takeoff		<= `TRUE;
					throttle_val_buff 	<= curr_motor_rate;
				end
				else begin// continue to increase latched throttle value
					// stage 1 throttle 0 -> 115 aka 0 -> 45%
					if (throttle_val_buff < TAKEOFF_RAMP_THROTTLE_VAL_1) begin
						if (timer_throttle_ramp > TIMER_ZERO)
							timer_throttle_ramp <= timer_throttle_ramp - `ONE;
						else begin
							throttle_val_buff	<= throttle_val_buff + `ONE;
							timer_throttle_ramp <= TAKE_OFF_TIMER_1;
						end	
					end
					// stage 2 throttle 115 -> 165 aka 45 -> 65%
					else if (throttle_val_buff < TAKEOFF_RAMP_THROTTLE_VAL_2) begin
						if (timer_throttle_ramp > TIMER_ZERO)
							timer_throttle_ramp <= timer_throttle_ramp - `ONE;
						else begin
							throttle_val_buff	<= throttle_val_buff + `ONE;
							timer_throttle_ramp <= TAKE_OFF_TIMER_2;
						end
					end
					// stage 3 throttle  slightly over hover to rise for flight_mode controlled timer amount
					else throttle_val_buff	<= TAKE_OFF_THROTTLE;
				end
				
				// all other target values held to zero to instill stable flight
				yaw_val_buff			<= `BYTE_ALL_ZERO;
				roll_val_buff			<= `BYTE_ALL_ZERO;
				pitch_val_buff			<= `BYTE_ALL_ZERO;
			end
			`REC_SEL_HOVER: begin
				// if motor_rates currently greater than 10 + HOVER slowly decrease to HOVER
				if (curr_motor_rate > (`HOVER_THROTTLE_VAL + `THROTTLE_VARIANCE)) begin
					if(hover_timer > TIMER_ZERO)
						hover_timer			<= hover_timer - `ONE;
					else begin
						hover_timer			<= REACH_HOVER_TIMER;
						throttle_val_buff 	<= throttle_val_buff - `ONE;
					end
				end
				// if motor_rate currently less than 10 + HOVER slowly increase to HOVER
				else if(curr_motor_rate < (`HOVER_THROTTLE_VAL - `THROTTLE_VARIANCE)) begin
					if(hover_timer > TIMER_ZERO)
						hover_timer			<= hover_timer - `ONE;
					else begin
						hover_timer			<= REACH_HOVER_TIMER;
						throttle_val_buff 	<= throttle_val_buff + `ONE;
					end
				end
				// Otherwise stay at HOVER throttle rate
				else throttle_val_buff	<= `HOVER_THROTTLE_VAL;
				
				// all other target values held to zero to instill stable flight
				yaw_val_buff			<= `BYTE_ALL_ZERO;
				roll_val_buff			<= `BYTE_ALL_ZERO;
				pitch_val_buff			<= `BYTE_ALL_ZERO;
			end
			`REC_SEL_PASS_THROUGH: begin
				yaw_val_buff			<= yaw_rec_val;
				roll_val_buff			<= roll_rec_val;
				pitch_val_buff			<= pitch_rec_val;
				throttle_val_buff		<= throttle_rec_val;
			end
			`REC_SEL_AUTO_LAND: begin
				//TODO: Get auto-land working but need height sensor or z-axis velocity sensor input
				if (curr_motor_rate > `MOTOR_VAL_MIN) begin
					if (!start_autoland) begin // if this is the first run through ge the current motor_rate
						start_autoland		<= `TRUE;
						throttle_val_buff 	<= curr_motor_rate;
					end
					else begin
						if(landing_timer > TIMER_ZERO)
							landing_timer		<= landing_timer - `ONE;
						else begin
							landing_timer		<= AUTO_LAND_TIMER;
							throttle_val_buff 	<= throttle_val_buff - `ONE;
						end
					end
				end
				else // motor_rate already at min val or zero 
					throttle_val_buff	<= `BYTE_ALL_ZERO;
					
				// All axis should target zero to stay level while in auto-land mode
				yaw_val_buff			<= `BYTE_ALL_ZERO;
				roll_val_buff			<= `BYTE_ALL_ZERO;
				pitch_val_buff			<= `BYTE_ALL_ZERO;
			end
			default: begin
				// should never reach here! So treat like restart
				yaw_val_buff			<= `BYTE_ALL_ZERO;
				roll_val_buff			<= `BYTE_ALL_ZERO;
				pitch_val_buff			<= `BYTE_ALL_ZERO;
				throttle_val_buff		<= `BYTE_ALL_ZERO;
			end
		endcase
	end
endmodule