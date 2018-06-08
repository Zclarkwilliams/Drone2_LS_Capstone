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
 *  flight_mode -	Controls the flight operation from the switches on the remote control 
 *					specifically swa and swb. This module mainly controls the data bus
 *					buffers from the receiver to the angle_controller and the output data
 *					from the imu, bno55_driver, to all the modules it feeds. This module
 *					also controls the integration of all the reset lines, physical, 
 *					firmware, and software.
 *
 * 	Outputs:
 *	@ resetn:
 *	@ imu_data_sel:
 *	@ rec_data_sel:
 *	@ curr_avg_motor_rate:	The average motor rate for all motor_mixer motor_#_rate outputs	8 bit
 *	@ DEBUG_WIRE:
 *
 * 	Inputs:
 * 	@ us_clk:
 * 	@ swa_swb_val:
 * 	@ soft_reset_n:
 * 	@ curr_avg_motor_rate:
 * 	@ imu_calib_status:
 * 	@ machxo3_switch_reset_n:
 */
 
`timescale 1ns / 1ns
`include "common_defines.v"

module flight_mode	(//	Module Outputs
					 output wire resetn,
					 output reg	[`IMU_DATA_SEL_BIT_WIDTH-1:0]	imu_data_sel,
					 output reg	[`REC_DATA_SEL_BIT_WIDTH-1:0]	rec_data_sel,
					 output reg [`MOTOR_RATE_BIT_WIDTH-1:0]		curr_avg_motor_rate,
					 // DEBUG wire for on-board debug_leds
					 output wire [`DEBUG_WIRE_BIT_WIDTH-1:0]	DEBUG_WIRE,
					 //	Module Inputs
					 input wire [`REC_VAL_BIT_WIDTH-1:0] 	imu_calib_status,
					 input wire [`REC_VAL_BIT_WIDTH-1:0] 	swa_swb_val,
					 input wire [`REC_VAL_BIT_WIDTH-1:0] 	curr_throttle_val,
					 input wire [`MOTOR_RATE_BIT_WIDTH-1:0]	curr_motor_1_rate,
					 input wire [`MOTOR_RATE_BIT_WIDTH-1:0] curr_motor_2_rate,
					 input wire [`MOTOR_RATE_BIT_WIDTH-1:0] curr_motor_3_rate,
					 input wire [`MOTOR_RATE_BIT_WIDTH-1:0] curr_motor_4_rate,
					 input wire 		soft_reset_n,
					 input wire 		machxo3_switch_reset_n,
					 input wire 		us_clk);
	
	// When IMU finished calibration imu_calib_status value should reflect this
	localparam CALIB_DONE_VAL = 8'hFF;
	
	// SWA and SWB position combination values encoded
	localparam  SW_AB_BIT_WIDTH = 8;
	localparam [SW_AB_BIT_WIDTH - 1:0] 
		SW_AB_0X	= 8'h00,
		SW_AB_10	= 8'h01,
		SW_AB_20	= 8'h02,
		SW_AB_11	= 8'h11,
		SW_AB_21 	= 8'h12;
	
	// Flight Modes possible 
	localparam  MODE_BIT_WIDTH = 3'd4;
	localparam [MODE_BIT_WIDTH-1:0] 
		MODE_STASIS				= 0,
		MODE_USER_RESET			= 1,
		MODE_TAKE_OFF			= 2,
		MODE_USER_CNTRL_NO_IMU	= 3,
		MODE_USER_CNTRL_IMU		= 4,
		MODE_AUTO_LAND 			= 5,
		MODE_DUMMY 				= 6,
		MODE_HOVER 				= 7;
	
	//	SWA, SWB position decoding from receiver module
	localparam [`REC_VAL_BIT_WIDTH-1:0]
		SWA_SWB_0X_MIN	= 118,
		SWA_SWB_0X_MAX 	= 128,
		SWA_SWB_10_MIN 	= 220,
		SWA_SWB_10_MAX 	= 230,
		SWA_SWB_20_MIN 	=  18,
		SWA_SWB_20_MAX 	=  28,
		SWA_SWB_11_MIN 	= 168,
		SWA_SWB_11_MAX 	= 178,
		SWA_SWB_12_MIN 	=  68,
		SWA_SWB_12_MAX 	=  78;

	//	System, imu, and other subsystem initialization timer
	localparam TIMER_BIT_WIDTH = 5'd22;
	localparam signed [TIMER_BIT_WIDTH-1:0]
		INIT_TIMER_LENGTH	= 10000000,
		INIT_TIMER_ZERO 	= 0;
	
	// Variables of regs and wires
	wire		imu_rdy;									// Initialization of IMU complete verified by calib_status
	reg	[7:0]	err_flag;									// error flag to make sure if motors running land and stay landed						
	reg			sys_rdy;									// Initialization of system and timer complete
	reg			first_start;								// Flag to catch if first start after power off for sys_rdy timer
	reg			take_off_start;								// Flag to check if drone on ground for auto-take-off mode
	reg			reset_land;									// Flag to notify that we are on a reset forced landing, i.e. user cannot resume control till landed
	reg  		[MODE_BIT_WIDTH-1:0] 	mode;				// assigning modes of flight
	reg	 		[SW_AB_BIT_WIDTH-1:0] 	swa_swb_position;	// SWA & SWB position assingnment
	reg signed 	[TIMER_BIT_WIDTH-1:0]	init_timer;			// System initialization timer

// ----------------------------------	Asynchronous Control ---------------------------------	//

	initial sys_rdy 	= `FALSE;
	initial first_start = `FALSE;
	
	// Debug LEDs assignment to monitor module functionality 
	assign DEBUG_WIRE = (!sys_rdy) ? 8'hAA : curr_avg_motor_rate;//mode;
	
	// Set init flag so as to know if calibration and first run occured
	assign imu_rdy = ((imu_calib_status == CALIB_DONE_VAL)) ? `FALSE : `TRUE;
	
	// Reset signal control, it is asynchronous
	// TODO: test && for setting sys_rdy to resetn 
	assign resetn = machxo3_switch_reset_n & soft_reset_n ;//& sys_rdy;
	
	// Get the SWA & SWB positions
	//always @(posedge us_clk) begin
	always @(swa_swb_val or sys_rdy) begin
		if (sys_rdy) begin //only read switches once the initialization timer done! 
			if 		(swa_swb_val >= SWA_SWB_0X_MIN && swa_swb_val <= SWA_SWB_0X_MAX)
				swa_swb_position	<= SW_AB_0X;
			else if (swa_swb_val >= SWA_SWB_10_MIN && swa_swb_val <= SWA_SWB_10_MAX)
				swa_swb_position	<= SW_AB_10;
			else if (swa_swb_val >= SWA_SWB_20_MIN && swa_swb_val <= SWA_SWB_20_MAX)
				swa_swb_position	<= SW_AB_20;
			else if (swa_swb_val >= SWA_SWB_11_MIN && swa_swb_val <= SWA_SWB_11_MAX)
				swa_swb_position	<= SW_AB_11;
			else if (swa_swb_val >= SWA_SWB_12_MIN && swa_swb_val <= SWA_SWB_12_MAX)
				swa_swb_position	<= SW_AB_21;
			else // Should never reach this, all Switch positions combos captured above
				swa_swb_position	<= swa_swb_position; // unless read during switch transition
		end
		else 
			swa_swb_position 		<= SW_AB_0X; // STAY IN STASIS MODE!!
	end
	
// ----------------------------------	Synchronous Control	----------------------------------	//

	// Get the average motor rate
	always @(posedge us_clk) begin
		curr_avg_motor_rate <= get_average(curr_motor_1_rate, curr_motor_2_rate, curr_motor_3_rate, curr_motor_4_rate);
	end
	
	// system initialization timer to allow esc, imu and other subsystems to setup and initialize
	always @(posedge us_clk or negedge machxo3_switch_reset_n or negedge soft_reset_n) begin
		if (!machxo3_switch_reset_n || !soft_reset_n || !first_start) begin
			init_timer		<= INIT_TIMER_LENGTH;// Currently set to 2s 
			first_start		<= `TRUE;
			sys_rdy			<= `FALSE;
		end
		else if (!sys_rdy && init_timer > INIT_TIMER_ZERO)  begin
			init_timer		<= init_timer - `ONE;
			first_start		<= `TRUE;
			sys_rdy			<= `FALSE;
		end
		else if (sys_rdy) begin
			//do nothing becasue already initialized and all that.
			init_timer 		<= INIT_TIMER_ZERO;
			first_start		<= `TRUE;
			sys_rdy 		<= `TRUE;
		end
		else begin
			init_timer		<= INIT_TIMER_ZERO;	// Currently set to 1.5 ms 
			first_start		<= `TRUE;
			if(imu_rdy) // imu calibration done and ready so start system
				sys_rdy		<= `TRUE;
			else // imu calib not ready yet so wait
				sys_rdy 	<= `FALSE;
		end
	end
	
	// Flight mode control and prioritization 
	always @(posedge us_clk or negedge soft_reset_n or negedge machxo3_switch_reset_n) begin
		if (!sys_rdy || !soft_reset_n || !machxo3_switch_reset_n || (swa_swb_position == SW_AB_0X)) begin
			if (curr_avg_motor_rate > `MOTOR_VAL_MIN) begin
				reset_land		<=`TRUE;
				mode			<= MODE_AUTO_LAND;
			end
			else begin
				rec_data_sel	<= `REC_SEL_OFF;
				imu_data_sel	<= `IMU_SEL_PASS_THROUGH;
				mode			<= MODE_STASIS;
			end
		end
		else begin
			case (mode)
				MODE_STASIS: begin
					if(!err_flag || sys_rdy) begin 
						if ((swa_swb_position == SW_AB_10 || swa_swb_position == SW_AB_11) && (curr_throttle_val <= `MOTOR_VAL_MIN))
							mode				<= MODE_USER_CNTRL_IMU;
						else
							mode				<= MODE_STASIS;
					end
					else begin
						rec_data_sel			<= `REC_SEL_OFF;
						imu_data_sel			<= `IMU_SEL_DISABLE;
						mode					<= MODE_STASIS;
					end
				end
				MODE_USER_CNTRL_IMU: begin
					case(swa_swb_position)
						SW_AB_0X: begin
							mode				<= MODE_AUTO_LAND;
						end
						SW_AB_11: begin
							mode				<= MODE_USER_CNTRL_NO_IMU;
						end
						SW_AB_20: begin
							if(curr_avg_motor_rate <= `MOTOR_VAL_MIN)
								mode			<= MODE_TAKE_OFF;
							else
								mode			<= MODE_HOVER;
						end
						default: begin
							rec_data_sel		<= `REC_SEL_PASS_THROUGH;
							imu_data_sel		<= `IMU_SEL_PASS_THROUGH;
							mode				<= MODE_USER_CNTRL_IMU;	
						end
					endcase
				end
				MODE_USER_CNTRL_NO_IMU: begin
					case(swa_swb_position)
						SW_AB_0X: begin
							mode				<= MODE_AUTO_LAND;
						end
						SW_AB_10: begin
							mode				<= MODE_USER_CNTRL_IMU;
						end
						SW_AB_21: begin
							if (curr_avg_motor_rate <= `MOTOR_VAL_MIN + `THROTTLE_VARIANCE)
								mode			<= MODE_TAKE_OFF;
							else 
								mode			<= MODE_HOVER;
						end
						default: begin
							rec_data_sel		<= `REC_SEL_PASS_THROUGH;
							imu_data_sel		<= `IMU_SEL_DISABLE;
							mode				<= MODE_USER_CNTRL_NO_IMU;	
						end
					endcase
				end
				MODE_TAKE_OFF: begin	//	Take off from motor off to hovering
					case (swa_swb_position)
						SW_AB_0X: begin	// User chose to go to land or stasis instead of continue to take-off
							mode				<= MODE_AUTO_LAND;
						end
						SW_AB_10: begin
							take_off_start		<= `FALSE;
							if (curr_throttle_val >= curr_avg_motor_rate) begin	// TODO: Change this to an average motor value instead of curr_motor_val
								mode			<= MODE_USER_CNTRL_NO_IMU;
							end
							else begin
								if (curr_avg_motor_rate < `HOVER_THROTTLE_VAL)
									mode		<= MODE_TAKE_OFF; // wait till user increases throttle to stay in flight
								else
									mode		<= MODE_HOVER; 	  // reached hover throttle value got to hover mode
							end
						end
						SW_AB_11: begin
							take_off_start		<= `FALSE;
							if (curr_throttle_val >= curr_avg_motor_rate)
								mode			<= MODE_USER_CNTRL_IMU;
							else begin
								if (curr_avg_motor_rate < `HOVER_THROTTLE_VAL)
									mode		<= MODE_TAKE_OFF; // wait till user increases throttle to stay in flight
								else
									mode		<= MODE_HOVER; 	  // reached hover throttle value got to hover mode
							end
						end
						default: begin
							imu_data_sel		<= `IMU_SEL_PASS_THROUGH; // IMU needs to be enabled for this mode
							if (curr_avg_motor_rate >= `HOVER_THROTTLE_VAL) begin
								take_off_start	<= `FALSE;
								mode			<= MODE_HOVER;
							end
							else begin
								take_off_start	<= `TRUE;
								rec_data_sel	<= `REC_SEL_AUTO_TAKE_OFF;
								mode			<= MODE_TAKE_OFF;
							end
						end
					endcase
				end
				MODE_HOVER: begin
					case (swa_swb_position)
						SW_AB_0X: begin
							mode				<= MODE_AUTO_LAND;
						end
						SW_AB_10: begin
							if (curr_throttle_val >= `HOVER_THROTTLE_VAL)
								mode			<= MODE_USER_CNTRL_IMU;
							else
								mode			<= MODE_HOVER; // wait till user increases throttle to stay in flight
						end
						SW_AB_11: begin
							if (curr_throttle_val >= `HOVER_THROTTLE_VAL)
								mode			<= MODE_USER_CNTRL_NO_IMU;
							else
								mode			<= MODE_HOVER; // wait till user increases throttle to stay in flight
						end
						default: begin // Keep hovering until otherwise instructed
							rec_data_sel		<= `REC_SEL_HOVER;
							imu_data_sel		<= `IMU_SEL_PASS_THROUGH;
							mode				<= MODE_HOVER;
						end
					endcase
				end
				MODE_AUTO_LAND: begin
					if (curr_avg_motor_rate <= `MOTOR_VAL_MIN) begin
						reset_land 				<=`FALSE;
						mode					<= MODE_STASIS;
					end
					else if (!reset_land && (swa_swb_position == SW_AB_10) && (curr_throttle_val >= curr_avg_motor_rate))
						mode					<= MODE_USER_CNTRL_IMU;
					else if (!reset_land && (swa_swb_position == SW_AB_11) && (curr_throttle_val >= curr_avg_motor_rate))
						mode					<= MODE_USER_CNTRL_NO_IMU;
					else begin
						rec_data_sel			<= `REC_SEL_AUTO_LAND;
						imu_data_sel			<= `IMU_SEL_PASS_THROUGH;
						mode					<= MODE_AUTO_LAND;
					end
				end
				default: begin // should never reach here treat as a reset
					err_flag					<= `TRUE;
					if(curr_avg_motor_rate > `MOTOR_VAL_MIN) begin
						reset_land				<= `TRUE;
						mode					<= MODE_AUTO_LAND;
					end
					else
						mode					<= MODE_STASIS;
				end
			endcase
		end
	end

// --------------------------- Average Motor Rate Calculator Function	----------------------	//

function [7:0] get_average;
		// Inputs
		input wire [7:0] val_1;
		input wire [7:0] val_2;
		input wire [7:0] val_3;
		input wire [7:0] val_4;
		// Output -> assigned in module above to be 8 bit average of all 4 input values, (4 motors rates)
		reg [10:0] sum;
		begin
			// sum 4 entered values
			sum = (val_1 + val_2 + val_3 + val_4);
			get_average =  (sum >> 2);
			// divide by 4 and assign as the output
		end	
	endfunction

endmodule
