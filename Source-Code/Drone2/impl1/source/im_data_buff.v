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
 * imu_data_buffer - This buffer controls the bno55_driver module output bus. The
 * 					 control signal is from the flight_mode module and cycles according
 *					 to swa, and swb from user remote control, internal states, and internal resets.
 *
 * Outputs:
 * @ :
 *
 * Inputs:
 * @ :
 */
 
`timescale 1ns / 1ns
`include "common_defines.v"

module imu_data_buffer (// Outputs
						output reg [`IMU_VAL_BIT_WIDTH-1:0] rate_x_buff,
						output reg [`IMU_VAL_BIT_WIDTH-1:0] rate_y_buff,
						output reg [`IMU_VAL_BIT_WIDTH-1:0] rate_z_buff,
						output reg [`IMU_VAL_BIT_WIDTH-1:0] angle_x_buff,
						output reg [`IMU_VAL_BIT_WIDTH-1:0] angle_y_buff,
						output reg [`IMU_VAL_BIT_WIDTH-1:0] angle_z_buff, 
						output reg [`IMU_VAL_BIT_WIDTH-1:0] lin_accel_x_buff,
						output reg [`IMU_VAL_BIT_WIDTH-1:0] lin_accel_y_buff,
						output reg [`IMU_VAL_BIT_WIDTH-1:0] lin_accel_z_buff, 
						output reg [`IMU_VAL_BIT_WIDTH-1:0] x_vel_buff,
						output reg [`IMU_VAL_BIT_WIDTH-1:0] y_vel_buff,
						output reg [`IMU_VAL_BIT_WIDTH-1:0] z_vel_buff,
						// Inputs
						input wire [`IMU_VAL_BIT_WIDTH-1:0] rate_x,
						input wire [`IMU_VAL_BIT_WIDTH-1:0] rate_y,
						input wire [`IMU_VAL_BIT_WIDTH-1:0] rate_z,
						input wire [`IMU_VAL_BIT_WIDTH-1:0] angle_x,
						input wire [`IMU_VAL_BIT_WIDTH-1:0] angle_y,
						input wire [`IMU_VAL_BIT_WIDTH-1:0] angle_z,
						input wire [`IMU_VAL_BIT_WIDTH-1:0] lin_accel_x,
						input wire [`IMU_VAL_BIT_WIDTH-1:0] lin_accel_y, 
						input wire [`IMU_VAL_BIT_WIDTH-1:0] lin_accel_z, 
						input wire [`IMU_VAL_BIT_WIDTH-1:0] x_vel,
						input wire [`IMU_VAL_BIT_WIDTH-1:0] y_vel,
						input wire [`IMU_VAL_BIT_WIDTH-1:0] z_vel,
						input wire [`IMU_DATA_SEL_BIT_WIDTH-1:0] imu_data_sel,
						input wire us_clk);
		
	always @ (posedge us_clk) begin
		case (imu_data_sel)
			`IMU_SEL_DISABLE:		begin
				rate_x_buff		<= `ALL_ZERO_2BYTE;
				rate_y_buff		<= `ALL_ZERO_2BYTE;
				rate_z_buff		<= `ALL_ZERO_2BYTE;
				angle_x_buff	<= `ALL_ZERO_2BYTE;
				angle_y_buff	<= `ALL_ZERO_2BYTE;
				angle_z_buff	<= `ALL_ZERO_2BYTE; 
			end
			`IMU_SEL_PASS_THROUGH:	begin
				x_vel_buff			<= x_vel;
				y_vel_buff			<= y_vel;
				z_vel_buff			<= z_vel;
				rate_x_buff			<= rate_x;
				rate_y_buff			<= rate_y;
				rate_z_buff			<= rate_z;
				angle_x_buff		<= angle_x;
				angle_y_buff		<= angle_y;
				angle_z_buff		<= angle_z;
				lin_accel_x_buff 	<=lin_accel_x;
				lin_accel_y_buff 	<=lin_accel_y;
				lin_accel_z_buff 	<=lin_accel_z;
			end
			default: begin 
				//should never reach this state, treat as reset.
				rate_x_buff		<= `ALL_ZERO_2BYTE;
				rate_y_buff		<= `ALL_ZERO_2BYTE;
				rate_z_buff		<= `ALL_ZERO_2BYTE;
				angle_x_buff	<= `ALL_ZERO_2BYTE;
				angle_y_buff	<= `ALL_ZERO_2BYTE;
				angle_z_buff	<= `ALL_ZERO_2BYTE; 
			end
		endcase
	end
endmodule
