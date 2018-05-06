/**
 * module body_frame_controller- top level module for rotation rate control.  
 * Takes in target rotation rates and actual rotation rates and outputs control
 * values to motor mixer.
 *
 * Parameters
 * RATE_BIT_WIDTH - Size of values from angle controller
 * IMU_VAL_BIT_WIDTH - Size of values from IMU
 *
 * Outputs * yaw_rate_out - yaw rate to motor mixer
 * roll_rate_out - roll rate to motor mixer
 * pitch_rate_out - pitch rate to motor mixer
 *
 * Inputs
 * * (target values are 2's complement, fixed-point, 12.4 bits)
 * yaw_rate_in - target yaw rotation rate from angle controller (deg/s) 
 * roll_rate_in - target roll rate from angle controller (deg/s)
 * pitch_rate_in - target pitch rate from angle controller (deg/s)
 *
 * (actual values are 2's complement, in 1/100ths of a degree)
 * roll_rotation - actual roll rate from IMU (deg/s,)
 * pitch_rotation - actual pitch rate from IMU (deg/s)
 * yaw_rotation - actual yaw rate from IMU (deg/s)
 * 
 * start_flag - signal from angle controller to  begin cycle
 * resetn - global reset signal
 * us_clk - 1MHz clock
 */
 module body_frame_controller #(
	parameter RATE_BIT_WIDTH = 16,
	parameter IMU_VAL_BIT_WIDTH = 16,
	parameter PID_RATE_BIT_WIDTH = 16)
 	(output wire [PID_RATE_BIT_WIDTH-1:0] yaw_rate_out,
	output wire [PID_RATE_BIT_WIDTH-1:0] roll_rate_out,
 	output wire [PID_RATE_BIT_WIDTH-1:0] pitch_rate_out,
	output reg complete_signal,
 	input [RATE_BIT_WIDTH-1:0] yaw_target,
 	input [RATE_BIT_WIDTH-1:0] roll_target,
 	input [RATE_BIT_WIDTH-1:0] pitch_target,
 	input [IMU_VAL_BIT_WIDTH-1:0] roll_rotation,
 	input [IMU_VAL_BIT_WIDTH-1:0] pitch_rotation,
 	input [IMU_VAL_BIT_WIDTH-1:0] yaw_rotation,
	input [RATE_BIT_WIDTH-1:0] roll_angle_error,
	input [RATE_BIT_WIDTH-1:0] pitch_angle_error,
 	input start_signal,
	input resetn,
	input us_clk);

	// working registers
	reg wait_flag, start_flag;
	
	

	// state names
	localparam
		WAITING  = 4'b0001,
		STARTING = 4'b0010,
		ACTIVE   = 4'b0100,
		COMPLETE = 4'b1000;

	// state variables
	reg [3:0] state, next_state;

	// update state
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn)
			state <= WAITING;
		else
			state <= next_state;
	end

	// next state logic
	always @(*) begin
		case (state)
			WAITING: begin
				if(!resetn)
					next_state = WAITING;
				else if(start_flag)
					next_state = STARTING;
				else
					next_state = WAITING;
			end
			STARTING: begin
				if(!resetn)
					next_state = WAITING;
				else if((yaw_active) && (pitch_active) && (roll_active))
					next_state = ACTIVE;
				else
					next_state = STARTING;
			end
			ACTIVE: begin
				if(!resetn)
					next_state = WAITING;
				else if((yaw_complete) && (pitch_complete) && (roll_complete))
					next_state = COMPLETE;
				else
					next_state = ACTIVE;
			end
			COMPLETE: begin
				next_state = WAITING;
			end
		endcase
	end
	
	// valid strobe logic
	assign pid_complete = (state == COMPLETE) ? 1'b1 : 1'b0;
	
	// sub-module control logic
	always @(state) begin
		case(state)
			WAITING: begin
				wait_flag = 1'b1;
				complete_signal = 1'b0;
			end
			STARTING: begin
				wait_flag = 1'b0;
				complete_signal = 1'b0;				
			end
			ACTIVE: begin
				wait_flag = 1'b0;
				complete_signal = 1'b0;
			end
			COMPLETE: begin
				wait_flag = 1'b0;
				complete_signal = 1'b1;
			end
		endcase
	end
	
	
	// pid instantiations

	pid #(
		.RATE_BIT_WIDTH(RATE_BIT_WIDTH),
		.PID_RATE_BIT_WIDTH(PID_RATE_BIT_WIDTH),
		.IMU_VAL_BIT_WIDTH(IMU_VAL_BIT_WIDTH)) 
	yaw_pid (
		.rate_out(yaw_rate_out),
		.pid_complete(yaw_complete),
		.pid_active(yaw_active),
		.target_rotation(yaw_target),
		.actual_rotation(yaw_rotation),
		.angle_error(16'h0000),
		.start_flag(start_flag),
		.wait_flag(wait_flag),
		.resetn(resetn),
		.us_clk(us_clk));
	
	pid #(
		.RATE_BIT_WIDTH(RATE_BIT_WIDTH),
		.PID_RATE_BIT_WIDTH(PID_RATE_BIT_WIDTH),
		.IMU_VAL_BIT_WIDTH(IMU_VAL_BIT_WIDTH)) 
	pitch_pid (
		.rate_out(pitch_rate_out),
		.pid_complete(pitch_complete),
		.pid_active(pitch_active),
		.target_rotation(pitch_target),
		.actual_rotation(pitch_rotation),
		.angle_error(pitch_angle_error),
		.start_flag(start_flag),
		.wait_flag(wait_flag),
		.resetn(resetn),
		.us_clk(us_clk));
	
	pid #(
		.RATE_BIT_WIDTH(RATE_BIT_WIDTH),
		.PID_RATE_BIT_WIDTH(PID_RATE_BIT_WIDTH),
		.IMU_VAL_BIT_WIDTH(IMU_VAL_BIT_WIDTH)) 
	roll_pid (
		.rate_out(roll_rate_out),
		.pid_complete(roll_complete),
		.pid_active(roll_active),
		.target_rotation(roll_target),
		.actual_rotation(roll_rotation),
		.angle_error(roll_angle_error),
		.start_flag(start_flag),
		.wait_flag(wait_flag),
		.resetn(resetn),
		.us_clk(us_clk));


endmodule
