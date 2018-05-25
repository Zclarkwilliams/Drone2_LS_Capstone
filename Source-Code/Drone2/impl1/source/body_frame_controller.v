`timescale 1ns / 1ns
`default_nettype none

/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

/**
 * module body_frame_controller- top level module for rotation rate control.
 * Takes in target rotation rates and actual rotation rates and outputs control
 * values to motor mixer.
 *
 * Outputs:
 * @yaw_rate_out - yaw rate to motor mixer
 * @roll_rate_out - roll rate to motor mixer
 * @pitch_rate_out - pitch rate to motor mixer
 *
 * Inputs:
 * (target values are 2's complement, fixed-point, 12.4 bits)
 * @yaw_rate_in - target yaw rotation rate from angle controller (deg/s)
 * @roll_rate_in - target roll rate from angle controller (deg/s)
 * @pitch_rate_in - target pitch rate from angle controller (deg/s)
 *
 * (actual values are 2's complement, in 1/100ths of a degree)
 * @roll_rotation - actual roll rate from IMU (deg/s)
 * @pitch_rotation - actual pitch rate from IMU (deg/s)
 * @yaw_rotation - actual yaw rate from IMU (deg/s)
 *
 * @start_flag - signal from angle controller to  begin cycle
 * @resetn - global reset signal
 * @us_clk - 1MHz clock
 */
 module body_frame_controller (
 	output wire [`PID_RATE_BIT_WIDTH-1:0] yaw_rate_out,
	output wire [`PID_RATE_BIT_WIDTH-1:0] roll_rate_out,
 	output wire [`PID_RATE_BIT_WIDTH-1:0] pitch_rate_out,
	output reg complete_signal,
	// Debug led output wire
	output wire [15:0] DEBUG_WIRE,
 	input wire [`RATE_BIT_WIDTH-1:0] yaw_target,
 	input wire [`RATE_BIT_WIDTH-1:0] roll_target,
 	input wire [`RATE_BIT_WIDTH-1:0] pitch_target,
 	input wire [`IMU_VAL_BIT_WIDTH-1:0] roll_rotation,
 	input wire [`IMU_VAL_BIT_WIDTH-1:0] pitch_rotation,
 	input wire [`IMU_VAL_BIT_WIDTH-1:0] yaw_rotation,
	input wire [`RATE_BIT_WIDTH-1:0] roll_angle_error,
	input wire [`RATE_BIT_WIDTH-1:0] pitch_angle_error,
 	input wire start_signal,
	input wire resetn,
	input wire us_clk);

  	// internal wires
	wire yaw_active, roll_active, pitch_active;
	wire yaw_complete, roll_complete, pitch_complete;

	// working registers
	reg wait_flag, start_flag;

	// state names
	localparam
		STATE_WAITING  = 4'b0001,
		STATE_STARTING = 4'b0010,
		STATE_ACTIVE   = 4'b0100,
		STATE_COMPLETE = 4'b1000;

	// state variables
	reg [3:0] state, next_state;

	// debug wires
	wire [15:0] DEBUG_WIRE_YAW, DEBUG_WIRE_ROLL, DEBUG_WIRE_PITCH;
	assign DEBUG_WIRE = (!resetn) ? 16'h0 : DEBUG_WIRE_PITCH;

	// latch start signal
	always @(posedge start_signal or posedge us_clk or negedge resetn) begin
		if(!resetn)
			start_flag <= 1'b0;
		else if(start_signal && !start_flag)
			start_flag <= 1'b1;
		else if(!start_signal && start_flag)
			start_flag <= 1'b0;
		else
			start_flag <= start_flag;
	end

	// update state
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn)
			state <= STATE_WAITING;
		else
			state <= next_state;
	end


	// next state logic
	always @(*) begin
    if(!resetn) begin
      next_state = STATE_WAITING;
    end
    else begin
  		case (state)
  			STATE_WAITING: begin
  				if(start_flag)
  					next_state = STATE_STARTING;
  				else
  					next_state = STATE_WAITING;
  			end
  			STATE_STARTING: begin
  				if((yaw_active) && (pitch_active) && (roll_active))
  					next_state = STATE_ACTIVE;
  				else
  					next_state = STATE_STARTING;
  			end
  			STATE_ACTIVE: begin
  				if((yaw_complete) && (pitch_complete) && (roll_complete))
  					next_state = STATE_COMPLETE;
  				else
  					next_state = STATE_ACTIVE;
  			end
  			STATE_COMPLETE: begin
  				next_state = STATE_WAITING;
  			end
        default: begin
          next_state = STATE_WAITING;
        end
  		endcase
    end
	end

	// sub-module control logic
	always @(state) begin
		case(state)
			STATE_WAITING: begin
				wait_flag = 1'b1;
				complete_signal = 1'b0;
			end
			STATE_STARTING: begin
				wait_flag = 1'b0;
				complete_signal = 1'b0;
			end
			STATE_ACTIVE: begin
				wait_flag = 1'b0;
				complete_signal = 1'b0;
			end
			STATE_COMPLETE: begin
				wait_flag = 1'b0;
				complete_signal = 1'b1;
			end
			default: begin
				wait_flag = 1'b1;
				complete_signal = 1'b0;
			end
		endcase
	end

	// pid instantiations
	pid yaw_pid (
		.DEBUG_WIRE(DEBUG_WIRE_YAW),
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

	pid pitch_pid (
		.DEBUG_WIRE(DEBUG_WIRE_PITCH),
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

	pid roll_pid (
		.DEBUG_WIRE(DEBUG_WIRE_ROLL),
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

