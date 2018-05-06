/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

/**
 * module pid - sub-module implementing a pid controller for  rotation rate
 * around a single axis.
 *
 * Outputs:
 *
 */
 module pid #(parameter RATE_BIT_WIDTH = 16,
 			  parameter PID_RATE_BIT_WIDTH = 16,
			  parameter IMU_VAL_BIT_WIDTH = 16)
 			 (output reg [PID_RATE_BIT_WIDTH-1:0] rate_out,
 			  output reg pid_complete,
			  output reg pid_active,
			  input [RATE_BIT_WIDTH-1:0] target_rotation,
 			  input [IMU_VAL_BIT_WIDTH-1:0] actual_rotation,
			  input [RATE_BIT_WIDTH-1:0] angle_error,
			  input start_flag,
			  input wait_flag,
			  input resetn,
			  input us_clk);


	// working registers
	reg [RATE_BIT_WIDTH-1:0] scaled_rotation;

	// state names
	localparam
		WAIT     = 6'b000001,
		CALC1    = 6'b000010,
		CALC2    = 6'b000100,
		CALC3    = 6'b001000,
		CALC4    = 6'b010000,
		COMPLETE = 6'b100000;

	// state variables
	reg [5:0] state, next_state;

	// update state
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn)
			state <= WAIT;
		else
			state <= next_state;
	end

	// next state logic
	always @(*) begin
		case(state)
			WAIT: begin
				if(!resetn)
					next_state = WAIT;
				else if(start_flag)
					next_state = CALC1;
				else
					next_state = WAIT;
			end
			CALC1: begin
				if(!resetn)
					next_state = WAIT;
				else
					next_state = CALC2;
			end
			CALC2: begin
				if(!resetn)
					next_state = WAIT;
				else
					next_state = CALC3;
			end
			CALC3: begin
				if(!resetn)
					next_state = WAIT;
				else
					next_state = COMPLETE;
			end
			COMPLETE: begin
				if(wait_flag)
					next_state = WAIT;
				else
					next_state = COMPLETE;
			end
		endcase
	end

	// calculation logic
	always @(state) begin
		case(state)
			WAIT: begin
				pid_active = 1'b0;
				pid_complete = 1'b0;
			end
			CALC1: begin
				pid_active = 1'b1;
				pid_complete = 1'b0;

			end
			CALC2: begin
				pid_active = 1'b1;
				pid_complete = 1'b0;

			end
			CALC3: begin
				pid_active = 1'b0;
				pid_complete = 1'b0;

			end
			COMPLETE: begin
				pid_active = 1'b0;
				pid_complete = 1'b0;

			end
		endcase
	end

endmodule

