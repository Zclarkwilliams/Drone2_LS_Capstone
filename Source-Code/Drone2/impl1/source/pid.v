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
 			  output reg pid_state_complete,
			  output reg pid_active,
			  input wire signed [RATE_BIT_WIDTH-1:0] target_rotation,
 			  input wire signed [IMU_VAL_BIT_WIDTH-1:0] actual_rotation,
			  input wire signed [RATE_BIT_WIDTH-1:0] angle_error,
			  input start_flag,
			  input wait_flag,
			  input resetn,
			  input us_clk);

	// working registers
	reg signed [RATE_BIT_WIDTH-1:0] 
		scaled_rotation, rotation_error, prev_rotation_error,
		rotation_proportional, rotation_integral, rotation_derivative,
		error_change, rotation_total;

	// min and max rate_out values
	localparam signed
		RATE_MIN = 16'hFFF0,
		RATE_MAX = 16'h0010;

	// proportionality constants
	localparam signed
		K_p = 16'h0001,
		K_i = 16'h0001, 
		K_d = 16'h0001;

	// state names
	localparam
		STATE_WAIT     = 6'b000001,
		STATE_CALC1    = 6'b000010,
		STATE_CALC2    = 6'b000100,
		STATE_CALC3    = 6'b001000,
		STATE_CALC4    = 6'b010000,
		STATE_COMPLETE = 6'b100000;
	
	// state variables
	reg [5:0] state, next_state;
	
	
	// update state
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn)
			state <= STATE_WAIT;
		else
			state <= next_state;
	end
	
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn) begin
			rate_out <= 16'h0000;
		end
		else begin
			case(state)
				STATE_WAIT: begin
					rate_out <= rate_out;
				end
				STATE_CALC1: begin
					prev_rotation_error <= rotation_error;
					rotation_error <= (target_rotation - actual_rotation);
					rotation_integral <= (K_i * angle_error);
				end
				STATE_CALC2: begin
					rotation_proportional <= (K_p * rotation_error);
					error_change <= (prev_rotation_error - rotation_error);
				end
				STATE_CALC3: begin
					rotation_derivative <= (K_d * error_change);
				end
				STATE_CALC4: begin
					rotation_total <= (rotation_proportional + rotation_integral + rotation_derivative);
				end
				STATE_COMPLETE: begin
					if(rotation_total < RATE_MIN)
						rate_out <= RATE_MIN;
					else if(rotation_total > RATE_MAX)
						rate_out <= RATE_MAX;
					else
						rate_out <= rotation_total;
				end
			endcase
		end
	end
	
	// next state logic
	always @(*) begin
		case(state)
			STATE_WAIT: begin
				if(!resetn)
					next_state = STATE_WAIT;
				else if(start_flag)
					next_state = STATE_CALC1;
				else
					next_state = STATE_WAIT;
			end
			STATE_CALC1: begin
				if(!resetn)
					next_state = STATE_WAIT;
				else
					next_state = STATE_CALC2;
			end
			STATE_CALC2: begin
				if(!resetn)
					next_state = STATE_WAIT;								
      else
					next_state = STATE_CALC3;
			end
			STATE_CALC3: begin
				if(!resetn)
					next_state = STATE_WAIT;
				else
					next_state = STATE_COMPLETE;
			end
			STATE_COMPLETE: begin
				if(wait_flag)
					next_state = STATE_WAIT;				
        else
					next_state = STATE_COMPLETE;
			end
		endcase
	end

	// calculation logic
	always @(state) begin
		case(state)
			STATE_WAIT: begin
				pid_active = 1'b0;
				pid_state_complete = 1'b0;
			end
			STATE_CALC1: begin
				pid_active = 1'b1;
				pid_state_complete = 1'b0;
				
			end
			STATE_CALC2: begin
				pid_active = 1'b1;
				pid_state_complete = 1'b0;
					
			end
			STATE_CALC3: begin
				pid_active = 1'b0;
				pid_state_complete = 1'b0;
					
			end
			STATE_COMPLETE: begin
				pid_active = 1'b0;
				pid_state_complete = 1'b0;
					
			end
		endcase
	end
	
endmodule

