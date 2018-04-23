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
		WAIT = 5'b00001,
		CALC1 = 5'b00010,
		CALC2 = 5'b00100,
		CALC3 = 5'b01000,
		CALC4 = 5'b10000;
	
	// state variables
	reg [4:0] state, next_state;
	
	// update state
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn)
			state <= WAIT;
		else
			state <= next_state;
	end
	
	// next state logic


endmodule
