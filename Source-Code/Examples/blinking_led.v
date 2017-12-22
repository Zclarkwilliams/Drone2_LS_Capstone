`timescale 1 ns / 1 ns
//===================================================================================================//
//
// blinking_led.v (top module)
// ---------------------------
// The module will responsible for generating 8 signals to drive 8 LEDs.
// The signals will be controled by a value of an internal 8-bit register.
// The value will be increased by 1 from 0000_0000 --> 1111_1111 and wrap around
//
//===================================================================================================//

module blinking_led (
	input        rst_btn,
	output       o_clk_1Hz,			// <-- testbench purpose
	output [7:0] LED
);

//==========================================================================//
// Declare variables, registers and wires
//==========================================================================//

	// Declare constant variable
	localparam 	counter = 20460000;

	// Internal registers
	reg [31:0] 	clk_cnt  = 32'b0;			// Counter for clock divider
	reg 		clk_1Hz  = 1'b0;			// Carry 1 MHz clock speed
	reg [7:0]	int_LED  = 8'b0;			// Store value for the outputs
	reg [2:0]	position = 3'b0;
	// Internal wires
	wire		sys_clk;					// Carry 2.08 MHz clock speed

//==========================================================================//
// Generate clock signals
//==========================================================================//

	// Generate 2.08 MHz clock signal
	OSCH OSCH_inst (
		.STDBY	   ( 1'b0	  ),
		.OSC	   ( sys_clk  ),
		.SEDSTDBY  (		  )
	);
	defparam OSCH_inst.NOM_FREQ = "20.46";

	// Generate 1 Hz clock signal
	always @ (posedge sys_clk or posedge rst_btn) begin
		if (rst_btn) begin
			clk_cnt <= 32'b0;
			clk_1Hz <= 1'b0;
		end
		else begin
			clk_cnt <= (clk_cnt < (counter-1)) 	   ? (clk_cnt + 1'b1) : 32'b0;
			clk_1Hz <= (clk_cnt > ((counter/2)-1)) ? 1'b1 : 1'b0;
		end
	end
	
//==========================================================================//
// Generate data for outputs
//==========================================================================//

	always @ (posedge clk_1Hz or posedge rst_btn) begin
		if (rst_btn) begin
			position <= 3'b0;
		end
		else begin
			position <= position + 1'b1;
		end
	end
	
	always @ (position) begin
		case (position)
			3'b001: begin
				int_LED = 8'b0000_0001;
			end
			3'b010: begin
				int_LED = 8'b0000_0010;
			end
			3'b011: begin
				int_LED = 8'b0000_0100;
			end
			3'b100: begin
				int_LED = 8'b0000_1000;
			end
			3'b101: begin
				int_LED = 8'b0001_0000;
			end
			3'b110: begin
				int_LED = 8'b0010_0000;
			end
			3'b111: begin
				int_LED = 8'b0100_0000;
			end
			3'b000: begin
				int_LED = 8'b1000_0000;
			end
			default: begin
				int_LED = 8'b0000_0001;
			end
		endcase
	end

//==========================================================================//
// assign data for outputs
//==========================================================================//

	assign LED = ~int_LED;
	assign o_clk_1Hz = clk_1Hz;		// <-- testbench purpose

endmodule
