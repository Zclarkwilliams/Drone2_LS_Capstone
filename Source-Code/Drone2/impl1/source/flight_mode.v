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
 * Flight mode selector
 */
`timescale 1ns / 1ns

`include "common_defines.v"

module flight_mode (
	output reg  [2:0] switch_a,
	output reg  [1:0] switch_b,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] swa_swb_val,
	input  wire resetn,
	input  wire us_clk
	);
	
	function automatic in_range;
		input reg [`REC_VAL_BIT_WIDTH-1:0]value;
		input reg [`REC_VAL_BIT_WIDTH-1:0]lower_lim;
		input reg [`REC_VAL_BIT_WIDTH-1:0]upper_lim;
		if ( (value >= lower_lim) && (lower_lim <= upper_lim) )
			in_range = 1'b1;
		else
			in_range = 1'b0;
	endfunction
	
	always@(posedge us_clk, negedge resetn) begin
		if(~resetn) begin
			switch_a <= 3'b000; switch_b <= 2'b00; 
		end
			else begin
			case(swa_swb_val)
				in_range(swa_swb_val, 8'd0,   8'd49 ): begin switch_a <= 3'b100; switch_b <= 2'b01; end
				in_range(swa_swb_val, 8'd50,  8'd99 ): begin switch_a <= 3'b100; switch_b <= 2'b10; end
				in_range(swa_swb_val, 8'd100, 8'd149): begin switch_a <= 3'b001; switch_b <= 2'b11; end //When SWA is in mode 0, SWB position doesn't make a difference
				in_range(swa_swb_val, 8'd150, 8'd199): begin switch_a <= 3'b010; switch_b <= 2'b10; end
				in_range(swa_swb_val, 8'd200, 8'd250): begin switch_a <= 3'b010; switch_b <= 2'b01; end
				//default :                              begin switch_a <= 3'b000; switch_b <= 2'b00; end
			endcase
		end
	end
	
endmodule
