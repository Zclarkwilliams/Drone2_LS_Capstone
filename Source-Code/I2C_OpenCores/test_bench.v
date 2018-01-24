
//`include "timescale.v"
`timescale 1ns / 1ns


module i2c_module_tb();
	wire scl ;
	wire sda ;
	reg rstn;
	wire [7:0] DATA_OUT;
	reg [7:0] DATA_IN;
	reg clk;

	i2c_module i2c(
		.scl(scl), 
		.sda(sda), 
		.rstn(rstn), 
		.DATA_OUT(DATA_OUT),
		.DATA_IN(DATA_IN),
		.clk(clk)
		);
		
	initial begin
		clk = 0;
		forever #10 clk = ~clk;
		end
	
	initial begin
		rstn = 1;
		#10 rstn = 0;
		#10 rstn = 1;
		#100000000;
		$stop;
		end
endmodule