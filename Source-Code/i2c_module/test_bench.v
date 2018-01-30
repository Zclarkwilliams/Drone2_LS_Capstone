`timescale 1 ps / 1 ps
module i2c_module_tb();
	wire scl	;
	wire sda ;
	reg rstn;
	reg purn;
	wire [7:0] dataRX;
	reg [7:0] dataTX;
	reg [7:0]moduleRegIn = 0;
    wire clk;
	wire done;
	reg go;
	reg readWriteIn = 1;


	GSR GSR_INST (.GSR (rstn));
	PUR PUR_INST (.PUR (purn));


	i2c_module i2c(
		.scl(scl), 
		.sda(sda), 
		.rstn( (rstn) ),
		.moduleDataOut(dataRX),
		.moduleDataIn(dataTX),
		.moduleRegIn(moduleRegIn),
		.readWriteIn(readWriteIn),
		.go(go),
		.done(done),
		.clk(clk)
		); /* synthesis syn_hier=hard */;

	assign ( pull1, strong0 ) scl = 1'b1;
	assign ( pull1, strong0 ) sda = 1'b1;
	initial begin
		rstn = 1;
		#10 rstn = 0;
		#10 rstn = 1;
		#100000000;
		$stop;
		end
endmodule