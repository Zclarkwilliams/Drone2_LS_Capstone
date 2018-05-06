`timescale 1 ns / 100 ps
module bno055_module_tb();
	wire scl1;
	wire sda1;
	wire scl2;
	wire sda2;
	reg rstn;
	reg purn;
	wire rstn_imu;
	wire [7:0] data_rx;
    wire clk;
	wire done;
	reg go;
	reg read_write_in = 1;
	reg [3:0] i2c_count;
	reg i2c_ack;
	wire SDA_DEBUG_IN, SCL_DEBUG_IN;
	reg [7:0]sda_byte;


	GSR GSR_INST (.GSR (rstn));
	PUR PUR_INST (.PUR (purn));


	bno055_driver bno055(
		.scl1(scl1),
		.sda1(sda1),
		.scl2(scl2),
		.sda2(sda2),
		.rstn( (rstn) ),
		.rstn_imu(rstn_imu),
		.data_out(data_rx),
		.SDA_DEBUG_IN(SDA_DEBUG_IN),
		.SCL_DEBUG_IN(SCL_DEBUG_IN),
		.clk(clk)
		); /* synthesis syn_hier=hard */;

// Generate a slave ACK every 9 i2c SCL posedges, regardless of what data is on the bus
	always@(posedge scl1, negedge rstn) begin
		if(~rstn) begin
			i2c_count = 1'b0;
			i2c_ack = 1'b0;
		end
		else begin
			i2c_count = i2c_count + 1'b1;
			if(i2c_count == 4'd9) begin
				i2c_ack = 1'b1;
				#100
				i2c_ack = 1'b0;
				i2c_count = 1'b0;
				sda_byte = 8'b0;
			end
			else begin
				i2c_ack = 1'b0;
				sda_byte = {sda_byte[6:0], sda1};
			end
		end
	end

	assign ( pull1, strong0 ) scl1 = 1'b1;
	assign ( pull1, strong0 ) sda1 = (i2c_ack == 1'b1) ? 1'b0: 1'b1;
	initial begin
		rstn = 1;
		#10 rstn = 0;
		#10 rstn = 1;
		read_write_in = 0;
		#1_000_000_000;
		$stop;
		end
endmodule
