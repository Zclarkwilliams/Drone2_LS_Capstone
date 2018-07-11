/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell,
 * Brett Creeley,
 * Daniel Christiansen,
 * Kirk Hooper,
 * Zachary Clark-Williams
 */

`timescale 1ns / 1ns
`include "common_defines.v"
`include "i2c_module_defines.v"

module i2c_module_top(
	inout  wire scl_2,              // I2C EFB #1 and #2 SCL wires
	inout  wire sda_2,              // I2C EFB #1 and #2 SDA wires
	input  wire rstn,                      // Async negative global reset signal 0 = reset, 1 = not reset
	input  wire sys_clk,                   // master clock for module, efb, and output to higher level modules
);

	reg  [7:0] addr;                                // Wishbone address register
	reg  [7:0] data_tx;                             // Temp storage of data to be written
	wire [7:0] data_rx;                             // Temp storage of received data
	wire ack;                                       // Ack from slave
	reg  rstn_local;                                // Manual EFB I2C reset
	reg  we, next_we;                               // Write enable, 1 for write, 0 for read
	reg  stb, next_stb;                             // Strobe from master
	wire cyc;                                       // Cycle start from master
	wire irq1_out, irq2_out;                        // IRQ output from EFB i2c modules
	reg  ack_flag, next_ack_flag;                   // Used to delay read of EFB ack set/clear by one clock and prevent ack in one state from being considered for following states
	reg  data_latch;                                // Strobe to data late to retain dataRX value, which in turn generates module data output
	reg  next_data_latch;                           // The next data_latch value that will be asserted at the following clock edge.


	// Connect the I2C module to this top module
	I2C_EFB_WB i2c_top(
		.wb_clk_i(sys_clk),					// Positive edge clock, >7.5x I2C rate
		.wb_rst_i( ~(rstn & rstn_local) ),	// Active-high, synchronous reset signal that will only reset the WISHBONE interface logic.
		.wb_cyc_i(cyc),						// Active high start of bus cycle
		.wb_stb_i(stb),						// Active high strobe, WISHBONE slave is the target for current transaction
		.wb_we_i(we),						// Read/Write control, 1=Write, 0=Read
		.wb_adr_i(addr),					// 8-bit address of EFB register
		.wb_dat_i(data_tx),					// Transmitted data byte TO EFB
		.wb_dat_o(data_rx),					// Received data byte from EFB
		.wb_ack_o(ack),						// Active-high command ack signal from EFB module, indicates that requested command is ack'd
		.i2c1_scl(scl_1),					// I2C #2 scl inout
		.i2c1_sda(sda_1),					// I2C #2 sda  inout
		.i2c1_irqo(irq1_out),				// I2C #2 IRQ Output
		.i2c2_scl(scl_2),					// I2C #2 scl inout
		.i2c2_sda(sda_2),					// I2C #2 sda inout
		.i2c2_irqo(irq2_out)				// I2C #2 IRQ Output
);

	// #0.100 forces delay during simulation to prevent mismatch with real synthesized behavior
	assign #0.100 cyc = stb; // Strobe and cycle are assigned the same value

	// Advance state and next values at each positive clock edge
	always@(posedge sys_clk, negedge rstn) begin
		if( ~(rstn) ) begin
			we             <= `FALSE;
			stb            <= `FALSE;
			addr           <= `BYTE_ALL_ZERO;
			data_tx        <= `BYTE_ALL_ZERO;
			ack_flag       <= `FALSE;
			data_latch     <= `FALSE;
		end
		else begin
			addr           <= next_addr;
			data_tx        <= next_data_tx;
			we             <= next_we;
			stb            <= next_stb;
			ack_flag       <= next_ack_flag;
			data_latch     <= next_data_latch;
		end
	end


endmodule

