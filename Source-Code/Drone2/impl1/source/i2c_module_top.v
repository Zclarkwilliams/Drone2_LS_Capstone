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

	
	inout  wire scl_1,              			// I2C EFB #1 SCL wires
	inout  wire sda_1,              			// I2C EFB #1 SDA wires,
	input  wire rstn,               			// Async negative global reset signal 0 = reset, 1 = not reset
	output reg  rstn_imu                   		// Low active reset signal to IMU hardware to trigger reset
	input  wire [5:0] target_read_count_efb1,   // The number of bytes to for the continuous read burst - Max value is 31 bytes
	input  wire [6:0] slave_address_efb1,       // Slave address to access
	output reg  [7:0] module_data_out_efb1,     // Received data byte for i2c read cycles
	input  wire [7:0] module_data_in_efb1,      // Byte input for i2c writes
	input  wire [7:0] module_reg_in_efb1,       // Register address to access in i2c write cycles
	input  wire read_write_in_efb1,             // Input bit that indicates whether transaction is a read or a write, should be set before "go" is asserted
	input  wire go_efb1,                        // Input signal to i2c module to begin transaction, needs to be asserted until busy output signal is returned
	output reg  busy_efb1,                      // Busy signal out from module while running an i2c transaction
	output reg  one_byte_ready_efb1,            // Strobed when a data byte is read, signals that data has been latched
	inout  wire scl_2,              			// I2C EFB #2 SCL wires
	inout  wire sda_2,              			// I2C EFB #2 SDA wires
	input  wire [5:0] target_read_count_efb2,   // The number of bytes to for the continuous read burst - Max value is 31 bytes
	input  wire [6:0] slave_address_efb2,       // Slave address to access
	output reg  [7:0] module_data_out_efb2,     // Received data byte for i2c read cycles
	input  wire [7:0] module_data_in_efb2,      // Byte input for i2c writes
	input  wire [7:0] module_reg_in_efb2,       // Register address to access in i2c write cycles
	input  wire read_write_in_efb2,             // Input bit that indicates whether transaction is a read or a write, should be set before "go" is asserted
	input  wire go_efb2,                        // Input signal to i2c module to begin transaction, needs to be asserted until busy output signal is returned
	output reg  busy_efb2,                      // Busy signal out from module while running an i2c transaction
	output reg  one_byte_ready_efb2,            // Strobed when a data byte is read, signals that data has been latched
	input  wire sys_clk                   		// master clock for module, efb, and output to higher level modules
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

	//  Instantiate i2c driver
	i2c_module1 i2c1(.scl_1(scl_1),
					.sda_1(sda_1),
					.rstn(rstn),
					.rstn_imu(rstn_imu),
					.target_read_count(target_read_count),
					.slave_address(slave_address),
					.module_data_out(data_rx),
					.module_data_in(data_tx),
					.module_reg_in(data_reg),
					.read_write_in(read_write_in),
					.go(go),
					.busy(busy),
					.one_byte_ready(one_byte_ready),
					.i2c_number(i2c_number),
					.sys_clk(sys_clk),
					//To EFB
					.ack(ack),
					.wb_cyc_i(cyc),						// Active high start of bus cycle
					.wb_stb_i(stb),						// Active high strobe, WISHBONE slave is the target for current transaction
					.wb_we_i(we),						// Read/Write control, 1=Write, 0=Read
					.wb_adr_i(addr),					// 8-bit address of EFB register
					.wb_dat_i(data_tx),					// Transmitted data byte TO EFB
					.wb_dat_o(data_rx)					// Received data byte from EFB
	);
	
	//  Instantiate i2c driver
	i2c_module2 i2c2(.scl_2(scl_2),
					.sda_2(sda_2),
					.rstn(rstn),
					.rstn_imu(rstn_imu),
					.target_read_count(target_read_count),
					.slave_address(slave_address),
					.module_data_out(data_rx),
					.module_data_in(data_tx),
					.module_reg_in(data_reg),
					.read_write_in(read_write_in),
					.go(go),
					.busy(busy),
					.one_byte_ready(one_byte_ready),
					.i2c_number(i2c_number),
					.sys_clk(sys_clk),
					//To EFB
					.ack(ack),
					.wb_cyc_i(cyc),						// Active high start of bus cycle
					.wb_stb_i(stb),						// Active high strobe, WISHBONE slave is the target for current transaction
					.wb_we_i(we),						// Read/Write control, 1=Write, 0=Read
					.wb_adr_i(addr),					// 8-bit address of EFB register
					.wb_dat_i(data_tx),					// Transmitted data byte TO EFB
					.wb_dat_o(data_rx)					// Received data byte from EFB
	);


endmodule

