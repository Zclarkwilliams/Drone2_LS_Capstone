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
	output wire rstn_imu,                 		// Low active reset signal to IMU hardware to trigger reset
	input  wire [5:0] target_read_count_efb1,   // The number of bytes to for the continuous read burst - Max value is 31 bytes
	input  wire [6:0] slave_address_efb1,       // Slave address to access
	output wire [7:0] module_data_out_efb1,     // Received data byte for i2c read cycles
	input  wire [7:0] module_data_in_efb1,      // Byte input for i2c writes
	input  wire [7:0] module_reg_in_efb1,       // Register address to access in i2c write cycles
	input  wire read_write_in_efb1,             // Input bit that indicates whether transaction is a read or a write, should be set before "go" is asserted
	input  wire go_efb1,                        // Input signal to i2c module to begin transaction, needs to be asserted until busy output signal is returned
	output wire busy_efb1,                      // Busy signal out from module while running an i2c transaction
	output wire one_byte_ready_efb1,            // Strobed when a data byte is read, signals that data has been latched
	inout  wire scl_2,              			// I2C EFB #2 SCL wires
	inout  wire sda_2,              			// I2C EFB #2 SDA wires
	input  wire [5:0] target_read_count_efb2,   // The number of bytes to for the continuous read burst - Max value is 31 bytes
	input  wire [6:0] slave_address_efb2,       // Slave address to access
	output wire [7:0] module_data_out_efb2,     // Received data byte for i2c read cycles
	input  wire [7:0] module_data_in_efb2,      // Byte input for i2c writes
	input  wire [7:0] module_reg_in_efb2,       // Register address to access in i2c write cycles
	input  wire read_write_in_efb2,             // Input bit that indicates whether transaction is a read or a write, should be set before "go" is asserted
	input  wire go_efb2,                        // Input signal to i2c module to begin transaction, needs to be asserted until busy output signal is returned
	output wire busy_efb2,                      // Busy signal out from module while running an i2c transaction
	output wire one_byte_ready_efb2,            // Strobed when a data byte is read, signals that data has been latched
	input  wire sys_clk                   		// master clock for module, efb, and output to higher level modules
);

	wire rstn_local;                                 // Manual EFB I2C reset
	wire [7:0] addr1;                                // Wishbone address register
	wire [7:0] data_tx1;                             // Temp storage of data to be written
	wire [7:0] data_rx;                             // Temp storage of received data
	wire ack;                                       // Ack from slave
	wire we1;                               // Write enable, 1 for write, 0 for read
	wire stb1;                             // Strobe from master
	wire cyc1;                                       // Cycle start from master
	wire [7:0] addr2;                                // Wishbone address register
	wire [7:0] data_tx2;                             // Temp storage of data to be written
	wire we2;                               		 // Write enable, 1 for write, 0 for read
	wire stb2;                                       // Strobe from master
	wire cyc2;                                       // Cycle start from master
	wire irq1_out, irq2_out;                         // IRQ output from EFB i2c modules
	wire ack_flag, next_ack_flag;                    // Used to delay read of EFB ack set/clear by one clock and prevent ack in one state from being considered for following states
	wire data_latch;                                 // Strobe to data late to retain dataRX value, which in turn generates module data output
	wire next_data_latch;                            // The next data_latch value that will be asserted at the following clock edge.


	// Connect the I2C module to this top module
	I2C_EFB_WB i2c_top(
		.wb_clk_i(sys_clk),					// Positive edge clock, >7.5x I2C rate
		.wb_rst_i( ~(rstn & rstn_local) ),	// Active-high, synchronous reset signal that will only reset the WISHBONE interface logic.
		.wb_cyc_i(cyc1|cyc2),						// Active high start of bus cycle
		.wb_stb_i(stb1|stb2),						// Active high strobe, WISHBONE slave is the target for current transaction
		.wb_we_i(we1|we2),						// Read/Write control, 1=Write, 0=Read
		.wb_adr_i(addr1|addr2),					// 8-bit address of EFB register
		.wb_dat_i(data_tx2|data_tx2),					// Transmitted data byte TO EFB
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
	i2c_module1_mid i2c1(
					.rstn(rstn),
					.rstn_imu(rstn_imu),
					.target_read_count(target_read_count_efb1),
					.slave_address(slave_address_efb1),
					.module_data_out(module_data_out_efb1),
					.module_data_in(module_data_in_efb1),
					.module_reg_in(module_reg_in_efb1),
					.read_write_in(read_write_in_efb1),
					.go(go_efb1),
					.busy(busy_efb1),
					.one_byte_ready(one_byte_ready_efb1),
					.sys_clk(sys_clk),
					//To EFB
					.ack(ack),
					.cyc(cyc1),						// Active high start of bus cycle
					.stb(stb1),						// Active high strobe, WISHBONE slave is the target for current transaction
					.we(we1),						// Read/Write control, 1=Write, 0=Read
					.addr(addr1),					// 8-bit address of EFB register
					.data_tx(data_tx1),					// Transmitted data byte TO EFB
					.data_rx(data_rx),					// Received data byte from EFB
					.rstn_local(rstn_local)
	);
	
	//  Instantiate i2c driver
	i2c_module2_mid i2c2(
					.rstn(rstn),
					.target_read_count(target_read_count_efb2),
					.slave_address(slave_address_efb2),
					.module_data_out(module_data_out_efb2),
					.module_data_in(module_data_in_efb2),
					.module_reg_in(module_reg_in_efb2),
					.read_write_in(read_write_in_efb2),
					.go(go_efb2),
					.busy(busy_efb2),
					.one_byte_ready(one_byte_ready_efb2),
					.sys_clk(sys_clk),
					//To EFB
					.ack(ack),
					.cyc(cyc2),						// Active high start of bus cycle
					.stb(stb2),						// Active high strobe, WISHBONE slave is the target for current transaction
					.we(we2),						// Read/Write control, 1=Write, 0=Read
					.addr(addr2),					// 8-bit address of EFB register
					.data_tx(data_tx2),					// Transmitted data byte TO EFB
					.data_rx(data_rx)					// Received data byte from EFB
					//.rstn_local(rstn_local)
	);


endmodule

