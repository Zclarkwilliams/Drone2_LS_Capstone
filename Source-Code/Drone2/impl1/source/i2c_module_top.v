/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

`timescale 1ns / 1ns
`include "common_defines.v"
`include "i2c_module_defines.v"

module i2c_module(
	inout  scl_1, scl_2,                   //  I2C EFB #1 and #2 SCL wires
	inout  sda_1, sda_2,                   //  I2C EFB #1 and #2 SDA wires
	input  wire rstn,                      //  Async negative global reset signal 0 = reset, 1 = not reset
	input  wire [5:0] target_read_count,   //  The number of bytes to for the continuous read burst - Max value is 31 bytes
	output reg  [7:0] module_data_out,     //  Received data byte for i2c read cycles
	input  wire [7:0] module_data_in,      //  Byte input for i2c writes
	input  wire [7:0] module_reg_in,       //  Register address to access in i2c write cycles
	input  wire [6:0] slave_address,       //  Slave address to access
	input  wire read_write_in,             //  Input bit that indicates whether transaction is a read or a write, should be set before "go" is asserted
	input  wire go,                        //  Input signal to i2c module to begin transaction, needs to be asserted until busy output signal is returned
	output reg  one_byte_ready,            //  Strobed when a data byte is read, signals that data has been latched
	input  wire sys_clk,                   //  master clock for module, efb, and output to higher level modules
	input  wire i2c_number,                //  I2C EFB module to use 0 = EFB1, , 1= EFB2
	output reg  busy,                      //  Busy signal out from module while running an i2c transaction
	output reg  rstn_imu                   //  Low active reset signal to IMU hardware to trigger reset
);

	reg  [7:0] addr;                                //  Wishbone address register
	reg  [7:0] data_tx;                             //  Temp storage of data to be written
	wire [7:0] data_rx;                             //  Temp storage of received data
	wire ack;                                       //  Ack from slave
	reg  rstn_local;                                //  Manual EFB I2C reset
	reg  we, next_we;                               //  Write enable, 1 for write, 0 for read
	reg  stb, next_stb;                             //  Strobe from master
	wire cyc;                                       //  Cycle start from master
	reg  read_action;                               //  Read flag for calling this function, will be used to determine whether to follow read or write sequence.
	reg  write_action;                              //  Write flag for calling this function, will be used to determine whether to follow read or write sequence.
	reg  [7:0]next_addr;                            //  Command register address
	reg  [7:0]next_data_tx;                         //  Data written to registers for this command
	reg  [(`I2C_STATE_BITS-1):0]i2c_cmd_state;      //  Module FSM state
	reg  [(`I2C_STATE_BITS-1):0]next_i2c_cmd_state; //  Module FSM NEXT state
	reg  [11:0]count_us;                            //  Count from 0 to value determined by clock rate, used to generate 1us delay trigger
	reg  clear_waiting_us;                          //  Start multi-us counter from pre-set start value
	reg  [23:0]count_wd_delay ;                     //  Countdown watchdog timer for hardware reset
	reg  wd_event_active;                           //  The current WD event state, active indicates a watchdog fault
	reg  clear_watchdog;                            //  Reset watchdog to max value
	wire irq1_out, irq2_out;                        //  IRQ output from EFB i2c modules
	reg  ack_flag, next_ack_flag;                   //  Used to delay read of EFB ack set/clear by one clock and prevent ack in one state from being considered for following states
	reg  data_latch;                                //  Strobe to data late to retain dataRX value, which in turn generates module data output
	reg  next_data_latch;                           //  The next data_latch value that will be asserted at the following clock edge.
	reg  [5:0]bytes_read_remain;                    //  The nmumber of bytes still to read, counts down from target_read_count to 0
	reg  clear_read_count;                          //  Clear the current bytes_read_remain value, 0 to clear, 1 to maintain
	reg  next_one_byte_ready;                       //  Next value of one_byte_ready at following sys_clk posedge
	reg  [7:0]wd_event_count;                       //  Count of the number of times that the watchdog timer rest the system, only counts to 128 and freezes to prevent wrap around hiding events
	reg  [7:0]next_wd_event_count;                  //  Next value of watchdog timer event count
	reg [7:0]efb_registers[9:0][1:0];

	// Assign register values
	task set_efb_reg_addresses;
		begin
			efb_registers[`I2C_CR_INDEX]    [`I2C_1_INDEX] = `I2C_1_CR   ;
			efb_registers[`I2C_CMDR_INDEX]  [`I2C_1_INDEX] = `I2C_1_CMDR ;
			efb_registers[`I2C_BR0_INDEX]   [`I2C_1_INDEX] = `I2C_1_BR0  ;
			efb_registers[`I2C_BR1_INDEX]   [`I2C_1_INDEX] = `I2C_1_BR1  ;
			efb_registers[`I2C_TXDR_INDEX]  [`I2C_1_INDEX] = `I2C_1_TXDR ;
			efb_registers[`I2C_SR_INDEX]    [`I2C_1_INDEX] = `I2C_1_SR   ;
			efb_registers[`I2C_GCDR_INDEX]  [`I2C_1_INDEX] = `I2C_1_GCDR ;
			efb_registers[`I2C_RXDR_INDEX]  [`I2C_1_INDEX] = `I2C_1_RXDR ;
			efb_registers[`I2C_IRQ_INDEX]   [`I2C_1_INDEX] = `I2C_1_IRQ  ;
			efb_registers[`I2C_IRQEN_INDEX] [`I2C_1_INDEX] = `I2C_1_IRQEN;
			efb_registers[`I2C_CR_INDEX]    [`I2C_2_INDEX] = `I2C_2_CR;
			efb_registers[`I2C_CMDR_INDEX]  [`I2C_2_INDEX] = `I2C_2_CMDR ;
			efb_registers[`I2C_BR0_INDEX]   [`I2C_2_INDEX] = `I2C_2_BR0  ;
			efb_registers[`I2C_BR1_INDEX]   [`I2C_2_INDEX] = `I2C_2_BR1  ;
			efb_registers[`I2C_TXDR_INDEX]  [`I2C_2_INDEX] = `I2C_2_TXDR ;
			efb_registers[`I2C_SR_INDEX]    [`I2C_2_INDEX] = `I2C_2_SR   ;
			efb_registers[`I2C_GCDR_INDEX]  [`I2C_2_INDEX] = `I2C_2_GCDR ;
			efb_registers[`I2C_RXDR_INDEX]  [`I2C_2_INDEX] = `I2C_2_RXDR ;
			efb_registers[`I2C_IRQ_INDEX]   [`I2C_2_INDEX] = `I2C_2_IRQ  ;
			efb_registers[`I2C_IRQEN_INDEX] [`I2C_2_INDEX] = `I2C_2_IRQEN;
		end
	endtask

	//
	//  Module body
	//
	// Connect the I2C module to this top module
	I2C_EFB_WB i2c_top(
		.wb_clk_i(sys_clk),                   //  Positive edge clock, >7.5x I2C rate
		.wb_rst_i( ~(rstn & rstn_local) ),//  Active-high, synchronous reset signal that will only reset the WISHBONE interface logic.
		.wb_cyc_i(cyc),                   //  Active high start of bus cycle
		.wb_stb_i(stb),                   //  Active high strobe, WISHBONE slave is the target for current transaction
		.wb_we_i(we),                     //  Read/Write control, 1=Write, 0=Read
		.wb_adr_i(addr),                  //  8-bit address of EFB register
		.wb_dat_i(data_tx),               //  Transmitted data byte TO EFB
		.wb_dat_o(data_rx),               //  Received data byte from EFB
		.wb_ack_o(ack),                   //  Active-high command ack signal from EFB module, indicates that requested command is ack'd
		.i2c1_scl(scl_1),                  //  I2C #2 scl inout
		.i2c1_sda(sda_1),                  //  I2C #2 sda  inout
		.i2c1_irqo(irq1_out),             //  I2C #2 IRQ Output
		.i2c2_scl(scl_2),                  //  I2C #2 scl inout
		.i2c2_sda(sda_2),                  //  I2C #2 sda inout
		.i2c2_irqo(irq2_out)              //  I2C #2 IRQ Output
);

	//#0.100 forces delay during simulation to prevent mismatch with real synthesized behavior
	assign #0.100 cyc = stb; // Strobe and cycle are assigned the same value

	//  Generates a multiple of 1us length duration delay trigger
	always@(posedge sys_clk, negedge clear_waiting_us, negedge rstn) begin
		if(~rstn)
			count_us       <= 12'hFFF;
		else if( clear_waiting_us == `CLEAR_US_TIMER )
			count_us       <= (`WAIT_US_DIVIDER*5);
		else if( count_us != 12'hFFF )
			count_us       <= (count_us - 1'b1);
		else
			count_us       <= count_us;
	end

	//  Generates a 30 ms watchdog timer
	always@(posedge sys_clk, negedge rstn) begin
		if(~rstn) begin
			count_wd_delay  <= 24'hFFFFFF;
			wd_event_active <= `FALSE;
		end
		else if ( (clear_watchdog  == `CLEAR_WD_TIMER) || (~rstn_imu) ) begin //If IMU is being rest keep WD from running
			count_wd_delay <= (`WAIT_MS_DIVIDER*8'd30);
			wd_event_active <= `FALSE;
		end
		else if( count_wd_delay != 24'hFFFFFF  ) begin
			count_wd_delay <= (count_wd_delay - 1'b1);
			wd_event_active <= `FALSE;
		end
		else begin
			count_wd_delay  <= count_wd_delay;
			wd_event_active <= `TRUE;
		end
	end

	//  Advance state at each positive clock edge
	always@(posedge sys_clk, negedge rstn) begin
		if( ~(rstn) ) begin
			we             <= `FALSE;
			stb            <= `FALSE;
			addr           <= `BYTE_ALL_ZERO;
			data_tx        <= `BYTE_ALL_ZERO;
			i2c_cmd_state  <= `I2C_STATE_RESET;
			ack_flag       <= `FALSE;
			data_latch     <= `FALSE;
			one_byte_ready <= `FALSE;
			wd_event_count <= `BYTE_ALL_ZERO;
			set_efb_reg_addresses();
		end
		else begin
			addr           <= next_addr;
			data_tx        <= next_data_tx;
			we             <= next_we;
			stb            <= next_stb;
			i2c_cmd_state  <= next_i2c_cmd_state;
			ack_flag       <= next_ack_flag;
			data_latch     <= next_data_latch;
			one_byte_ready <= next_one_byte_ready;
			wd_event_count <= next_wd_event_count;
			set_efb_reg_addresses();
		end
	end

	always@(posedge sys_clk, negedge rstn) begin
		if( ~(rstn) ) begin //Default to NO action
				read_action  <= 1'b0;
				write_action <= 1'b0;
		end
		else begin
			if (~read_write_in == 1'b1) begin
				read_action  <= 1'b1;
				write_action <= 1'b0;
			end
			else begin
				read_action  <= 1'b0;
				write_action <= 1'b1;
			end
		end
	end

	always@(posedge sys_clk, negedge rstn) begin
		if( ~(rstn) ) begin
			module_data_out        <= 8'b00;
			next_one_byte_ready    <= 1'b0;
		end
		else if( (data_latch == 1'b1) && (one_byte_ready == 1'b0) )begin
			module_data_out        <= data_rx;
			next_one_byte_ready    <= 1'b1;
		end
		else begin
			module_data_out        <= module_data_out;
			next_one_byte_ready    <= 1'b0;
		end
	end

	always@(posedge data_latch, negedge clear_read_count, negedge rstn) begin
		if( (rstn == 1'b0) || (clear_read_count == 1'b0) ) begin
			bytes_read_remain <= target_read_count;
		end
		else begin
			bytes_read_remain <= (bytes_read_remain - 1'b1);
		end
	end

	//  Determine next state of FSM and drive EFB inputs
	always@(*) begin
		// Default to preserve these values, can be altered in lower steps
		rstn_local          = `HIGH;
		clear_waiting_us    = `RUN_US_TIMER;
		clear_watchdog      = `RUN_WD_TIMER;
		next_data_latch     = `FALSE;
		busy                = `HIGH;
		rstn_imu            = `HIGH;
		clear_read_count    = `HIGH;
		next_wd_event_count = wd_event_count;
		if( ~(rstn) ) begin  // Block FSM while rstn held low
			rstn_local          = `HIGH;
			clear_waiting_us    = `RUN_US_TIMER;
			clear_watchdog      = `CLEAR_WD_TIMER;
			next_data_latch     = `FALSE;
			busy                = `HIGH;
			rstn_imu            = `LOW;
			clear_read_count    = `HIGH;
			next_wd_event_count = 0;
		end
		else if (wd_event_active) begin //  Handle watchdog timer wrap around, reset IMU and EFB
			rstn_local          = `LOW;
			clear_waiting_us    = `RUN_US_TIMER;
			clear_watchdog      = `CLEAR_WD_TIMER;
			next_data_latch     = `FALSE;
			busy                = `HIGH;
			rstn_imu            = `LOW;
			clear_read_count    = `HIGH;
			next_i2c_cmd_state  = `I2C_STATE_RESET;
			if(wd_event_count[7] != 1'b1) // If this counter already hit 128, just stop, prevent wrap around and masking issue
				next_wd_event_count = wd_event_count + 1'b1; //  Increment event counter
		end
		else begin
			case(i2c_cmd_state)
//++++++++++++++++++++++++++++++++++++++++++++++++//
				// Module startup states
//++++++++++++++++++++++++++++++++++++++++++++++++//
				`I2C_STATE_RESET: begin
					rstn_local         = `LOW; //  Trigger manual reset of EFB I2C module
					clear_watchdog     = `CLEAR_WD_TIMER; // Hold watchdog low, don't run it yet
					next_we            = `I2C_WE_READ;
					next_stb           = `I2C_CMD_STOP;
					next_addr          = `BYTE_ALL_ZERO;
					next_data_tx       = `BYTE_ALL_ZERO;
					rstn_imu           = `LOW;
					next_i2c_cmd_state = `I2C_STATE_SET_PRESCALE_LOW;
					clear_read_count   = `HIGH;
				end
				`I2C_STATE_SET_PRESCALE_LOW: begin //  Set pre-scale LOW byte for WISHBONE clock rate
					clear_watchdog         = `CLEAR_WD_TIMER; // Hold watchdog low, don't run it yet
					if( ( (ack == `TRUE) && (ack_flag == `TRUE) ) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						rstn_imu           = `LOW;
						clear_read_count   = `LOW;
						next_i2c_cmd_state = `I2C_STATE_SET_PRESCALE_HI;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_BR0_INDEX][i2c_number];
						next_data_tx       = 8'd190; // Prescaler value for a 38.00 MHz clock to 400kHz i2c rate
						next_ack_flag      = `TRUE;
						rstn_imu           = `LOW;
						clear_read_count   = `HIGH;
						next_i2c_cmd_state = `I2C_STATE_SET_PRESCALE_LOW;
					end
				end
				`I2C_STATE_SET_PRESCALE_HI: begin	//  Set pre-scale HIGH byte for WISHBONE clock rate, most likely to always be 0x00
													//  Writing this byte forces another reset of EFB module
					clear_watchdog         = `CLEAR_WD_TIMER; // Hold watchdog low, don't run it yet
					if( ( (ack == `TRUE) && (ack_flag == `TRUE) ) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						busy               = `HIGH;
						clear_read_count   = `HIGH;
						next_i2c_cmd_state = `I2C_STATE_INIT_BOOT_WAIT1;
					end
					else begin
						clear_waiting_us   = `CLEAR_US_TIMER;
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_BR1_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;  // Set the high register to 0, not needed for 38.00 MHz clock
						next_ack_flag      = `TRUE;
						busy               = `HIGH;
						clear_read_count   = `HIGH;
						next_i2c_cmd_state = `I2C_STATE_SET_PRESCALE_HI;
					end
				end
				`I2C_STATE_INIT_BOOT_WAIT1: begin //  Start wait of 1us for i2c EFB "boot-up" delay after reset
					clear_watchdog         = `CLEAR_WD_TIMER; // Hold watchdog low, don't run it yet
					next_we                = `I2C_WE_READ;
					next_stb               = `I2C_CMD_STOP;
					next_addr              = `BYTE_ALL_ZERO;
					next_data_tx           = `BYTE_ALL_ZERO;
					busy                   = `HIGH;
					clear_read_count       = `HIGH;
					if(count_us != 12'hFFF)
						next_i2c_cmd_state = `I2C_STATE_INIT_BOOT_WAIT2;
					else
						next_i2c_cmd_state = `I2C_STATE_INIT_BOOT_WAIT1;
				end
				`I2C_STATE_INIT_BOOT_WAIT2: begin //  Wait for 1us wait to complete
					clear_watchdog         = `CLEAR_WD_TIMER; // Hold watchdog low, don't run it yet
					next_we                = `I2C_WE_READ;
					next_stb               = `I2C_CMD_STOP;
					next_addr              = `BYTE_ALL_ZERO;
					next_data_tx           = `BYTE_ALL_ZERO;
					busy                   = `HIGH;
					clear_read_count       = `HIGH;
					if(count_us[11] == 1'b1) //  1us counter wrapped around to 0xFF, wait is done
						next_i2c_cmd_state = `I2C_STATE_INIT_ENA;
					else
						next_i2c_cmd_state = `I2C_STATE_INIT_BOOT_WAIT2;
				end
				`I2C_STATE_INIT_ENA: begin //   enable I2C core and set output delay to 300ns from scl posedge
					clear_watchdog         = `CLEAR_WD_TIMER; // Hold watchdog low, don't run it yet
					if( ( ack == `TRUE ) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						busy               = `HIGH;
						clear_read_count   = `HIGH;
						next_i2c_cmd_state = `I2C_STATE_WAIT;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_CR_INDEX][i2c_number];
						next_data_tx       = (`I2C_CR_I2CEN | `I2C_CR_SDA_DEL_SEL_300NS);
						next_ack_flag      = `TRUE;
						clear_read_count   = `HIGH;
						next_i2c_cmd_state = `I2C_STATE_INIT_ENA;
					end
				end
				`I2C_STATE_WAIT: begin //Wait for next step
					clear_watchdog     = `CLEAR_WD_TIMER; // Hold watchdog low, don't run it yet
					next_we            = `I2C_WE_READ;
					next_stb           = `I2C_CMD_STOP;
					next_addr          = `BYTE_ALL_ZERO;
					next_data_tx       = `BYTE_ALL_ZERO;
					busy               = `LOW;
					clear_read_count   = `HIGH;
					if(go == `HIGH)
						next_i2c_cmd_state = `I2C_STATE_WAIT_NOT_BUSY;
					else
						next_i2c_cmd_state = `I2C_STATE_WAIT;
				end
				`I2C_STATE_WAIT_NOT_BUSY: begin //Wait for next step, check TRRDY and BUSY bits before leaving
					clear_watchdog         = `RUN_WD_TIMER; // Start watchdog timer
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						busy               = `HIGH;
						clear_read_count   = `LOW;
						next_i2c_cmd_state = `I2C_STATE_WAIT_NOT_BUSY;
						if( (data_rx && ( data_rx[`I2C_SR_TRRDY] == `I2C_BUS_TRRDY_READY) && (data_rx[`I2C_SR_BUSY] == `I2C_BUS_BUSY) ) || (data_rx[`I2C_SR_BUSY] == `I2C_BUS_NOT_BUSY) ) begin
							if(read_action)
								next_i2c_cmd_state = `I2C_STATE_R_SET_SLAVE_WRITE;
							else if(write_action)
								next_i2c_cmd_state = `I2C_STATE_W_SET_SLAVE_WRITE;
							// If Neither of these then it will default to wait more, one of them should have been asserted by this point
						end
					end
					else begin //If BUSY is still busy, wait until last transaction completes
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_SR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						busy               = `HIGH;
						clear_read_count   = `LOW;
						next_i2c_cmd_state = `I2C_STATE_WAIT_NOT_BUSY;
					end
				end
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
				// Start of WRITE sequence
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
				`I2C_STATE_W_SET_SLAVE_WRITE: begin //   Set target slave address with write bit appended
					clear_watchdog         = `RUN_WD_TIMER; // Start watchdog timer
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						busy               = `HIGH;
						next_i2c_cmd_state = `I2C_STATE_W_SET_WRITE;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_TXDR_INDEX][i2c_number];
						next_data_tx       = {slave_address,`I2C_BUS_WR_BIT};
						next_ack_flag      = `TRUE;
						busy               = `HIGH;
						next_i2c_cmd_state = `I2C_STATE_W_SET_SLAVE_WRITE;
					end
				end
				`I2C_STATE_W_SET_WRITE: begin //   Send I2C command STA and WR
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_W_READ_CHK_SR1;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_CMDR_INDEX][i2c_number];
						next_data_tx       = (`I2C_CMDR_STA | `I2C_CMDR_WR );
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_W_SET_WRITE;
					end
				end
				`I2C_STATE_W_READ_CHK_SR1: begin //   Check Transmit/Receive Ready bit
					if( ( (ack == `TRUE) && (ack_flag == `TRUE) ) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_W_READ_CHK_SR1;
						if(data_rx && (data_rx[`I2C_SR_TRRDY] == `I2C_BUS_TRRDY_READY) ) begin
								next_i2c_cmd_state = `I2C_STATE_W_SET_SLAVE_REG;
						end
					end
					else begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_SR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_W_READ_CHK_SR1;
					end
				end
				`I2C_STATE_W_SET_SLAVE_REG: begin //  Store the slave's register/memory address to access
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_W_WRITE_SLAVE_REG;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_TXDR_INDEX][i2c_number];
						next_data_tx       = module_reg_in;
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_W_SET_SLAVE_REG;
					end
				end
				`I2C_STATE_W_WRITE_SLAVE_REG: begin //  Send I2C command WR to write the register address to the slave
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_W_READ_CHK_SR2;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_CMDR_INDEX][i2c_number];
						next_data_tx       = (`I2C_CMDR_WR );
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_W_WRITE_SLAVE_REG;
					end
				end
				`I2C_STATE_W_READ_CHK_SR2: begin //   Check Transmit/Receive Ready is Ready
					if( ( (ack == `TRUE) && (ack_flag == `TRUE) ) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_W_READ_CHK_SR2;
						if(data_rx && (data_rx[`I2C_SR_TRRDY] == `I2C_BUS_TRRDY_READY) ) begin
							next_i2c_cmd_state = `I2C_STATE_W_SET_REG_VAL;
						end
					end
					else begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_SR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_W_READ_CHK_SR2;
					end
				end
				`I2C_STATE_W_SET_REG_VAL: begin //  Store the register value
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_W_WRITE_REG_VAL;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_TXDR_INDEX][i2c_number];
						next_data_tx       = module_data_in;
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_W_SET_REG_VAL;
					end
				end
				`I2C_STATE_W_WRITE_REG_VAL: begin //  Send I2C command WR to write the register value to the slave
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_W_READ_CHK_SR3;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_CMDR_INDEX][i2c_number];
						next_data_tx       = (`I2C_CMDR_WR );
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_W_WRITE_REG_VAL;
					end
				end
				`I2C_STATE_W_READ_CHK_SR3: begin //   Check Transmit/Receive Ready is Ready
					if( ( (ack == `TRUE) && (ack_flag == `TRUE) ) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_W_READ_CHK_SR3;
						if(data_rx && (data_rx[`I2C_SR_TRRDY] == `I2C_BUS_TRRDY_READY) ) begin
							next_i2c_cmd_state = `I2C_STATE_W_WRITE_STOP;
						end
					end
					else begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_SR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_W_READ_CHK_SR3;
					end
				end
				`I2C_STATE_W_WRITE_STOP: begin //  Stop i2c transaction
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_W_READ_CHK_SR4;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_CMDR_INDEX][i2c_number];
						next_data_tx       = (`I2C_CMDR_STO);
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_W_WRITE_STOP;
					end
				end
				`I2C_STATE_W_READ_CHK_SR4: begin //   Check bus busy is DE-ASSERTED
					if( ( (ack == `TRUE) && (ack_flag == `TRUE) ) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						if(data_rx && (data_rx[`I2C_SR_BUSY] == `I2C_BUS_NOT_BUSY) )
							next_i2c_cmd_state = `I2C_STATE_WAIT;
						else
							next_i2c_cmd_state = `I2C_STATE_W_READ_CHK_SR4;
					end
					else begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_SR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_W_READ_CHK_SR4;
					end
				end
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
				// Start of READ sequence
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
				`I2C_STATE_R_SET_SLAVE_WRITE: begin //   Set target slave address with write bit appended
					clear_read_count       = `LOW; // Clear the previous bytes read counter
					clear_watchdog         = `RUN_WD_TIMER; // Start watchdog timer
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_SET_WRITE1;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_TXDR_INDEX][i2c_number];
						next_data_tx       = {slave_address,`I2C_BUS_WR_BIT};
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_SET_SLAVE_WRITE;
					end
				end
				`I2C_STATE_R_SET_WRITE1: begin //   Send I2C command STA and WR
					clear_read_count   = `HIGH;
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR1;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_CMDR_INDEX][i2c_number];
						next_data_tx       = (`I2C_CMDR_STA | `I2C_CMDR_WR );
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_SET_WRITE1;
					end
				end
				`I2C_STATE_R_READ_CHK_SR1: begin //   Check Transmit/Receive Ready is Ready
					if( ( (ack == `TRUE) && (ack_flag == `TRUE) ) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR1;
						if(data_rx && (data_rx[`I2C_SR_TRRDY] == `I2C_BUS_TRRDY_READY) ) begin
							next_i2c_cmd_state = `I2C_STATE_R_SET_SLAVE_REG;
						end
					end
					else begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_SR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR1;
					end
				end
				`I2C_STATE_R_SET_SLAVE_REG: begin //  Store the slave's register/memory address to access
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_WRITE_REG;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_TXDR_INDEX][i2c_number];
						next_data_tx       = module_reg_in;
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_SET_SLAVE_REG;
					end
				end
				`I2C_STATE_R_WRITE_REG: begin //  Send I2C command WR to send the register address to the slave
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR2;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_CMDR_INDEX][i2c_number];
						next_data_tx       = (`I2C_CMDR_WR );
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_WRITE_REG;
					end
				end
				`I2C_STATE_R_READ_CHK_SR2: begin //   Check Transmit/Receive Ready is Ready and that ACK is received
					if( ( (ack == `TRUE) && (ack_flag == `TRUE) ) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR2;
						if(data_rx && (data_rx[`I2C_SR_RARC] == `I2C_BUS_RARC ) && (data_rx[`I2C_SR_TRRDY] == `I2C_BUS_TRRDY_READY)) begin
								next_i2c_cmd_state = `I2C_STATE_R_SET_SLAVE_READ;
						end
					end
					else begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_SR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR2;
					end
				end
				`I2C_STATE_R_SET_SLAVE_READ: begin //  Set target slave address again, but with READ bit appended
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_SET_WRITE2;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_TXDR_INDEX][i2c_number];
						next_data_tx       = {slave_address,`I2C_BUS_RD_BIT};
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_SET_SLAVE_READ;
					end
				end
				`I2C_STATE_R_SET_WRITE2: begin //  Send I2C commands WR and STA (Second STA), this restarts I2C in read mode
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_WAIT_SRW;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_CMDR_INDEX][i2c_number];
						next_data_tx       = (`I2C_CMDR_STA |`I2C_CMDR_WR );
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_SET_WRITE2;
					end
				end
				`I2C_STATE_R_WAIT_SRW: begin //  Check SRW bit (Slave Read/Write, 1 = Master read mode)
					if( ( (ack == `TRUE) && (ack_flag == `TRUE) ) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_WAIT_SRW;
						if(data_rx && (data_rx[`I2C_SR_SRW] == `I2C_BUS_SRW_MASTER_RX) ) begin
							next_i2c_cmd_state = `I2C_STATE_R_SET_READ;
						end
					end
					else begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_SR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_WAIT_SRW;
					end
				end
				`I2C_STATE_R_SET_READ: begin           // Start of multi/single byte read
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR3;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_CMDR_INDEX][i2c_number];
						next_data_tx       = ( `I2C_CMDR_RD );
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_SET_READ;
					end
				end
				`I2C_STATE_R_READ_CHK_SR3: begin //   Check Transmit/Receive Ready bit
					if( ( (ack == `TRUE) && (ack_flag == `TRUE) ) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_data_latch    = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR3;
						if(data_rx && (data_rx[`I2C_SR_TRRDY] == `I2C_BUS_TRRDY_READY))
							next_i2c_cmd_state = `I2C_STATE_R_READ_DATA1;
					end
					else begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_SR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						next_data_latch    = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR3;
					end
				end
				`I2C_STATE_R_READ_DATA1: begin // Read the byte
					if(data_latch) begin //  Byte read completed, move to next state
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_data_latch    = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_SET_READ_STOP;
						if(bytes_read_remain > 5'h01) // Won't have been decremented by this point, so not >= here
							next_i2c_cmd_state = `I2C_STATE_R_SET_READ; // Wait for another byte
					end
					else if( (ack == `TRUE) && (ack_flag == `TRUE) && (data_latch == 1'b0) ) begin //  byte is ready, read the byte
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_RXDR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						if(bytes_read_remain >= 5'h01) begin // Still bytes remaining, latch this byte
							next_data_latch    = `TRUE;
							next_i2c_cmd_state = `I2C_STATE_R_READ_DATA1;
						end                                  // No more bytes left, don't strobe data latch
						else begin
							next_data_latch    = `FALSE;
							next_i2c_cmd_state = `I2C_STATE_R_SET_READ_STOP;
						end
					end
					else begin                                         //  Wait for transaction to complete
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_RXDR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						next_data_latch    = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_DATA1;
					end
				end
				`I2C_STATE_R_SET_READ_STOP: begin   //  Last byte read
													//  Read the register single byte and also signal stop, this is the last byte to read
													//  I2C commands STO, RD, and ACK are sent, this will be the last byte received
													//  I2C bit 'I2C_CMDR_ACK' is included here because the master must signal NACK to the slave
													//  when it receives the byte, this indicates that this is the last byte to send
													//  ACK = 1 => NACK from master to slave, ACK = 0 (Default) => ACK from master to slave
					if( (ack == `TRUE) && (ack_flag == `TRUE) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR4;
					end
					else begin
						next_we            = `I2C_WE_WRITE;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_CMDR_INDEX][i2c_number];
						next_data_tx       = ( `I2C_CMDR_RD );
						next_data_tx       = ( `I2C_CMDR_STO | `I2C_CMDR_RD | `I2C_CMDR_ACK  );
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_SET_READ_STOP;
					end
				end
				`I2C_STATE_R_READ_CHK_SR4: begin //   Check Transmit/Receive Ready bit
					if( ( (ack == `TRUE) && (ack_flag == `TRUE) ) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR4;
						if(data_rx && (data_rx[`I2C_SR_TRRDY] == `I2C_BUS_TRRDY_READY) ) begin
							next_i2c_cmd_state = `I2C_STATE_R_READ_DATA2;
						end
					end
					else begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_SR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR4;
					end
				end
				`I2C_STATE_R_READ_DATA2: begin // Read the byte
					if(data_latch) begin //  Byte read completed, move to next state
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_data_latch    = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR5;
					end
					else if( (ack == `TRUE) && (ack_flag == `TRUE) && (data_latch == 1'b0) ) begin //  byte is ready, read the byte
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_RXDR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						if(bytes_read_remain != 5'h00) begin //  If this was the last byte, latch onti it
							next_data_latch    = `TRUE;
							next_i2c_cmd_state = `I2C_STATE_R_READ_DATA2;
						end                                  //  Otherwise this byte is discarded (Used in single byte reads, EFB doesn't always stop in time for single byte read)
						else begin
							next_data_latch    = `FALSE;
							next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR5;
						end
					end
					else begin                                         //  Wait for transaction to complete
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_RXDR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						next_data_latch    = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_DATA2;
					end
				end

				`I2C_STATE_R_READ_CHK_SR5: begin //   Check bus BUSY bit is DE-ASSERTED
					if( ( (ack == `TRUE) && (ack_flag == `TRUE) ) ) begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_STOP;
						next_addr          = `BYTE_ALL_ZERO;
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `FALSE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR5;
						if(data_rx[`I2C_SR_BUSY] == `I2C_BUS_NOT_BUSY) begin // Don't include data_rx && here, when BUSY is deasserted none of the other bits are guaranteed valid, so 0x00 can also mean not busy, regardless of the other bits being 0
							next_i2c_cmd_state = `I2C_STATE_WAIT;
						end
					end
					else begin
						next_we            = `I2C_WE_READ;
						next_stb           = `I2C_CMD_START;
						next_addr          = efb_registers[`I2C_SR_INDEX][i2c_number];
						next_data_tx       = `BYTE_ALL_ZERO;
						next_ack_flag      = `TRUE;
						next_i2c_cmd_state = `I2C_STATE_R_READ_CHK_SR5;
					end
				end
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
				// Default case, shouldn't be triggered, but here for FSM safety
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
				default: begin
					clear_watchdog     = `RUN_WD_TIMER; // Start watchdog timer
					next_we            = `I2C_WE_READ;
					next_stb           = `I2C_CMD_STOP;
					next_addr          = `BYTE_ALL_ZERO;
					next_data_tx       = `BYTE_ALL_ZERO;
					next_ack_flag      = `FALSE;
					next_i2c_cmd_state = `I2C_STATE_RESET;
				end
			endcase
		end
	end
endmodule

