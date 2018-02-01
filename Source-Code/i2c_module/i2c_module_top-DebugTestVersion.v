
`timescale 1 ps / 1 ps
`include "i2c_module_defines.v"
// Version 6 - Streamlined and flattened

/*

          Lattice Mapping Report File for Design Module 'i2c_module'



Design Information

Target Vendor:  LATTICE
Target Device:  LCMXO3LF-6900CCABGA256
Target Performance:   5
Mapper:  xo3c00f,  version:  Diamond (64-bit) 3.10.1.112
Mapped on:  01/29/18  16:57:23


Design Summary
   Number of registers:     56 out of  7485 (1%)
      PFU registers:           56 out of  6864 (1%)
      PIO registers:            0 out of   621 (0%)
   Number of SLICEs:       129 out of  3432 (4%)
      SLICEs as Logic/ROM:    129 out of  3432 (4%)
      SLICEs as RAM:            0 out of  2574 (0%)
      SLICEs as Carry:          5 out of  3432 (0%)
   Number of LUT4s:        258 out of  6864 (4%)
      Number used as logic LUTs:        248
      Number used as distributed RAM:     0
      Number used as ripple logic:       10
      Number used as shift registers:     0
   Number of PIO sites used: 31 + 4(JTAG) out of 207 (17%)
   Number of block RAMs:  0 out of 26 (0%)
   Number of GSRs:  1 out of 1 (100%)
   EFB used :       Yes
   JTAG used :      No
   Readback used :  No
   Oscillator used :  Yes
   Startup used :   No
   POR :            On
   Bandgap :        On
   Number of Power Controller:  0 out of 1 (0%)
   Number of Dynamic Bank Controller (BCINRD):  0 out of 6 (0%)
   Number of Dynamic Bank Controller (BCLVDSO):  0 out of 1 (0%)
   Number of DCCA:  0 out of 8 (0%)
   Number of DCMA:  0 out of 2 (0%)
   Number of PLLs:  0 out of 2 (0%)
   Number of DQSDLLs:  0 out of 2 (0%)
   Number of CLKDIVC:  0 out of 4 (0%)
   Number of ECLKSYNCA:  0 out of 4 (0%)
   Number of ECLKBRIDGECS:  0 out of 2 (0%)


*/

module i2c_module(sda, scl, rstn, moduleDataOut, moduleDataIn, moduleRegIn, readWriteIn, go, done, clk);
	//
	//  wires && regs
	//

	inout scl ;
	inout sda ;
	output wire [7:0] moduleDataOut;
	input wire  [7:0] moduleDataIn;
	input wire  [7:0] moduleRegIn;
	input wire readWriteIn;
	input wire go;
	output wire done;
	reg  [7:0] addr;  //  Wishbone address register
	reg  [7:0] dataTX;   //  Temp storage of data to be written

	//////////////////////
	//// For Debuging ////
	//Some nonsense to give these wires a connection for now
	assign done = (8'h01&&(moduleRegIn || moduleDataIn || readWriteIn || go ) );
	reg modRegInReg;     //Fixed assignment of module register input, change to moduleRegIn and remove this reg
	reg modDataInReg;    //Fixed assignment of module data input, change to moduleDataIn and remove this reg
	input wire rstn;     //  async negative reset signal 0 = reset, 1 = not reset
	output wire clk;     //  master clock to tb
	wire staticRead = 1;
	//wire clk;
	wire [7:0] dataRX;   //  Temp storage of received data
	//reg [7:0]dataRX;  // Debug
	wire ack;            //  Ack from slave
	//reg ack; //  Debug
	//////////////////////

	reg rstn_local;      //  Manual EFB I2C reset
	reg we, nextWe;      //  Write enable, 1 for write, 0 for read
	reg stb, nextStb;    //  Strobe from master
	wire cyc = stb;      //  Cycle start from master - Strobe and cycle are assigned the same value

	reg readAction;      //  Read flag for calling this function, will be used to determine whether to follow read or write sequence.
	reg writeAction;     //  Write flag for calling this function, will be used to determine whether to follow read or write sequence.
	reg [7:0]nextAddr;   //  Command register address
	reg [7:0]nextDataTX; //  Data written to registers for this command
	reg [(`STATE_BITS-1):0]i2cCmdState;     
	reg [(`STATE_BITS-1):0]i2cCmdNextState; //  State and NexstState for I2C command sequence
	reg [7:0]count_1us;  //  Count from 0 to value determined by clock rate, used to generate 1us delay trigger
	wire irq_out;        //  IRQ output from WB

	reg waiting1us;
	reg clearWaiting1us;


	localparam [7:0]BNO055_CHIP_ID_REG = 8'h00;


	//
	//  Module body
	//

	// Generate module master clock
	clock CLK (clk);

	// Connect the I2C module to this top module
	I2C_EFB_WB i2c_top(
		.wb_clk_i(clk),     //  Positive edge clock, >7.5x I2C rate
		.wb_rst_i(~(rstn & rstn_local)), //  Active-high, synchronous reset signal that will only reset the WISHBONE interface logic.
		.wb_cyc_i(cyc),     //  Active high start of bus cycle
		.wb_stb_i(stb),     //  Active high strobe, WISHBONE slave is the target for current transaction
		.wb_we_i(we),       //  Read/Write control, 1=Write, 0=Read
		.wb_adr_i(addr),    //  8-bit address of EFB register
		.wb_dat_i(dataTX),  //  Transmitted data byte TO EFB
		.wb_dat_o(dataRX),  //  Received data byte from EFB
		.wb_ack_o(ack),     //  Active-high transfer ack signal from EFB module, indicates that requested transfer is ack'd
		.i2c1_scl(scl),     //  I2C clock inout
		.i2c1_sda(sda),     //  I2C Data  inout
		.i2c1_irqo(irq_out) //  IRQ Outout
		);


	//  Generates a 1us duration delay trigger
	always@(posedge clk, negedge clearWaiting1us, negedge rstn) begin//, negedge clearWaiting1us)
		if( ~clearWaiting1us | ~rstn )begin
			count_1us  <= 1'b0;
			waiting1us <= 1'b0;
		end
		else begin
			if(count_1us >= `WAIT_1US_DIVIDER) begin
				count_1us  <= 1'b0;
				waiting1us <= 1'b0;
			end
			else begin
				count_1us  <= count_1us + 1'b1;
				waiting1us <= 1'b1;
			end
		end
	end

	//  Advance state at each positive clock edge
	// This is positive clocked because the EFB requires input on the positive clock edge (It probably latches on the negative edge internally)
	always@(posedge clk, negedge rstn) begin
		if(~rstn) begin
			we          <= `FALSE;
			stb         <= `FALSE;
			addr        <= `ALL_ZERO;
			dataTX      <= `ALL_ZERO;
			i2cCmdState <= `I2C_CMD_STATE_RESET;
		end
		else begin
			addr        <= nextAddr;
			dataTX      <= nextDataTX;
			we          <= nextWe;
			stb         <= nextStb;
			i2cCmdState <= i2cCmdNextState;
		end
	end

	always@(posedge clk, negedge rstn) begin
		if(~rstn) begin //Default to NO action
				readAction  <= 1'b0;
				writeAction <= 1'b0;
		end
		else begin
			if (~readWriteIn == 1'b1) begin
				readAction  <= 1'b1;
				writeAction <= 1'b0;
			end
			else begin
				readAction  <= 1'b0;
				writeAction <= 1'b1;
			end
		end
	end

	assign moduleDataOut = dataRX;
	//  Determine next state and drive module outputs
	always@(*) begin
		// Default to preserve these values, can be altered in lower steps
		rstn_local          = 1'b1;
		clearWaiting1us     = 1'b1;
		if(~rstn) begin
			clearWaiting1us = `STOP_1US_TIMER;
			nextWe          = `I2C_1_WE_READ;
			nextStb         = `STOP;
			nextAddr        = `ALL_ZERO;
			nextDataTX      = `ALL_ZERO;
			modDataInReg    = `ALL_ZERO;
			modRegInReg     = `ALL_ZERO;
			clearWaiting1us = `STOP_1US_TIMER;
			i2cCmdNextState = `I2C_CMD_STATE_RESET;
			rstn_local      = 1'b1;
		end
		else begin
			case(i2cCmdState)
				//++++++++++++++++++++++++//
				// Module startup states
				//++++++++++++++++++++++++//
				`I2C_CMD_STATE_RESET: begin
					rstn_local      = 0; //  Trigger manual reset of I2C module
					nextWe          = `I2C_1_WE_READ;
					nextStb         = `STOP;
					nextAddr        = `ALL_ZERO;
					nextDataTX      = `ALL_ZERO;
					//////////////////
					// debug test
					//  Force wait here until readWrteIn is 0, which is a manual start trigger
					//////////////////
					if(readWriteIn == 1'b0) begin
						clearWaiting1us = `START_1US_TIMER;
						i2cCmdNextState = `I2C_CMD_STATE_INIT1_BOOT_WAIT1;
					end
					else begin
						clearWaiting1us = `START_1US_TIMER;
						i2cCmdNextState = `I2C_CMD_STATE_RESET;
					end
					//////////////////
				end
				`I2C_CMD_STATE_INIT1_BOOT_WAIT1: begin///  Wait first 1us i2c boot-up delay after reset
					rstn_local          = 1'b1;
					if(waiting1us == 1'b1) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						clearWaiting1us = `START_1US_TIMER;
						i2cCmdNextState = `I2C_CMD_STATE_INIT1_BOOT_WAIT2;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						clearWaiting1us = `START_1US_TIMER;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_INIT1_BOOT_WAIT2: begin///  Wait first 1us i2c boot-up delay after reset
					
					rstn_local          = 1'b1;
					if(waiting1us == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						clearWaiting1us = `STOP_1US_TIMER;
						i2cCmdNextState = `I2C_CMD_STATE_PRESCALE_LOW;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						clearWaiting1us = `START_1US_TIMER;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_PRESCALE_LOW: begin
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_BR0;
						nextDataTX      = 8'd84;
						clearWaiting1us = `STOP_1US_TIMER;
						i2cCmdNextState = `I2C_CMD_STATE_WAIT_NO_ACK1;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_BR0;
						nextDataTX      = 8'd84; //Prescaler low = WB_freq/(I2C_prescale*4) should = 400kHz, WB_freq = 133 MHz at the moment
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_WAIT_NO_ACK1: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_PRESCALE_HI;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_PRESCALE_HI: begin //Writing to pre-scaler high byte will trigger EFB module reset
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_BR1;
						nextDataTX      = 8'h00;
						clearWaiting1us = `STOP_1US_TIMER;
						i2cCmdNextState = `I2C_CMD_STATE_WAIT_NO_ACK2;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_BR1;
						nextDataTX      = 8'h00;
						clearWaiting1us = `STOP_1US_TIMER;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_WAIT_NO_ACK2: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						clearWaiting1us = `START_1US_TIMER;
						i2cCmdNextState = `I2C_CMD_STATE_INIT2_BOOT_WAIT1;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						clearWaiting1us = `START_1US_TIMER;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_INIT2_BOOT_WAIT1: begin//  Start second 1us i2c boot-up delay after this reset
					if(waiting1us == 1) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						clearWaiting1us = `START_1US_TIMER;
						i2cCmdNextState = `I2C_CMD_STATE_INIT2_BOOT_WAIT2;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						clearWaiting1us = `START_1US_TIMER;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_INIT2_BOOT_WAIT2: begin//  Complete 1us i2c boot-up delay
					if(waiting1us == 0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						clearWaiting1us = `STOP_1US_TIMER;
						i2cCmdNextState = `I2C_CMD_STATE_INIT_ENA;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						clearWaiting1us = `START_1US_TIMER;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_INIT_ENA: begin //   enable I2C core
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_CR;
						nextDataTX      = `I2C_1_CR_I2CEN;;
						i2cCmdNextState = `I2C_CMD_STATE_WAIT;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_CR;
						nextDataTX      = `I2C_1_CR_I2CEN;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_WAIT_NO_ACK3: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_WAIT;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_WAIT: begin //Wait for next step
					//if( (readAction == `TRUE) | (writeAction == `TRUE) ) begin skipped for now, forcing read
					if( (staticRead == 1'b1) ) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_WAIT_NO_ACK4;
					end
					else begin
						//////////////////
						//debug test
						modDataInReg    = 8'h00;
						$stop; //Halt test bench, does not synthesize
						//////////////////
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_WAIT_NO_ACK4: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_WAIT_NOT_BUSY;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_WAIT_NOT_BUSY: begin //Wait for next step, check BUSY bit before leaving
					if( (ack == 1'b1) && (dataRX[`I2C_1_SR_BUSY] == `NOT_BUSY) ) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						/*if     (writeAction == `TRUE)
							i2cCmdNextState = `I2C_CMD_STATE_W_SET_SLAVE;
						else if(readAction  == `TRUE)
							i2cCmdNextState = `I2C_CMD_STATE_R_SET_SLAVE_WRITE;
						else*///Forcing to read if this happens, just for now
							i2cCmdNextState = `I2C_CMD_STATE_R_SET_SLAVE_WRITE;
							  //Weirdness.... just restart
							//i2cCmdNextState = `I2C_CMD_STATE_RESET;
					end
					else begin //I2C_1_SR_BUSY is still busy, wait until last transaction completes
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
	
				//++++++++++++++++++++++++//
				// Start of WRITE sequence
				//++++++++++++++++++++++++//
				`I2C_CMD_STATE_W_SET_SLAVE: begin //   Set target slave address with write bit appended
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_WAIT_NO_ACK1;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_TXDR;
						nextDataTX      = {`BNO055_SLAVE_ADDRESS,`WR_BIT};
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_WAIT_NO_ACK1: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_SET_WRITE;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_SET_WRITE: begin //   Send I2C command STA and WR
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_WAIT_NO_ACK2;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_CMDR;
						nextDataTX      = (`I2C_1_CMDR_STA || `I2C_1_CMDR_WR);
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_WAIT_NO_ACK2: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_READ_CHK_SR1;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_READ_CHK_SR1: begin//   Check BUSY bit
					if( (ack == 1'b1) &&(dataRX[`I2C_1_SR_TRRDY] == 1'b1) ) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_WAIT_NO_ACK3;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_WAIT_NO_ACK3: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_SET_S_REG;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_SET_S_REG: begin //  Store the slave's register/memory to access
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_WAIT_NO_ACK4;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_TXDR;
						nextDataTX      = modDataInReg;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_WAIT_NO_ACK4: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_WRITE_REG;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_WRITE_REG: begin //  Send I2C command WR to write the register address to the slave
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_WAIT_NO_ACK5;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_CMDR;
						nextDataTX      = (`I2C_1_CMDR_WR);
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_WAIT_NO_ACK5: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_READ_CHK_SR2;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_READ_CHK_SR2: begin//  Check BUSY bit
					if( (ack == 1'b1) && (dataRX[`I2C_1_SR_TRRDY] == 1'b1) ) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_WAIT_NO_ACK6;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_WAIT_NO_ACK6: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_SET_REG_VAL;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_SET_REG_VAL: begin //  Send data to write to the slave's register
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_WAIT_NO_ACK7;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_TXDR;
						nextDataTX      = modDataInReg;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_WAIT_NO_ACK7: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_WRITE_REG_VAL;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_WRITE_REG_VAL: begin //  Send I2C commands WR and STO to write to the slave's register and stop I2C, this is the last write transaction
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_WAIT_NO_ACK8;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_CMDR;
						nextDataTX      = (`I2C_1_CMDR_STO || `I2C_1_CMDR_WR);
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_WAIT_NO_ACK8: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_W_READ_CHK_SR3;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_W_READ_CHK_SR3: begin//   Check BUSY bit
					if( (ack == 1'b1) & (dataRX[`I2C_1_SR_TRRDY] == 1'b1) ) begin  //  If transaction is completed go back to wait
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_WAIT; 
					end
					else begin                                         //  Wait for transaction to complete
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end

				//++++++++++++++++++++++++//
				// Start of READ sequence
				//++++++++++++++++++++++++//
				`I2C_CMD_STATE_R_SET_SLAVE_WRITE: begin //   Set target slave address with write bit appended
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_TXDR;
						nextDataTX      = {`BNO055_SLAVE_ADDRESS,`WR_BIT};
						i2cCmdNextState = `I2C_CMD_STATE_R_WAIT_NO_ACK1;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_TXDR;
						nextDataTX      = {`BNO055_SLAVE_ADDRESS,`WR_BIT};
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_WAIT_NO_ACK1: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_SET_WRITE1;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_SET_WRITE1: begin //   Send I2C command STA and WR
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_CMDR;
						nextDataTX      = (`I2C_1_CMDR_STA || `I2C_1_CMDR_WR);
						i2cCmdNextState = `I2C_CMD_STATE_R_WAIT_NO_ACK2;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_CMDR;
						nextDataTX      = (`I2C_1_CMDR_STA || `I2C_1_CMDR_WR);
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_WAIT_NO_ACK2: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = 6'd60;
						//i2cCmdNextState = `I2C_CMD_STATE_R_READ_CHK_SR1;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				
				
/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Added for debug, read back written values//////////////
/////////////////////////////////////////////////////////////////////////////////////////
				6'd60: begin //   Set target slave address with write bit appended
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_TXDR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = 6'd61;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_TXDR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				6'd61: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = 6'd62;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				6'd62: begin //   Send I2C command STA and WR
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_RXDR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = 6'd63;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_RXDR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				6'd63: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_READ_CHK_SR1;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				
/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// End of Debug section                    s//////////////
/////////////////////////////////////////////////////////////////////////////////////////


				`I2C_CMD_STATE_R_READ_CHK_SR1: begin//   Check TRRDY bit
					if( (ack == 1'b1) && (dataRX[`I2C_1_SR_TRRDY] == 1'b1) ) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_WAIT_NO_ACK3;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_WAIT_NO_ACK3: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_SET_S_REG;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_SET_S_REG: begin //  Store the slave's register/memory to access
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_TXDR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_WAIT_NO_ACK4;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_TXDR;
						nextDataTX      = `ALL_ZERO; 	//This is a trash value for now, change to something good - But this is the BNO055's chip ID register (0x00)
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_WAIT_NO_ACK4: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_WRITE_REG;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_WRITE_REG: begin //  Send I2C command WR to write the register address to the slave
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_CMDR;
						nextDataTX      = (`I2C_1_CMDR_WR);
						i2cCmdNextState = `I2C_CMD_STATE_R_WAIT_NO_ACK5;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_CMDR;
						nextDataTX      = (`I2C_1_CMDR_WR);
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_WAIT_NO_ACK5: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_READ_CHK_SR2;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_READ_CHK_SR2: begin //  Check TRRDY bit
					if( (ack == 1'b1) && (dataRX[`I2C_1_SR_TRRDY] == 1'b1) ) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_WAIT_NO_ACK6;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_WAIT_NO_ACK6: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_SET_SLAVE_READ;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_SET_SLAVE_READ: begin //  Set target slave address again, but with READ bit appended
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_TXDR;
						nextDataTX      = {`BNO055_SLAVE_ADDRESS,`RD_BIT};
						i2cCmdNextState = `I2C_CMD_STATE_R_WAIT_NO_ACK7;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_TXDR;
						nextDataTX      = {`BNO055_SLAVE_ADDRESS,`RD_BIT};
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_WAIT_NO_ACK7: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_SET_WRITE2;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_SET_WRITE2: begin //  Send I2C commands WR and STA (Second STA) to write to the slave's register, this restarts I2C in read mode
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_CMDR;
						nextDataTX      = (`I2C_1_CMDR_STA ||`I2C_1_CMDR_WR);
						i2cCmdNextState = `I2C_CMD_STATE_R_WAIT_NO_ACK8;
					end
					else begin
						nextWe          = `I2C_1_WE_WRITE;
						nextStb         = `START;
						nextAddr        = `I2C_1_CMDR;
						nextDataTX      = (`I2C_1_CMDR_STA ||`I2C_1_CMDR_WR);
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_WAIT_NO_ACK8: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_WAIT_SRW;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_WAIT_SRW: begin //  Check SRW bit (Slave Read/Write, 1 = Master read mode)
					if( (ack == 1'b1) && (dataRX[`I2C_1_SR_SRW] == `SRW_MODE) ) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_WAIT_NO_ACK9;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_WAIT_NO_ACK9: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_SET_READ_STOP;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_SET_READ_STOP: begin 	//  Read the register single byte and also signal stop, this is the last byte to read
														//  I2C commands STO, RD, and ACK are sent, this will be the last byte received
														//  I2C bit 'I2C_1_CMDR_ACK' is included here because the master must signal NACK to the slave
														//  when it receives the byte, this indicates that this is the last byte to send
														//  ACK = 1 => NACK from master to slave, ACK = 0 (Default) => ACK from master to slave
														//  If master sends ACK the slave can send the value of the next register, and the next register,
														//  and so on. Until the master sends NACK, then the process stops.
					if(ack == 1'b1) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_CMDR;
						nextDataTX      = ( `I2C_1_CMDR_STO || (`I2C_1_CMDR_RD || `I2C_1_CMDR_ACK) );
						i2cCmdNextState = `I2C_CMD_STATE_R_WAIT_NO_ACK10;
					end
						else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_CMDR;
						nextDataTX      = ( `I2C_1_CMDR_STO || (`I2C_1_CMDR_RD || `I2C_1_CMDR_ACK) );
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_WAIT_NO_ACK10: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_READ_CHK_SR3;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_READ_CHK_SR3: begin //   Check TRRDY
					if( (ack == 1'b1) && (dataRX[`I2C_1_SR_TRRDY] == 1'b1) ) begin //
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_WAIT_NO_ACK11;
						$display("Done: Received an ACK, SR=%b", dataRX);
					end
					else if( (ack == 1'b1) && (dataRX[`I2C_1_SR_TRRDY] == 1'b0) ) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
						$display("Received an ACK, waiting for RX");
					end
					else begin                                        
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_SR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
						$display("Waiting for ACK");
					end
				end
				`I2C_CMD_STATE_R_WAIT_NO_ACK11: begin //  Wait for ack to go to 0
					if(ack == 1'b0) begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_R_READ_DATA;
					end
					else begin
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `STOP;
						nextAddr        = `ALL_ZERO;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
				`I2C_CMD_STATE_R_READ_DATA: begin // Read the byte
					if( ack == 1'b1 ) begin //  If transaction is completed, read the byte, go back to wait
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_RXDR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = `I2C_CMD_STATE_WAIT;
					end
					else begin                                         //  Wait for transaction to complete
						nextWe          = `I2C_1_WE_READ;
						nextStb         = `START;
						nextAddr        = `I2C_1_RXDR;
						nextDataTX      = `ALL_ZERO;
						i2cCmdNextState = i2cCmdState;
					end
				end
	
				//++++++++++++++++++++++++//
				// Default case, shouldn't be triggered
				//++++++++++++++++++++++++//
				default: begin
					nextWe          = `I2C_1_WE_READ;
					nextStb         = `STOP;
					nextAddr        = `ALL_ZERO;
					nextDataTX      = `ALL_ZERO;
					clearWaiting1us = `STOP_1US_TIMER;
					i2cCmdNextState = `I2C_CMD_STATE_RESET;
				end
			endcase
		end
	end
endmodule