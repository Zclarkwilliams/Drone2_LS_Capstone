// `include "timescale.v"
`timescale 1ns / 1ns


module i2c_module(scl, sda, rstn, DATA_IN);

// Bit width parameters
	localparam awidth = 4'h8; //  Width of a register address
	localparam dwidth = 4'h8; //  Width of a data chunk, this is a byte
	localparam stateBits = 6; //  The number of bits used to represent the current state
	
	// 
	//  wires && regs
	// 
	inout scl ;
	inout sda ;
	input wire rstn; //   async reset, not from wb
	reg [7:0] DATA_OUT;
	input wire [7:0] DATA_IN;
	wire clk;  //   master clock from wb
	
	reg  [awidth -1:0] adr;    //  Wishbone address register
	reg  [dwidth -1:0] datTX; //  Temp storage of data to be written
	wire [dwidth -1:0] datRX; //  Temp storage of received data
	reg ReadType; //  Read of data or read for "Check Status Register" routine
	reg we;   //  Write enable, 1 for write, 0 for read
	reg stb;  //  Strobe from master
	reg cyc;  //  Cycle start from master
	wire ack; //  Ack from slave
	
	reg READ_CMD;                   //Read flag for calling this function, will be used to determine whether to follow read or write sequence.
	reg WRITE_CMD;                  //Write flag for calling this function, will be used to determine whether to follow read or write sequence.
	reg [awidth -1:0]	cmd_addr;   //Command register address
	reg [dwidth -1:0]	cmd_data;   //Data written to registers for this command
	reg [stateBits:0]CSR_CallerNextStateBuffer, WR_CallerNextStateBuffer, RD_CallerNextStateBuffer; //Next state for Write, Read, and Check status register, once these sequences complete this is where the FSM resumes.
	reg [stateBits:0]CSRcaller_next_state, WRcaller_next_state, RDcaller_next_state; //Buffers for the above regs, stores them until needed later
	reg [7:0]check_SR;                                      //Stores status register value
	reg [stateBits:0]I2C_CMD_State, I2C_CMD_NextState;      //State and NexstState for I2C command sequence
	reg I2C_clk;      										//Clock that runs at the I2C rate
	reg [7:0]count;  //Count from 0 to value determined by clock rate, used to generate I2C rate clock and 1us counter value
	
	reg wait_1us_timer;
	reg clear_wait_1us_timer;


	localparam BNO055_CHIP_ID_REG = 8'h00;
	
	localparam I2C_DIVIDER = 67;     //   Divider to provide a clock posedge every 2.5 uS 1/(400kHz)
	localparam WAIT_1US_DIVIDER = 6; //  Wait 1us at 53.20 MHz
	
	// Registers

	localparam I2C_1_CR    = 8'h40; //  Control register
	localparam I2C_1_CMDR  = 8'h41; //  Command register
	localparam I2C_1_BR0   = 8'h42; //  clock pre-scaler low byte
	localparam I2C_1_BR1   = 8'h43; //  clock pre-scaler high byte
	localparam I2C_1_TXDR  = 8'h44; //  Transmit data
	localparam I2C_1_SR    = 8'h45; //  Status
	localparam I2C_1_GCDR  = 8'h46; //  General Call
	localparam I2C_1_RXDR  = 8'h47; //  Receive data
	localparam I2C_1_IRQ   = 8'h48; //  IRQ
	localparam I2C_1_IRQEN = 8'h49; //  IRQ Enable

	// WE bits
	localparam I2C_1_WE_WRITE = 1;
	localparam I2C_1_WE_READ  = 0;
	
	// Control register bits
	localparam I2C_1_CR_I2CEN             = 8'b0 | {1'b1<<7};  //  Enable I2C module
	localparam I2C_1_CR_GCEN              = 8'b0 | {1'b1<<6};  //  Enable general call
	localparam I2C_1_CR_WKUPEN            = 8'b0 | {1'b1<<5};  //  Enable Wakeup from sleep
	localparam I2C_1_CR_SDA_DEL_SEL_300NS = 8'b0 | {2'b00<<2}; //  SDA output delay of 300ns
	localparam I2C_1_CR_SDA_DEL_SEL_150NS = 8'b0 | {2'b01<<2}; //  SDA output delay of 150ns
	localparam I2C_1_CR_SDA_DEL_SEL_75S   = 8'b0 | {2'b10<<2}; //  SDA output delay of 75s
	localparam I2C_1_CR_SDA_DEL_SEL_0NS   = 8'b0 | {2'b11<<2}; //  SDA output delay of 0ns
	
	// Command register bits
	localparam I2C_1_CMDR_STA   = 8'b0 | {1'b1<<7}; //  Generate start condition
	localparam I2C_1_CMDR_STO   = 8'b0 | {1'b1<<6}; //  Generate stop condition
	localparam I2C_1_CMDR_RD    = 8'b0 | {1'b1<<5}; //  Indicate READ from slave
	localparam I2C_1_CMDR_WR    = 8'b0 | {1'b1<<4}; //  Indicate WRITE to slave
	localparam I2C_1_CMDR_ACK   = 8'b0 | {1'b1<<3}; //  Ack option when receiving, 0 = ACK, 1 = NACK
	localparam I2C_1_CMDR_CKDIS = 8'b0 | {1'b1<<2}; //  Enable clock stretching by slave, 1 = Enabled, 0 = Disabled
	
	// Status register bit numbers
	localparam I2C_1_SR_TIP   = 7; 	// Transmit In Progress. The current data byte is being transferred.signal synchronization. This bit could be high after configuration wake-
									// up and before the first valid I2C transfer start (when BUSY is low), and it is not indicating byte in transfer, but an invalid indicator.
									// 1 = Busy, 0 = Not Busy
	localparam I2C_1_SR_BUSY  = 6;	// I2C Bus Busy - 1 = Busy, 0 = Not Busy
	localparam I2C_1_SR_RARC  = 5;	// Received Acknowledge, 0 = Ack Received, 1 = NO Ack Received
	localparam I2C_1_SR_SRW   = 4;	// Slave Read/Write - Indicates transmit or receive mode 1 = Master receive, 0 = Master transmitting
	localparam I2C_1_SR_ARBL  = 3;	// Arbitration lost - The core lost arbitration in Master mode, 1 = Arbitration lost, 0 = Normal
	localparam I2C_1_SR_TRRDY = 2;	// Transmitter or Receiver Ready = Ready to receive or transmit, 1 = Ready, 0 = Not Ready
	localparam I2C_1_SR_TROE  = 3;	// Transmitter/Receiver Overrun Error, 1 = overrun, 0 = Normal
	localparam I2C_1_SR_HGC   = 2;	// Hardware General Call received, 1 = General call in slave mode, 0 = Normal
	

	// Value aliases
	localparam START     = 1'b1; //  Start bit for both stb and cyc
	localparam STOP      = 1'b0; //  Stop  bit for both stb and cyc
	localparam RD_BIT    = 1'b1; //  Set Read/Write to READ, appended to SLAVE_ADDRESS as SDA transmits to bus
	localparam WR_BIT    = 1'b0; //  Set Read/Write to WRITE, appended to SLAVE_ADDRESS as SDA transmits to bus
	localparam DATA_READ = 0;
	localparam CSR_READ  = 1;
	localparam SLAVE_ADDRESS = 7'b01010_01; // BNO055 SLAVE address = 0x29

	// 
	//  State Definitions
	//  State Definitions
	// 
	// Initial default state
	localparam I2C_CMD_STATE_RESET             = 0;
	localparam I2C_CMD_STATE_INIT_WAIT1        = 1;
	localparam I2C_CMD_STATE_INIT_WAIT2        = 2;
	localparam I2C_CMD_STATE_INIT_ENA          = 3;
	localparam I2C_CMD_STATE_WAIT              = 4;
	
	// Write states
	localparam I2C_CMD_STATE_W_SET_SLAVE       = 5;
	localparam I2C_CMD_STATE_W_SET_WRITE       = 6;
	localparam I2C_CMD_STATE_W_READ_CH_SR1     = 7;
	localparam I2C_CMD_STATE_W_SET_S_REG       = 8;
	localparam I2C_CMD_STATE_W_WRITE_REG       = 9;
	localparam I2C_CMD_STATE_W_READ_CH_SR2     = 10;
	localparam I2C_CMD_STATE_W_SET_REG_VAL     = 11;
	localparam I2C_CMD_STATE_W_WRITE_REG_VAL   = 12;
	localparam I2C_CMD_STATE_W_READ_CH_SR3     = 13;
	localparam I2C_CMD_STATE_W_WRITE_STOP      = 14;
	localparam I2C_CMD_STATE_W_READ_CH_SR4     = 15;
	
	// Read states
	localparam I2C_CMD_STATE_R_SET_SLAVE_WRITE = 16;
	localparam I2C_CMD_STATE_R_SET_WRITE1      = 17;
	localparam I2C_CMD_STATE_R_READ_CH_SR1     = 18;
	localparam I2C_CMD_STATE_R_SET_S_REG       = 19;
	localparam I2C_CMD_STATE_R_WRITE_REG       = 20;
	localparam I2C_CMD_STATE_R_READ_CH_SR2     = 21;
	localparam I2C_CMD_STATE_R_SET_SLAVE_READ  = 22;
	localparam I2C_CMD_STATE_R_SET_WRITE2      = 23;
	localparam I2C_CMD_STATE_R_WAIT_SRW1       = 24;
	localparam I2C_CMD_STATE_R_WAIT_SRW2       = 25;
	localparam I2C_CMD_STATE_R_SET_READ        = 26;
	localparam I2C_CMD_STATE_R_CMD_READ_STOP   = 27; //RD+NACK+STOP = 0x6C
	localparam I2C_CMD_STATE_R_READ_CH_SR3     = 28;
	
	// States for checking status register
	localparam CSR_STATE_READ_SR               = 29;
	localparam CSR_STATE_CHECK_SR              = 30;
	
	
	// States for writing to a register
	localparam WR_STATE_SET_REG                = 31;
	localparam WR_STATE_W_ACKS                 = 32;
	localparam WR_STATE_DIS_WB                 = 33;
	// States for reading from a register9;
	localparam RD_STATE_SET_REG                = 34;
	localparam RD_STATE_W_ACKS                 = 35;
	localparam RD_STATE_READ_DATA              = 36;


	// 
	//  Module body
	// 


	// Handle global reset signal generation
	GSR GSR_INST (.GSR (rstn));


	// Generate module master clock
	clock CLK (clk);
	// Connect the I2C module to this top module
	I2C_EMBEDDED_WB i2c_top(
		.wb_clk_i(clk),   //  Positive edge clock, >7.5x I2C rate
		.wb_rst_i(rstn),  //  Active-high, synchronous reset signal that will only reset the WISHBONE interface logic.
		.wb_cyc_i(cyc),   //  Active high start of bus cycle
		.wb_stb_i(stb),   //  Active high strobe, WISHBONE slave is the target for current transaction
		.wb_we_i(we),     //  Read/Write control, 1=Write, 0=Read
		.wb_adr_i(adr),   //  8-bit address of EFB register
		.wb_dat_i(datTX), //  Transmitted data byte TO EFB
		.wb_dat_o(datRX), //  Received data byte from EFB
		.wb_ack_o(ack),   //  Active-high transfer ack signal from EFB module, indicates that requested transfer is ack'd
		.i2c1_scl(scl),   //  I2C clock inout
		.i2c1_sda(sda)   //  I2C Data  inout
		);
	/*
	
	Bypassing this block for now. It wasn't working like I expected.
	
	// Generate I2C interval clk for 400kHz I2C clock rate down from 53.20MHz clock
	// Also generates a 1us clock timer
	always@(posedge clk, negedge rstn)//, negedge clear_wait_1us_timer)
		begin
			if(~rstn)
				begin
					count <= 0;
					I2C_clk <= 0;
					wait_1us_timer <=0;
				end
			else
				begin
					if (~clear_wait_1us_timer)
						begin
							wait_1us_timer <= 0;
						end
					else
						begin
							count <= count + 1;
							if(count > I2C_DIVIDER)
								begin
									count <= 0;
									I2C_clk <= ~I2C_clk;
								end
							if(count >= WAIT_1US_DIVIDER)
								begin
									wait_1us_timer <= 1;
								end
						end
				end
		end
	*/
	
	//Generate I2C interval clk for 400kHz I2C clock rate down from 53.20MHz clock
	always@(posedge clk, negedge rstn) begin
		if(~rstn) begin
			count <= 0;
			I2C_clk <= 0;
		end
		else begin
			count <= count + 1;
			if(count > I2C_DIVIDER) begin
				count <= 0;
				I2C_clk <= ~I2C_clk;
				end
			end
		end

	// Latch the Write/Read/CheckStatusRegister next states on clock negative edge
	always@(negedge clk, negedge rstn)
		begin
			if(~rstn)
				begin
					CSR_CallerNextStateBuffer <= CSRcaller_next_state;
					WR_CallerNextStateBuffer  <= WRcaller_next_state;
					RD_CallerNextStateBuffer  <= RDcaller_next_state;
				end
			else
				begin
					if(CSRcaller_next_state)
						CSR_CallerNextStateBuffer <= CSRcaller_next_state;
					if(WRcaller_next_state)
						WR_CallerNextStateBuffer  <= WRcaller_next_state;
					if(RDcaller_next_state)
						RD_CallerNextStateBuffer  <= RDcaller_next_state;
				end
		end	
	
	// Advance state at each positive clock edge
	always@(posedge I2C_clk, negedge rstn)
		begin
			if(~rstn)
				begin
					I2C_CMD_State <= I2C_CMD_STATE_RESET;
				end
			else
				begin
					I2C_CMD_State <= I2C_CMD_NextState;
				end
		end

	// Take write register arguments and assign to relevant variables,
	// allows single line "calls" to reg write FSM section	
	task wb_write_reg;
		input [awidth-1:0] TASKWR_cmd_addr;
		input [dwidth-1:0] TASKWR_cmd_data;
		input [stateBits:0] WR_CallerNextState;
		begin
			cmd_addr =  TASKWR_cmd_addr;
			cmd_data =  TASKWR_cmd_data;
			WRcaller_next_state = WR_CallerNextState;
			I2C_CMD_NextState =  WR_STATE_SET_REG;
		end
	endtask
	
	// Take read register arguments and assign to relevant variables,
	// allows single line "calls" to reg read FSM section	
	task wb_read_reg;
		input  [awidth-1:0]  TASKRD_cmd_addr;
		input  RD_ReadType;
		input  [stateBits:0] RD_CallerNextState;
		begin
			cmd_addr =  TASKRD_cmd_addr;
			ReadType =  RD_ReadType;
			RDcaller_next_state = RD_CallerNextState;
			I2C_CMD_NextState = RD_STATE_SET_REG;
		end
	endtask

	// Take Check Status Register arguments and assign to relevant variables,
	// allows single line "calls" to Check Status Register (CSR) FSM section	
	task wb_check_SR;
		input [stateBits:0] CSR_CallerNextState;
		begin
			CSRcaller_next_state = CSR_CallerNextState;
			I2C_CMD_NextState = CSR_STATE_READ_SR;
		end
	endtask

	// Determine next state and drive module outputs
	always@(I2C_CMD_State, ack)
		begin
			case(I2C_CMD_State)
				I2C_CMD_STATE_RESET:
					begin
						READ_CMD  = 0;
						WRITE_CMD = 0;
						we       = I2C_1_WE_READ;
						cyc      = STOP;
						stb      = STOP;
						clear_wait_1us_timer = 1;
						if(~rstn)
							I2C_CMD_NextState =  I2C_CMD_STATE_RESET;
						else
							I2C_CMD_NextState =  I2C_CMD_STATE_INIT_WAIT1;
					end
				I2C_CMD_STATE_INIT_WAIT1: //  Start 1us bootup delay
					begin
						READ_CMD  = 0;
						WRITE_CMD = 0;
						we       = I2C_1_WE_READ;
						cyc      = STOP;
						stb      = STOP;
						clear_wait_1us_timer = 0;
						I2C_CMD_NextState =  I2C_CMD_STATE_INIT_WAIT2;
					end
				I2C_CMD_STATE_INIT_WAIT2: //  Exit 1us bootup delay
					begin
						READ_CMD  = 0;
						WRITE_CMD = 0;
						we       = I2C_1_WE_READ;
						cyc      = STOP;
						stb      = STOP;
						//if(wait_1us_timer == 1)
						//	begin
								I2C_CMD_NextState =  I2C_CMD_STATE_INIT_ENA;
						//		clear_wait_1us_timer = 1;

						//	end
						//else
						//	I2C_CMD_NextState =  I2C_CMD_STATE_INIT_WAIT2;
					end
				I2C_CMD_STATE_INIT_ENA: //   enable I2C core
					begin
						wb_write_reg(
									I2C_1_CR,
									{I2C_1_CR_I2CEN, I2C_1_CR_SDA_DEL_SEL_300NS},
									I2C_CMD_STATE_WAIT);
					end
				I2C_CMD_STATE_WAIT:
					begin
					
						// Just for testing, force to read, get rid of this soon!
						READ_CMD = 1;
						// / End just for testing code
						
						if(WRITE_CMD == 1'b1)
							I2C_CMD_NextState =  I2C_CMD_STATE_W_SET_SLAVE;
						else if(READ_CMD == 1'b1)
							I2C_CMD_NextState =  I2C_CMD_STATE_R_SET_SLAVE_WRITE;
						else //  Neither read nor write, just wait more
							I2C_CMD_NextState =  I2C_CMD_STATE_WAIT;
					end

				// Start of WRITE sequence
				I2C_CMD_STATE_W_SET_SLAVE: //   present slave address, set write-bit
					begin
						wb_write_reg(I2C_1_TXDR,
									{SLAVE_ADDRESS,WR_BIT},
									I2C_CMD_STATE_W_SET_WRITE);
					end
				I2C_CMD_STATE_W_SET_WRITE: //   present slave address, set write-bit
					begin
						wb_write_reg(I2C_1_CMDR,
									(I2C_1_CMDR_STA | I2C_1_CMDR_ACK |I2C_1_CMDR_WR),
									I2C_CMD_STATE_W_READ_CH_SR1);
					end
				I2C_CMD_STATE_W_READ_CH_SR1: //   Check TRRDY bit (Transfer Ready)
					begin
						wb_check_SR(I2C_CMD_STATE_W_SET_S_REG);                            
					end
				I2C_CMD_STATE_W_SET_S_REG: //   Slave's register/memory to access - NEED TO SET THIS to something
					begin
						wb_write_reg(I2C_1_TXDR,
									8'h00, 																//This is a trash value for now, change to something good
									I2C_CMD_STATE_W_WRITE_REG); 
					end
				I2C_CMD_STATE_W_WRITE_REG: //   Write the register/memory value
					begin
						wb_write_reg(I2C_1_CMDR,
									(I2C_1_CMDR_ACK |I2C_1_CMDR_WR),
									I2C_CMD_STATE_W_READ_CH_SR2); 
					end
				I2C_CMD_STATE_W_READ_CH_SR2: //   Check TRRDY bit (Transfer Ready)
					begin
						wb_check_SR(I2C_CMD_STATE_W_SET_REG_VAL);                          
					end
				I2C_CMD_STATE_W_SET_REG_VAL:
					begin
						wb_write_reg(I2C_1_TXDR,
									DATA_IN,
									I2C_CMD_STATE_W_WRITE_REG_VAL);   
					end
				I2C_CMD_STATE_W_WRITE_REG_VAL:
					begin
						wb_write_reg(I2C_1_CMDR,
									(I2C_1_CMDR_WR | I2C_1_CMDR_ACK),
									I2C_CMD_STATE_W_READ_CH_SR3);   
					end
				I2C_CMD_STATE_W_READ_CH_SR3: //   Check TRRDY bit (Transfer Ready)
					begin
						wb_check_SR(I2C_CMD_STATE_W_WRITE_STOP);                          
					end
				I2C_CMD_STATE_W_WRITE_STOP:
					begin
						wb_write_reg(I2C_1_CMDR,
									(I2C_1_CMDR_STO |I2C_1_CMDR_ACK | I2C_1_CMDR_CKDIS), //Stop transfer, with ack and clk stretching,
									I2C_CMD_STATE_W_READ_CH_SR4);   
					end
				I2C_CMD_STATE_W_READ_CH_SR4: //   Check TRRDY bit (Transfer Ready)
					begin
						wb_check_SR(I2C_CMD_STATE_WAIT);                           
						if(check_SR == 1'b0) begin
							WRITE_CMD = 1'b0;
							READ_CMD  = 1'b0;
						end
					end
	
				// Start of READ sequence
				I2C_CMD_STATE_R_SET_SLAVE_WRITE: //   present slave address, set write-bit
					begin
						wb_write_reg(I2C_1_TXDR,
									{SLAVE_ADDRESS,WR_BIT},
									I2C_CMD_STATE_R_SET_WRITE1);
					end
				I2C_CMD_STATE_R_SET_WRITE1: //   present slave address, set write-bit
					begin
						wb_write_reg(I2C_1_CMDR,
									(I2C_1_CMDR_STA | I2C_1_CMDR_ACK |I2C_1_CMDR_WR),
									I2C_CMD_STATE_R_READ_CH_SR1);
					end
				I2C_CMD_STATE_R_READ_CH_SR1: //   Check TRRDY bit (Transfer Ready)
					begin
						wb_check_SR(I2C_CMD_STATE_R_SET_S_REG);                            
					end
				I2C_CMD_STATE_R_SET_S_REG: //   Slave's register/memory to access - NEED TO SET THIS to something
					begin
						wb_write_reg(I2C_1_TXDR,
									8'h00, 																//This is a trash value for now, change to something good
									I2C_CMD_STATE_R_WRITE_REG); 
					end
				I2C_CMD_STATE_R_WRITE_REG: //   Write the register/memory value
					begin
						wb_write_reg(I2C_1_CMDR,
									(I2C_1_CMDR_ACK |I2C_1_CMDR_WR),
									I2C_CMD_STATE_R_READ_CH_SR2); 
					end
				I2C_CMD_STATE_R_READ_CH_SR2: //   Check TRRDY bit (Transfer Ready)
					begin
						wb_check_SR(I2C_CMD_STATE_R_SET_SLAVE_READ);                          
					end
					
				I2C_CMD_STATE_R_SET_SLAVE_READ: // Set slave address with read bit
					begin
						wb_write_reg(I2C_1_TXDR,
									{SLAVE_ADDRESS,RD_BIT},
									I2C_CMD_STATE_R_SET_WRITE2);   
					end
				I2C_CMD_STATE_R_SET_WRITE2: // Write slave address
					begin
						wb_write_reg(I2C_1_CMDR,
									(I2C_1_CMDR_STA | I2C_1_CMDR_ACK |I2C_1_CMDR_WR),
									I2C_CMD_STATE_R_WAIT_SRW1);   
					end
				I2C_CMD_STATE_R_WAIT_SRW1: //   Check SRW bit (Slave Read/Write, 1 = Master read mode)
					begin
						wb_read_reg(I2C_1_SR,
									CSR_READ,
									I2C_CMD_STATE_R_WAIT_SRW2);                       
					end
				I2C_CMD_STATE_R_WAIT_SRW2: //   Check SRW bit (Slave Read/Write, 1 = Master read mode)
					begin
						if(check_SR[I2C_1_SR_SRW] == 1) //The master is configured in read mode
							I2C_CMD_NextState =  I2C_CMD_STATE_R_SET_READ;
						else
							I2C_CMD_NextState =  I2C_CMD_STATE_R_WAIT_SRW2;
					end
				I2C_CMD_STATE_R_SET_READ:
					begin
						wb_write_reg(I2C_1_CMDR,
									(I2C_1_CMDR_RD | I2C_1_CMDR_ACK),
									I2C_CMD_STATE_R_CMD_READ_STOP);   
					end
				I2C_CMD_STATE_R_CMD_READ_STOP:
					begin
						wb_write_reg(I2C_1_CMDR,
									(I2C_1_CMDR_STO | I2C_1_CMDR_RD | I2C_1_CMDR_ACK | I2C_1_CMDR_CKDIS), //Stop read transfer, with ack and clk stretching,
									I2C_CMD_STATE_R_READ_CH_SR3);   
					end
				I2C_CMD_STATE_R_READ_CH_SR3: //   Check TRRDY bit (Transfer Ready)
					begin
						wb_check_SR(I2C_CMD_STATE_WAIT);                           
						if(check_SR == 1'b0) begin
							WRITE_CMD = 1'b0;
							READ_CMD  = 1'b0;
						end
					end
					
					
				// FSM portion for writing to a register
				WR_STATE_SET_REG:
					begin
						we    = I2C_1_WE_WRITE;
						cyc   = START;
						stb   = START;
						adr   = cmd_addr;
						datTX = cmd_data;
						I2C_CMD_NextState =  WR_STATE_W_ACKS;
					end
				WR_STATE_W_ACKS:
					begin
						we    = I2C_1_WE_WRITE;
						cyc   = START;
						stb   = START;
						adr   = cmd_addr;
						datTX = cmd_data;
						if(ack) //  Wait for ACK from slave
							I2C_CMD_NextState =  WR_STATE_DIS_WB;
						else
							I2C_CMD_NextState =  WR_STATE_W_ACKS;
					end
				WR_STATE_DIS_WB:
					begin
						we    = I2C_1_WE_READ;
						cyc   = STOP;
						stb   = STOP;
						if(ack) //Ack should go away once stb and cyc are de-asserted
							I2C_CMD_NextState =  WR_STATE_DIS_WB;
						else    //OK, good, move on
							begin
								adr   = {awidth{1'bx}};
								datTX = {dwidth{1'bx}};
								I2C_CMD_NextState =  WR_CallerNextStateBuffer;
							end
					end
				
				
				
				// FSM portion for reading from a register
				RD_STATE_SET_REG:
					begin
						we    = I2C_1_WE_READ;
						cyc   = START;
						stb   = START;
						adr   = cmd_addr;
						datTX = {dwidth{1'bx}};
						I2C_CMD_NextState =  RD_STATE_W_ACKS;
					end
				RD_STATE_W_ACKS:
					begin
						we    = I2C_1_WE_READ;
						cyc   = START;
						stb   = START;
						adr   = cmd_addr;
						datTX = {dwidth{1'bx}};
						if(ack)
							I2C_CMD_NextState =  RD_STATE_READ_DATA;
						else
							I2C_CMD_NextState =  RD_STATE_W_ACKS;
					end
				RD_STATE_READ_DATA:
					begin
						we    = I2C_1_WE_READ;
						cyc   = STOP;
						stb   = STOP;
						adr   = {awidth{1'bx}};
						if(ack) //Ack should go away once stb and cyc are de-asserted
							I2C_CMD_NextState =  RD_STATE_READ_DATA;
						else    //OK, good, move on
							begin
								if(ReadType == DATA_READ)
									DATA_OUT =  datRX;
								else //  ReadType == CSR_READ
									check_SR =  datRX;
								datTX= {dwidth{1'bx}};
								I2C_CMD_NextState =  RD_CallerNextStateBuffer;
							end
					end
	
	
	
				// FSM portion for checking status register
				CSR_STATE_READ_SR:
					begin
						check_SR =  1'b1;    //  Force to one to begin, will be set to 0 when done.
						wb_read_reg(I2C_1_SR,
									CSR_READ,
									CSR_STATE_CHECK_SR);
					end
				CSR_STATE_CHECK_SR:         //  Transmit/Receive ready
					begin
						if(check_SR[I2C_1_SR_TRRDY] == 1) 
							I2C_CMD_NextState =  CSR_CallerNextStateBuffer;
						else
							I2C_CMD_NextState =  CSR_STATE_READ_SR;
					end


				default: // Default case, shouldn't be triggered	
					begin
						I2C_CMD_NextState =  I2C_CMD_STATE_RESET;
					end
			endcase
		end
endmodule