/////////////////////////////////////////////////////////////////////
////                                                             ////
////  WISHBONE rev.B2 compliant I2C Master controller Testbench  ////
////                                                             ////
////                                                             ////
////  Author: Richard Herveille                                  ////
////          richard@asics.ws                                   ////
////          www.asics.ws                                       ////
////                                                             ////
////  Downloaded from: http://www.opencores.org/projects/i2c/    ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
////                                                             ////
//// Copyright (C) 2001 Richard Herveille                        ////
////                    richard@asics.ws                         ////
////                                                             ////
//// This source file may be used and distributed without        ////
//// restriction provided that this copyright statement is not   ////
//// removed from the file and that any derivative work contains ////
//// the original copyright notice and the associated disclaimer.////
////                                                             ////
////     THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY     ////
//// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   ////
//// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   ////
//// FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR      ////
//// OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,    ////
//// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ////
//// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ////
//// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ////
//// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF  ////
//// LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT  ////
//// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  ////
//// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         ////
//// POSSIBILITY OF SUCH DAMAGE.                                 ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
//
// Change History:
//
//               Revision 1.7  2005/02/27 09:24:18  rherveille
//               Fixed scl, sda delay.
//
//               Revision 1.6  2004/02/28 15:40:42  rherveille
//               *** empty log message ***
//
//               Revision 1.4  2003/12/05 11:04:38  rherveille
//               Added slave address configurability
//
//               Revision 1.3  2002/10/30 18:11:06  rherveille
//               Added timing tests to i2c_model.
//               Updated testbench.
//
//               Revision 1.2  2002/03/17 10:26:38  rherveille
//               Fixed some race conditions in the i2c-slave model.
//               Added debug information.
//               Added headers.
//
// Name:  tst_bench_top.v   
// 
// Description: This module is the top-level testbench, which controls the 
// flow of the wb v.s. i2c transmit and receive procedures. 
//   
//-------------------------------------------------------------------------
// LSC Code Revision History :
//-------------------------------------------------------------------------
// Ver: | Author	|Mod. Date	|Changes Made:
// V2.0 | cm		|12/2008        |rstn initialized to 0
//      |               		|use one i2c master for simulation
//      |                               |use 50MHz clock, prescaler to 100(h64)
//                                      |connect scl and sda instead of oe, i, o
//                                      |read memory within defined memory space (up to 03)
//	                                |replace q, qq with check_SR, data_rxr
//-------------------------------------------------------------------------

`include "timescale.v"

module i2c_module(scl, sda, rstn, DATA_OUT, DATA_IN);//, clk);
	//
	// wires && regs
	//
	inout scl ;
	inout sda ;
	input wire rstn; // async reset, not from wb
	output reg [7:0] DATA_OUT;
	input wire [7:0] DATA_IN;
	//wire clk;  // master clock from wb
	//input wire clk;  // master clock from wb
	reg [31:0] adr;
	reg  [7:0] datTX;
	wire [7:0] datRX;
	reg we;
	reg stb;
	reg cyc;
	wire ack;
	wire inta;
	
	localparam awidth = 4'h7;
	localparam dwidth = 4'h8;
	localparam stateBits = 6;
	
	reg READ_CMD;
	reg WRITE_CMD;
	reg delay;
	reg [awidth -1:0]	cmd_addr;
	reg [dwidth -1:0]	cmd_data;
	reg [stateBits:0]CSRcaller_next_state, WRcaller_next_state, RDcaller_next_state;
	reg [7:0]check_SR, data_rxr;
	reg [stateBits:0]I2C_CMD_State, I2C_CMD_NextState;
	reg I2C_clk;
	reg [7:0]count;
	wire scl_pad_i;
	wire scl_pad_o;
	wire scl_padoen_o;
	wire sda_pad_i;
	wire sda_pad_o;
	wire sda_padoen_o;
	
	localparam BNO055_CHIP_ID_REG = 8'h00;
	
	localparam I2C_DIVIDER = 67; // Divider to provide a clock posedge every 2.5 uS 1/(400kHz)
	
	localparam PRER_LO = 3'b000;
	localparam PRER_HI = 3'b001;
	localparam CTR     = 3'b010;
	localparam RXR     = 3'b011;
	localparam TXR     = 3'b011;
	localparam CR      = 3'b100;
	localparam SR      = 3'b100;
	localparam TXR_R   = 3'b101; // undocumented / reserved output
	localparam CR_R    = 3'b110; // undocumented / reserved output
	localparam RD      = 1'b1;
	localparam WR      = 1'b0;
	localparam SADR    = 7'b01010_01; //Slave address of the BNO055 = 0x29

	//Initial default state
	localparam I2C_CMD_STATE_RESET            = 0;
	localparam I2C_CMD_STATE_INIT_CLK1        = 1;
	localparam I2C_CMD_STATE_INIT_CLK2        = 2;
	localparam I2C_CMD_STATE_INIT_ENA         = 3;
	localparam I2C_CMD_STATE_WAIT             = 4;
	//Write states
	localparam I2C_CMD_STATE_W_SET_SLAVE      = 5;
	localparam I2C_CMD_STATE_W_SET_START      = 6;
	localparam I2C_CMD_STATE_W_READ_CH_SR1    = 7;
	localparam I2C_CMD_STATE_W_SET_S_REG      = 8;
    localparam I2C_CMD_STATE_W_CMD_WRITE1     = 9;
	localparam I2C_CMD_STATE_W_READ_CH_SR2    = 10;
	localparam I2C_CMD_STATE_W_SET_TX_REG     = 11;
    localparam I2C_CMD_STATE_W_CMD_WRITE2     = 12;
	localparam I2C_CMD_STATE_W_SET_STOP       = 13;
	localparam I2C_CMD_STATE_W_READ_CH_SR3    = 14;
	//Read states
	localparam I2C_CMD_STATE_R_SET_SLAVE      = 15;
	localparam I2C_CMD_STATE_R_SET_START      = 16;
	localparam I2C_CMD_STATE_R_READ_CH_SR1    = 17;
	localparam I2C_CMD_STATE_R_SET_S_REG      = 18;
    localparam I2C_CMD_STATE_R_CMD_WRITE1     = 19;
	localparam I2C_CMD_STATE_R_READ_CH_SR2    = 20;
	localparam I2C_CMD_STATE_R_SET_SLAVE_RD   = 21;
    localparam I2C_CMD_STATE_R_CMD_READ_START = 22;
	localparam I2C_CMD_STATE_R_READ_CH_SR3    = 23;
    localparam I2C_CMD_STATE_R_CMD_READ_ACKM  = 24;
	localparam I2C_CMD_STATE_R_READ_CH_SR4    = 25;
	localparam I2C_CMD_STATE_R_READ_STR_RX    = 26;
	localparam I2C_CMD_STATE_R_SET_STOP       = 27;
	localparam I2C_CMD_STATE_R_READ_CH_SR5    = 28;

	//States for checking status register
	localparam CSR_STATE_READ_SR             = 29;
	localparam CSR_STATE_CHECK_SR            = 30;

	//States for writing to a register
	localparam WR_STATE_DELAY1a              = 31;
	localparam WR_STATE_DELAY1b              = 32;
	localparam WR_STATE_SET_REG              = 33;
	localparam WR_STATE_W_ACKS               = 34;
    localparam WR_STATE_DELAY2a              = 35;
    localparam WR_STATE_DELAY2b              = 36;
	localparam WR_STATE_DIS_WB               = 37;
	
	//States for reading from a register
	localparam RD_STATE_DELAY1a              = 38;
	localparam RD_STATE_DELAY1b              = 39;
	localparam RD_STATE_SET_REG              = 40;
	localparam RD_STATE_W_ACKS               = 41;
	localparam RD_STATE_DELAY2a              = 42;
    localparam RD_STATE_DELAY2b              = 43;
	localparam RD_STATE_READ                 = 44;

	//
	// Module body
	//


	clock CLK (clk);
	i2c_master i2c_top (
		// wishbone interface
		.wb_clk_i(clk),
		.wb_rst_i(1'b0),
		.arst_i(rstn),
		.wb_adr_i(adr[2:0]),
		.wb_dat_i(datTX),
		.wb_dat_o(datRX),
		.wb_we_i(we),
		.wb_stb_i(stb),
		.wb_cyc_i(cyc),
		.wb_ack_o(ack),
		.wb_inta_o(inta),
		.scl_pad_i(scl_pad_i),
		.scl_pad_o(scl_pad_o),
		.scl_padoen_o(scl_padoen_o),
		.sda_pad_i(sda_pad_i),
		.sda_pad_o(sda_pad_o),
		.sda_padoen_o(sda_padoen_o)
	);

	
	assign scl = (scl_padoen_o == 1'b1) ? 1'bz: scl_pad_o;
	assign sda = (sda_padoen_o == 1'b1) ? 1'bz: sda_pad_o;
	assign scl_pad_i = scl;
	assign sda_pad_i = sda;
	
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

	always@(posedge I2C_clk, negedge rstn) begin
		if(~rstn) begin
			I2C_CMD_State <= I2C_CMD_STATE_RESET;
			end
		else begin
			I2C_CMD_State <= I2C_CMD_NextState;
			end
	end

		
	task wb_write_reg;
		input TASKWR_delay;
		input [awidth-1:0] TASKWR_cmd_addr;
		input [dwidth-1:0] TASKWR_cmd_data;
		input [stateBits:0] WR_CallerNextState;
		output [stateBits:0] WRcaller_next_state;
		begin
			delay= TASKWR_delay;
			cmd_addr= TASKWR_cmd_addr;
			cmd_data= TASKWR_cmd_data;
			I2C_CMD_NextState= WR_STATE_DELAY1a;
			WRcaller_next_state= WR_CallerNextState;
		end
	endtask
	
	
	task wb_read_reg;
		input TASKRD_delay;
		input [awidth-1:0] TASKRD_cmd_addr;
		output [dwidth-1:0] TASKRD_cmd_data;
		input [stateBits:0] RD_CallerNextState;
		output [stateBits:0] RDcaller_next_state;
		begin
			delay= TASKRD_delay;
			cmd_addr= TASKRD_cmd_addr;
			cmd_data= TASKRD_cmd_data;
			I2C_CMD_NextState= RD_STATE_DELAY1a;
			RDcaller_next_state= RD_CallerNextState;
		end
	endtask
	
	task wb_check_SR;
		input [stateBits:0] CSR_CallerNextState;
		output [stateBits:0] CSRcaller_next_state;
		begin
			I2C_CMD_NextState= CSR_STATE_READ_SR;
			CSRcaller_next_state= CSR_CallerNextState;
		end
	endtask

	//always@(negedge clk, posedge rstn, posedge ack) begin
	always@(I2C_CMD_State, WRcaller_next_state, RDcaller_next_state, CSRcaller_next_state, ack) begin
		//Defaults to maintain state, may be overridden in lower steps
		cyc  = cyc;
		stb  = stb;
		we   = we;
		WRcaller_next_state = WRcaller_next_state;
		RDcaller_next_state = RDcaller_next_state;
		CSRcaller_next_state= CSRcaller_next_state;
		case(I2C_CMD_State)
			I2C_CMD_STATE_RESET: begin
				READ_CMD = 0;
				WRITE_CMD= 0;
				we       = 0;
				cyc      = 0;
				stb      = 0;
				if(~rstn)
					I2C_CMD_NextState= I2C_CMD_STATE_RESET;
				else
					I2C_CMD_NextState= I2C_CMD_STATE_INIT_CLK1;
				end
			I2C_CMD_STATE_INIT_CLK1: begin
				wb_write_reg(1, PRER_LO, 8'h64, I2C_CMD_STATE_INIT_CLK2, WRcaller_next_state); // load prescaler lo-byte
				READ_CMD = 0;
				WRITE_CMD= 0;
				we       = 0;
				cyc      = 0;
				stb      = 0;
				end
			I2C_CMD_STATE_INIT_CLK2:
				wb_write_reg(1, PRER_HI, 8'h00, I2C_CMD_STATE_INIT_ENA, WRcaller_next_state); // load prescaler hi-byte
			I2C_CMD_STATE_INIT_ENA:
				wb_write_reg(1, CTR, 8'h80, I2C_CMD_STATE_WAIT, WRcaller_next_state);         // enable core
			I2C_CMD_STATE_WAIT: begin
				//Just for testing, force to read, get rid of this soon!
				READ_CMD         = 1;
				if(WRITE_CMD == 1'b1)
					I2C_CMD_NextState= I2C_CMD_STATE_W_SET_SLAVE;
				else if(READ_CMD == 1'b1)
					I2C_CMD_NextState= I2C_CMD_STATE_R_SET_SLAVE;
				else
					I2C_CMD_NextState= I2C_CMD_STATE_WAIT;
				end
				
			//Start of WRITE sequence
			I2C_CMD_STATE_W_SET_SLAVE:
				wb_write_reg(1, TXR, {SADR,WR}, I2C_CMD_STATE_W_SET_START, WRcaller_next_state);// present slave address, set write-bit
			I2C_CMD_STATE_W_SET_START:
				wb_write_reg(0, CR, 8'h90, I2C_CMD_STATE_W_READ_CH_SR1, WRcaller_next_state);   // set command (start, write)
			I2C_CMD_STATE_W_READ_CH_SR1: 
				wb_check_SR(I2C_CMD_STATE_W_SET_S_REG, CSRcaller_next_state);                    // check tip bit (Transfer In Progress)
			I2C_CMD_STATE_W_SET_S_REG:
				wb_write_reg(1, TXR, BNO055_CHIP_ID_REG, I2C_CMD_STATE_W_CMD_WRITE1, WRcaller_next_state); // present slave's memory address - NEED TO SET THIS to something
			I2C_CMD_STATE_W_CMD_WRITE1:
				wb_write_reg(0, CR, 8'h10, I2C_CMD_STATE_W_READ_CH_SR2, WRcaller_next_state);   // set command (write)
			I2C_CMD_STATE_W_READ_CH_SR2: 
				wb_check_SR(I2C_CMD_STATE_W_SET_TX_REG, CSRcaller_next_state);                   // check tip bit (Transfer In Progress)
			I2C_CMD_STATE_W_SET_TX_REG:
				wb_write_reg(1, TXR, 8'ha5, I2C_CMD_STATE_W_CMD_WRITE2, WRcaller_next_state);   // present data - NEED TO SET THIS to something
			I2C_CMD_STATE_W_CMD_WRITE2:
				wb_write_reg(0, CR, 8'h10, I2C_CMD_STATE_W_SET_STOP, WRcaller_next_state);      // set command (write)
			I2C_CMD_STATE_W_SET_STOP:
				wb_write_reg(0, CR, 8'h50, I2C_CMD_STATE_W_READ_CH_SR3, WRcaller_next_state);   // set command (stop, write)
			I2C_CMD_STATE_W_READ_CH_SR3: begin
				wb_check_SR(I2C_CMD_STATE_WAIT, CSRcaller_next_state);                           // check tip bit (Transfer In Progress)
				if(check_SR == 1'b0) begin
					WRITE_CMD= 1'b0;
					READ_CMD = 1'b0;
					end
				end
				
			//Start of READ sequence
			I2C_CMD_STATE_R_SET_SLAVE:
				wb_write_reg(1, TXR,{SADR,WR}, I2C_CMD_STATE_R_SET_START, WRcaller_next_state); // present slave address, set write-bit
			I2C_CMD_STATE_R_SET_START:
				wb_write_reg(0, CR, 8'h90, I2C_CMD_STATE_R_READ_CH_SR1, WRcaller_next_state);   // set command (start, write)
			I2C_CMD_STATE_R_READ_CH_SR1: 
				wb_check_SR(I2C_CMD_STATE_R_SET_S_REG, CSRcaller_next_state);                    // check tip bit (Transfer In Progress)	
			I2C_CMD_STATE_R_SET_S_REG:
				wb_write_reg(1, TXR, 8'h00, I2C_CMD_STATE_R_CMD_WRITE1, WRcaller_next_state);   // present slave's memory address
			I2C_CMD_STATE_R_CMD_WRITE1:
				wb_write_reg(0, CR, 8'h10, I2C_CMD_STATE_R_READ_CH_SR2, WRcaller_next_state);   // set command (write)
			I2C_CMD_STATE_R_READ_CH_SR2: 
				wb_check_SR(I2C_CMD_STATE_R_SET_SLAVE_RD, CSRcaller_next_state);                 // check tip bit (Transfer In Progress)
			I2C_CMD_STATE_R_SET_SLAVE_RD:
				wb_write_reg(1, TXR, {SADR,RD}, I2C_CMD_STATE_R_CMD_READ_START, WRcaller_next_state);// present slave's address, set read-bit
			I2C_CMD_STATE_R_CMD_READ_START:
				wb_write_reg(0, CR, 8'h90, I2C_CMD_STATE_R_READ_CH_SR3, WRcaller_next_state);   // set command (start, write)
			I2C_CMD_STATE_R_READ_CH_SR3: 
				wb_check_SR(I2C_CMD_STATE_R_CMD_READ_ACKM, CSRcaller_next_state);                // check tip bit (Transfer In Progress)
			I2C_CMD_STATE_R_CMD_READ_ACKM:
				wb_write_reg(1, CR, 8'h20, I2C_CMD_STATE_R_READ_CH_SR4, WRcaller_next_state);   // set command (read, ack_read)
			I2C_CMD_STATE_R_READ_CH_SR4: 
				wb_check_SR(I2C_CMD_STATE_R_READ_STR_RX, CSRcaller_next_state);                  // check tip bit (Transfer In Progress)
			I2C_CMD_STATE_R_READ_STR_RX:
				wb_read_reg(1, RXR, DATA_OUT, I2C_CMD_STATE_R_SET_STOP, RDcaller_next_state);   //Store read data
			I2C_CMD_STATE_R_SET_STOP:
				wb_write_reg(1, CR, 8'h40, I2C_CMD_STATE_R_READ_CH_SR5, WRcaller_next_state);   // set command (stop)
			I2C_CMD_STATE_R_READ_CH_SR5: begin
				wb_check_SR(I2C_CMD_STATE_WAIT, CSRcaller_next_state);                                // check tip bit (Transfer In Progress)
				if(check_SR == 1'b0) begin
					WRITE_CMD= 1'b0;
					READ_CMD = 1'b0;
					end
				end
				
				
			//FSM portion for writing to a register
			WR_STATE_DELAY1a: begin
				we    = 1'b0;
				cyc   = 1'b0;
				stb   = 1'b0;
				if(delay) //If a next delay (Can be either 1 or 0), this delay is mandatory
					I2C_CMD_NextState= WR_STATE_DELAY1b;
				else
					I2C_CMD_NextState= WR_STATE_SET_REG;
				end
			WR_STATE_DELAY1b: begin
				we   = 1'b0;
				cyc  = 1'b0;
				stb  = 1'b0;
				I2C_CMD_NextState= WR_STATE_SET_REG;
				end
			WR_STATE_SET_REG: begin
				adr  = cmd_addr;
				datTX= cmd_data;
				cyc  = 1'b1;
				stb  = 1'b1;
				we   = 1'b1;
				I2C_CMD_NextState= WR_STATE_W_ACKS;
				end
			WR_STATE_W_ACKS: begin
				we   = 1'b1;
				cyc  = 1'b1;
				stb  = 1'b1;
				if(ack)
					I2C_CMD_NextState= WR_STATE_DELAY2a;
				else
					I2C_CMD_NextState= WR_STATE_W_ACKS;
				end
			WR_STATE_DELAY2a: begin
				we   = 1'b1;
				cyc  = 1'b1;
				stb  = 1'b1;
				I2C_CMD_NextState= WR_STATE_DELAY2b;
				end
			WR_STATE_DELAY2b: begin
				we   = 1'b1;
				cyc  = 1'b1;
				stb  = 1'b1;
				I2C_CMD_NextState= WR_STATE_DIS_WB;
				end
			WR_STATE_DIS_WB: begin
				cyc  = 1'b0;
				stb  = 1'b0;
				adr  = {awidth{1'bx}};
				datTX= {dwidth{1'bx}};
				we   = 1'b0;
				I2C_CMD_NextState= WRcaller_next_state;
				end
			
			
			
			//FSM portion for reading from a register
			RD_STATE_DELAY1a: begin
				we   = 1'b0;
				cyc  = 1'b0;
				stb  = 1'b0;
				if(delay) //If a next delay (Can be either 1 or 0), this delay is mandatory
					I2C_CMD_NextState= RD_STATE_DELAY1b;
				else
					I2C_CMD_NextState= RD_STATE_SET_REG;
				end
			RD_STATE_DELAY1b: begin
				we   = 1'b0;
				cyc  = 1'b0;
				stb  = 1'b0;
				I2C_CMD_NextState= RD_STATE_SET_REG;
				end
			RD_STATE_SET_REG: begin
				adr  = cmd_addr;
				datTX= {dwidth{1'bx}};
				cyc  = 1'b1;
				stb  = 1'b1;
				we   = 1'b0;
				I2C_CMD_NextState= RD_STATE_W_ACKS;
				end
			RD_STATE_W_ACKS: begin
				cyc  = 1'b1;
				stb  = 1'b1;
				we   = 1'b0;
				if(ack)
					I2C_CMD_NextState= RD_STATE_DELAY2a;
				else
					I2C_CMD_NextState= RD_STATE_W_ACKS;
				end
			RD_STATE_DELAY2a: begin
				cyc  = 1'b1;
				stb  = 1'b1;
				we   = 1'b0;
				I2C_CMD_NextState= RD_STATE_DELAY2b;
				end
			RD_STATE_DELAY2b: begin
				cyc  = 1'b1;
				stb  = 1'b1;
				we   = 1'b0;
				I2C_CMD_NextState= RD_STATE_READ;
				end
			RD_STATE_READ: begin
				cyc  = 1'b0;
				stb  = 1'b0; //1'bx; cm - change to inactive state
				adr  = {awidth{1'bx}};
				datTX= {dwidth{1'bx}};
				we   = 1'b0; //1'hx; cm - change to inactive state
				cmd_data= datRX;
				I2C_CMD_NextState= RDcaller_next_state;
				end



			//FSM portion for checking status register
			CSR_STATE_READ_SR: begin
				check_SR= 1'b1;
				wb_read_reg(1, SR, check_SR, CSR_STATE_CHECK_SR, RDcaller_next_state);
				end
			CSR_STATE_CHECK_SR: begin
				if(check_SR[1] == 0)
					I2C_CMD_NextState= CSRcaller_next_state;
				else
					I2C_CMD_NextState= CSR_STATE_READ_SR;
				end

			//Default case, shouldn't be triggered	
			default:
				I2C_CMD_NextState= I2C_CMD_STATE_INIT_CLK1;
		endcase
	end

endmodule