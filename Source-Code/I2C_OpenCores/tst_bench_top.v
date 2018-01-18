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
//// OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         ////
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
module tst_bench_top(scl, sda, rstn, DATA_OUT, DATA_IN);
	//
	// wires && regs
	//
	inout scl ;
	inout sda ;
	input wire rstn; // async reset, not from wb
	output reg [7:0] DATA_OUT;
	input wire [7:0] DATA_IN;
	wire clk;  // master clock from wb
	reg [31:0] adr;
	reg  [7:0] dat_o;
	wire [7:0] dat0_i;
	reg we;
	reg stb;
	reg cyc;
	wire ack;
	wire inta;
	reg READ_CMD, WRITE_CMD;
	reg go_next;
	reg block_reading;
	reg block_writing;
	reg [7:0] check_SR, data_rxr;  // give meaningful names to q, and qq, respectively
	parameter PRER_LO = 3'b000;
	parameter PRER_HI = 3'b001;
	parameter CTR     = 3'b010;
	parameter RXR     = 3'b011;
	parameter TXR     = 3'b011;
	parameter CR      = 3'b100;
	parameter SR      = 3'b100;
	parameter TXR_R   = 3'b101; // undocumented / reserved output
	parameter CR_R    = 3'b110; // undocumented / reserved output
	parameter RD      = 1'b1;
	parameter WR      = 1'b0;
	parameter SADR    = 7'b0010_000;
	
	localparam awidth = 4'h7;
	localparam dwidth = 4'h8;
	
	localparam CSR_STATE_READ_SR  = 4'h0;
	localparam CSR_STATE_WAIT     = 4'h1;
	localparam CSR_STATE_CHECK_SR = 4'h2;

	localparam WR_STATE_DELAY1a = 4'h0;
	localparam WR_STATE_DELAY1b = 4'h1;
	localparam WR_STATE_SET_REG = 4'h2;
	localparam WR_STATE_W_ACKS  = 4'h3;
    localparam WR_STATE_DELAY2a = 4'h4;
    localparam WR_STATE_DELAY2b = 4'h5;
	localparam WR_STATE_DIS_WB  = 4'h6;
	
	localparam RD_STATE_DELAY1a = 4'h0;
	localparam RD_STATE_DELAY1b = 4'h1;
	localparam RD_STATE_SET_REG = 4'h2;
	localparam RD_STATE_W_ACKS  = 4'h3;
	localparam RD_STATE_DELAY2a = 4'h4;
    localparam RD_STATE_DELAY2b = 4'h5;
	localparam RD_STATE_READ    = 4'h6;
	
	//Initial default state
	localparam I2C_CMD_STATE_INIT          = 0;
	localparam I2C_CMD_STATE_WAIT          = 1;
	//Write states
	localparam I2C_CMD_STATE_W_SET_SLAVE   = 2;
	localparam I2C_CMD_STATE_W_SET_START   = 3;
	localparam I2C_CMD_STATE_W_READ_CH_SR1 = 4;
	localparam I2C_CMD_STATE_W_SET_S_REG   = 5;
    localparam I2C_CMD_STATE_W_CMD_WRITE1  = 6;
	localparam I2C_CMD_STATE_W_READ_CH_SR2 = 7;
	localparam I2C_CMD_STATE_W_SET_TX_REG  = 8;
    localparam I2C_CMD_STATE_W_CMD_WRITE2  = 9;
	localparam I2C_CMD_STATE_W_SET_STOP    = 10;
	localparam I2C_CMD_STATE_W_READ_CH_SR3 = 11;
	//Read states
	localparam I2C_CMD_STATE_R_SET_SLAVE      = 12;
	localparam I2C_CMD_STATE_R_SET_START      = 13;
	localparam I2C_CMD_STATE_R_READ_CH_SR1    = 14;
	localparam I2C_CMD_STATE_R_SET_S_REG      = 15;
    localparam I2C_CMD_STATE_R_CMD_WRITE1     = 16;
	localparam I2C_CMD_STATE_R_READ_CH_SR2    = 17;
	localparam I2C_CMD_STATE_R_SET_SLAVE_RD   = 18;
    localparam I2C_CMD_STATE_R_CMD_READ_START = 19;
	localparam I2C_CMD_STATE_R_READ_CH_SR3    = 20;
    localparam I2C_CMD_STATE_R_CMD_READ_ACKM  = 21;
	localparam I2C_CMD_STATE_R_READ_CH_SR4    = 22;
	localparam I2C_CMD_STATE_R_READ_STR_RX    = 23;
	localparam I2C_CMD_STATE_R_SET_STOP       = 24;
	localparam I2C_CMD_STATE_R_READ_CH_SR5    = 25;

	reg [3:0]CSRState, CSRNextState;
	reg [3:0]WriteState, WriteNextState;
	reg [3:0]ReadState, ReadNextState;
	reg [5:0]I2C_CMD_State, I2C_CMD_NextState;
	//
	// Module body
	//

	wire stb0 = stb & ~adr[3];
    assign dat_i = ({{8'd8}{stb0}} & dat0_i);

	clock CLK (clk);
	i2c_master_wb_top i2c_top (
		// wishbone interface
		.wb_clk_i(clk),
		.wb_rst_i(1'b0),
		.arst_i(rstn),
		.wb_adr_i(adr[2:0]),
		.wb_dat_i(dat_o),
		.wb_dat_o(dat0_i),
		.wb_we_i(we),
		.wb_stb_i(stb0),
		.wb_cyc_i(cyc),
		.wb_ack_o(ack),
		.wb_inta_o(inta),
    .scl(scl),
    .sda(sda)
	);
	
	task wb_write_reg;
		input   delay;
		integer delay;
		input	[awidth -1:0]	a;
		input	[dwidth -1:0]	d;
		input [5:0]caller_next_state;
		input modify_caller_next_state;
		begin
			case(WriteState)
				WR_STATE_DELAY1a: begin
					block_writing = 1;
					if(delay) //If delay (Either 1 or 0)
						WriteNextState <= WR_STATE_DELAY1b;
					else
						WriteNextState <= WR_STATE_SET_REG;
					end
			WR_STATE_DELAY1b: begin
					WriteNextState <= WR_STATE_SET_REG;
					end
			WR_STATE_SET_REG: begin
					adr   = a;
					dat_o = d;
					cyc   = 1'b1;
					stb   = 1'b1;
					we    = 1'b1;
					WriteNextState <= WR_STATE_W_ACKS;
					end
			WR_STATE_W_ACKS: begin
					if(ack)
						WriteNextState <= WR_STATE_DELAY2a;
					else
						WriteNextState <= WR_STATE_W_ACKS;
					end
			WR_STATE_DELAY2a: begin
					WriteNextState <= WR_STATE_DELAY2b;
					end
			WR_STATE_DELAY2b: begin
					WriteNextState <= WR_STATE_DIS_WB;
					end
			WR_STATE_DIS_WB: begin
					cyc   = 1'b0;
					stb   = 1'b0; //1'bx; cm - change to inactive state
					adr   = {awidth{1'bx}};
					dat_o = {dwidth{1'bx}};
					we   = 1'b0; //1'hx; cm - change to inactive state
					block_writing = 0;
					WriteNextState <= WR_STATE_DELAY1a;
					if(modify_caller_next_state)
						I2C_CMD_NextState <= caller_next_state;
					end
				default: begin //Shouldn't happen
					block_writing = 0;
					WriteNextState <= WR_STATE_DELAY1a;
					end
			endcase
		end
	endtask
	
	task wb_read_reg;
		input   delay;
		integer delay;
		input	 [awidth -1:0]	a;
		output	[dwidth -1:0]	d;
		input [5:0]caller_next_state;
		input modify_caller_next_state;
	
		begin
			case(ReadState)
			RD_STATE_DELAY1a: begin
					block_reading = 1;
					if(delay) //If delay (Either 1 or 0)
						WriteNextState <= RD_STATE_DELAY1b;
					else
						WriteNextState <= RD_STATE_SET_REG;
					end
			RD_STATE_DELAY1b: begin
					WriteNextState <= RD_STATE_SET_REG;
					end
			RD_STATE_SET_REG: begin
					adr   = a;
					dat_o = {dwidth{1'bx}};
					cyc   = 1'b1;
					stb   = 1'b1;
					we    = 1'b0;
					WriteNextState <= RD_STATE_W_ACKS;
					end
			RD_STATE_W_ACKS: begin
					if(ack)
						WriteNextState <= RD_STATE_DELAY2a;
					else
						WriteNextState <= RD_STATE_W_ACKS;
					end
			RD_STATE_DELAY2a: begin
					WriteNextState <= RD_STATE_DELAY2b;
					end
			RD_STATE_DELAY2b: begin
					WriteNextState <= RD_STATE_READ;
					end
			RD_STATE_READ: begin
					cyc   = 1'b0;
					stb   = 1'b0; //1'bx; cm - change to inactive state
					adr   = {awidth{1'bx}};
					dat_o = {dwidth{1'bx}};
					we   = 1'b0; //1'hx; cm - change to inactive state
					d    = dat0_i;
					block_reading = 0;
					if(modify_caller_next_state)
						I2C_CMD_NextState <= caller_next_state;
					WriteNextState <= RD_STATE_DELAY1a;
					end
				default: begin //Shouldn't happen
					block_reading = 0;
					WriteNextState <= RD_STATE_DELAY1a;
					end
			endcase
		end
	endtask
	
	task wb_check_SR;
		input [5:0]caller_next_state;
		begin
			case(CSRState)
			CSR_STATE_READ_SR: begin
				check_SR = 1'b1;
				wb_read_reg(1, SR, check_SR, 0, 0);
				CSRNextState <= CSR_STATE_WAIT;
				end
			CSR_STATE_WAIT: begin
				if(~block_reading)
					CSRNextState <= CSR_STATE_CHECK_SR;
				end
			CSR_STATE_CHECK_SR: begin
				if(check_SR == 0)
					I2C_CMD_NextState <= caller_next_state;
				CSRNextState <= CSR_STATE_READ_SR;
				end
			endcase
		end
	endtask

	always@(posedge clk, negedge rstn) begin
		if(~rstn) begin
			CSRState      <= CSR_STATE_READ_SR;
			WriteState    <= WR_STATE_DELAY1a;
			ReadState     <= RD_STATE_DELAY1a;
			I2C_CMD_State <= I2C_CMD_STATE_INIT;
			end
		else begin
			CSRState      <= CSRNextState;
			WriteState    <= WriteNextState;
			ReadState     <= ReadNextState;
			I2C_CMD_State <= I2C_CMD_NextState;
			end
	end
	
	always@(negedge clk, negedge rstn) begin
		if(~rstn) begin
			CSRNextState      <= CSR_STATE_READ_SR;
			WriteNextState    <= WR_STATE_DELAY1a;
			ReadNextState     <= RD_STATE_DELAY1a;
			I2C_CMD_NextState <= I2C_CMD_STATE_INIT;
			READ_CMD          <= 0;
			WRITE_CMD         <= 0;
			go_next           <= 0;
			block_reading     <= 0;
			block_writing     <= 0;
			end
		else begin
			case(I2C_CMD_State)
				I2C_CMD_STATE_INIT: begin
					wb_write_reg(1, PRER_LO, 8'h64, 0, 0); // load prescaler lo-byte
					wb_write_reg(1, PRER_HI, 8'h00, 0, 0); // load prescaler hi-byte
					wb_write_reg(1, CTR,     8'h80, I2C_CMD_STATE_WAIT, 1); // enable core
					end
				I2C_CMD_STATE_WAIT: begin
					if(WRITE_CMD == 1'b1)
						I2C_CMD_NextState <= I2C_CMD_STATE_W_SET_SLAVE;
					else if(READ_CMD == 1'b1)
						I2C_CMD_NextState <= I2C_CMD_STATE_R_SET_SLAVE;
					else
						I2C_CMD_NextState <= I2C_CMD_STATE_WAIT;
					end
					
				//Start of WRITE sequence
				I2C_CMD_STATE_W_SET_SLAVE:
					wb_write_reg(1, TXR, {SADR,WR}, I2C_CMD_STATE_W_SET_START, 1 );// present slave address, set write-bit
				I2C_CMD_STATE_W_SET_START:
					wb_write_reg(0, CR,      8'h90, I2C_CMD_STATE_W_READ_CH_SR1, 1 ); // set command (start, write)
				I2C_CMD_STATE_W_READ_CH_SR1: // check tip bit (Transfer In Progress)
					wb_check_SR(I2C_CMD_STATE_W_SET_S_REG);
				I2C_CMD_STATE_W_SET_S_REG:
					wb_write_reg(1, TXR,     8'h00, I2C_CMD_STATE_W_CMD_WRITE1, 1); // present slave's memory address - NEED TO SET THIS to something
				I2C_CMD_STATE_W_CMD_WRITE1:
					wb_write_reg(0, CR,      8'h10, I2C_CMD_STATE_W_READ_CH_SR2, 1); // set command (write)
				I2C_CMD_STATE_W_READ_CH_SR2: // check tip bit (Transfer In Progress)
					wb_check_SR(I2C_CMD_STATE_W_SET_TX_REG);
				I2C_CMD_STATE_W_SET_TX_REG:
					wb_write_reg(1, TXR,     8'ha5, I2C_CMD_STATE_W_CMD_WRITE2, 1); // present data - NEED TO SET THIS to something
				I2C_CMD_STATE_W_CMD_WRITE2:
					wb_write_reg(0, CR,      8'h10, I2C_CMD_STATE_W_SET_STOP, 1); // set command (write)
				I2C_CMD_STATE_W_SET_STOP:
					wb_write_reg(0, CR,      8'h50, I2C_CMD_STATE_W_READ_CH_SR3, 1); // set command (stop, write)
				I2C_CMD_STATE_W_READ_CH_SR3: begin// check tip bit (Transfer In Progress)
					wb_check_SR(I2C_CMD_STATE_WAIT);
					if(check_SR == 1'b0) begin
						WRITE_CMD     <= 1'b0;
						READ_CMD      <= 1'b0;
						end
					end
					

				//Start of READ sequence
				I2C_CMD_STATE_R_SET_SLAVE:
					wb_write_reg(1, TXR,{SADR,WR}, I2C_CMD_STATE_R_SET_START, 1 ); // present slave address, set write-bit
				I2C_CMD_STATE_R_SET_START:
					wb_write_reg(0, CR,     8'h90, I2C_CMD_STATE_R_READ_CH_SR1, 1); // set command (start, write)
				I2C_CMD_STATE_R_READ_CH_SR1: // check tip bit (Transfer In Progress)
					wb_check_SR(I2C_CMD_STATE_R_SET_S_REG);			
				I2C_CMD_STATE_R_SET_S_REG:
					wb_write_reg(1, TXR,     8'h01, I2C_CMD_STATE_R_CMD_WRITE1, 1); // present slave's memory address
				I2C_CMD_STATE_R_CMD_WRITE1:
					wb_write_reg(0, CR,      8'h10, I2C_CMD_STATE_R_READ_CH_SR2, 1); // set command (write)
				I2C_CMD_STATE_R_READ_CH_SR2: // check tip bit (Transfer In Progress)
					wb_check_SR(I2C_CMD_STATE_R_SET_SLAVE_RD);
				I2C_CMD_STATE_R_SET_SLAVE_RD:
					wb_write_reg(1, TXR, {SADR,RD}, I2C_CMD_STATE_R_CMD_READ_START, 1); // present slave's address, set read-bit
				I2C_CMD_STATE_R_CMD_READ_START:
					wb_write_reg(0, CR,      8'h90, I2C_CMD_STATE_R_READ_CH_SR3, 1); // set command (start, write)
				I2C_CMD_STATE_R_READ_CH_SR3: // check tip bit (Transfer In Progress)
					wb_check_SR(I2C_CMD_STATE_R_CMD_READ_ACKM);
				I2C_CMD_STATE_R_CMD_READ_ACKM:
					wb_write_reg(1, CR,      8'h20, I2C_CMD_STATE_R_READ_CH_SR4, 1); // set command (read, ack_read)
				I2C_CMD_STATE_R_READ_CH_SR4: // check tip bit (Transfer In Progress)c
					wb_check_SR(I2C_CMD_STATE_R_SET_STOP);
				I2C_CMD_STATE_R_READ_STR_RX:
					wb_read_reg(1, RXR, DATA_OUT, I2C_CMD_STATE_R_SET_STOP, 1); //Store read data
				I2C_CMD_STATE_R_SET_STOP:
					wb_write_reg(1, CR,      8'h40, I2C_CMD_STATE_R_READ_CH_SR5, 1); // set command (stop)
				I2C_CMD_STATE_R_READ_CH_SR5: begin// check tip bit (Transfer In Progress)
					wb_check_SR(I2C_CMD_STATE_WAIT);
					if(check_SR == 1'b0) begin
						WRITE_CMD     <= 1'b0;
						READ_CMD      <= 1'b0;
						end
					end

			endcase
		end
	end
endmodule