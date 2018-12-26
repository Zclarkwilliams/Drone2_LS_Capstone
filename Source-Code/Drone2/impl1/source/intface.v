//   ==================================================================
//   >>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<
//   ------------------------------------------------------------------
//   Copyright (c) 2013 by Lattice Semiconductor Corporation
//   ALL RIGHTS RESERVED 
//   ------------------------------------------------------------------
//
//   Permission:
//
//      Lattice SG Pte. Ltd. grants permission to use this code
//      pursuant to the terms of the Lattice Reference Design License Agreement. 
//
//
//   Disclaimer:
//
//      This VHDL or Verilog source code is intended as a design reference
//      which illustrates how these types of functions can be implemented.
//      It is the user's responsibility to verify their design for
//      consistency and functionality through the use of formal
//      verification methods.  Lattice provides no warranty
//      regarding the use or functionality of this code.
//
//   --------------------------------------------------------------------
//
//                  Lattice SG Pte. Ltd.
//                  101 Thomson Road, United Square #07-02 
//                  Singapore 307591
//
//
//                  TEL: 1-800-Lattice (USA and Canada)
//                       +65-6631-2000 (Singapore)
//                       +1-503-268-8001 (other locations)
//
//                  web: http://www.latticesemi.com/
//                  email: techsupport@latticesemi.com
//
//   --------------------------------------------------------------------
//
// --------------------------------------------------------------------
// Code Revision History :
// --------------------------------------------------------------------
// Ver: | Author |Mod. Date |Changes Made:
// V1.0 |        |          | Initial ver
// V1.1 | S.R.   |18/12/08  | modified to support Mico8
//
// --------------------------------------------------------------------


`ifndef INTFACE_FILE
`define INTFACE_FILE
`define MODEM
//`endif
//`include "txcver_fifo.v"
`timescale 1ns/10ps
module intface #(parameter CLK_IN_MHZ = 25,
                 parameter BAUD_RATE = 115200,
                 parameter ADDRWIDTH = 3,
                 parameter DATAWIDTH = 8,
                 parameter FIFO = 0) 
       (
        // Global reset and clock
        reset,
        clk,
        // wishbone interface
        adr_i,
        dat_i,
        dat_o,
        stb_i,
        cyc_i,
        we_i,
        sel_i,
        bte_i,
        ack_o,
        intr,
        // Registers
        rbr,
        rbr_fifo,		
        thr,
        // Rising edge of registers read/write strobes
        rbr_rd,
        thr_wr,
        lsr_rd,
        `ifdef MODEM
        msr_rd,
        msr,
        mcr,
        `endif
        // Receiver/Transmitter control
        databits,
        stopbits,
        parity_en,
        parity_even,
        parity_stick,
        tx_break,
        // Receiver/Transmitter status
        rx_rdy,
        overrun_err,
        parity_err,
        frame_err,
        break_int,
        thre,
        temt,
	fifo_empty,
	fifo_empty_thr,
	thr_rd,
	fifo_almost_full,
	divisor
       ); 
    
    input   reset ;
    input   clk;   
    output  fifo_empty_thr; 
    input   fifo_empty; 
    input   fifo_almost_full;
    input   thr_rd;
    input [7:0] adr_i;
    input [15:0] dat_i;
    input        we_i ; 
    input        stb_i;
    input        cyc_i;
    input [3:0]  sel_i;
    input [1:0]  bte_i;

    input [7:0]          rbr_fifo;
    input [DATAWIDTH-1:0] rbr; 
    input                 rx_rdy      ;
    input                 overrun_err ;
    input                 parity_err  ;
    input                 frame_err   ;
    input                 break_int   ;
    input                 thre        ;
    input                 temt        ;
    
    output                  lsr_rd   ;
    output [15:0] dat_o ;
    output                  intr ;
    output                  ack_o;
    output [DATAWIDTH-1:0]  thr  ;  
    output                  rbr_rd;
    output                  thr_wr;  
    
    output [1:0]            databits;
    output [1:0]            stopbits;
    output                  parity_en   ;
    output                  parity_even ;
    output                  parity_stick;
    output                  tx_break    ;
    `ifdef MODEM
    output                  msr_rd   ;
    input  [DATAWIDTH-1:0]  msr   ;
    output [1:0]            mcr  ;  
    `endif
    output [15:0]           divisor;
    
   reg    ack_o;
    
   reg  [DATAWIDTH-1:0] data_8bit ;
   wire [15:0]    dat_o ;
  
   wire [DATAWIDTH-1:0]    thr_fifo;
   reg [DATAWIDTH-1:0]     thr_nonfifo; 	   

 generate
   if (FIFO == 1)
     assign thr  = thr_fifo;
   else
     assign thr  = thr_nonfifo;
 endgenerate


   reg [6:0] lsr;
   reg  [6:0] lcr;

   wire [3:0] 		    iir     ;
  `ifdef MODEM
   reg  [3:0] 		    ier     ;
   `else 
   reg  [2:0] 		    ier     ;
   `endif 
   
   wire 		    rx_rdy_int   ;
   wire 		    thre_int    ;
   wire 		    dataerr_int ;
   wire 		    data_err     ;
   
   wire thr_wr_strobe;
   wire rbr_rd_strobe;
   wire iir_rd_strobe;
   reg iir_rd_strobe_delay;
   wire lsr_rd_strobe;
   wire div_wr_strobe;
   wire lsr_rd;
   reg lsr2_r, lsr3_r, lsr4_r;

   reg  thr_wr;
   wire rbr_rd_fifo;	   	   
   reg  rbr_rd_nonfifo;

 generate
   if (FIFO == 1)
     assign rbr_rd  = rbr_rd_fifo;
   else
     assign rbr_rd  = rbr_rd_nonfifo;
 endgenerate

   `ifdef MODEM 
   wire  modem_stat   ;
   wire  modem_int   ;
   wire  msr_rd_strobe;
   wire  msr_rd   ; 
   reg  [1:0]  mcr     ;  
   reg         msr_rd_strobe_detect;
   `endif

// FIFO signals for FIFO mode
   wire         fifo_full_thr;
   wire         fifo_empty_thr;
   wire         fifo_almost_full_thr;
   wire         fifo_almost_empty_thr;
   wire [7:0]   fifo_din_thr;
   reg          fifo_wr_thr;
   reg          fifo_wr_q_thr;
   wire         fifo_wr_pulse_thr;

   // UART baud 16x clock generator
   reg [15:0]   divisor;
   always @(posedge clk or posedge reset) begin
      if (reset)
        divisor <= ((CLK_IN_MHZ*1000*1000)/(BAUD_RATE));
      else if (div_wr_strobe)
        divisor <= dat_i[15:0];
   end
 //changed for mico8 support from 5bit to 3 bit reg_addr
   localparam A_RBR = 3'b000;
   localparam A_THR = 3'b000;
   localparam A_IER = 3'b001;
   localparam A_IIR = 3'b010;
   localparam A_LCR = 3'b011;
   localparam A_LSR = 3'b101;
   localparam A_DIV = 3'b111; 
   
   
`ifdef MODEM
   localparam A_MSR = 3'b110;//00;
   localparam A_MCR = 3'b100;//00;
`endif

  always @(posedge clk or posedge reset)  begin 
    if (reset)
      thr_wr <= 1'b0;
    else 
      thr_wr <= thr_wr_strobe;
   end 

  assign lsr_rd = lsr_rd_strobe;

  assign rbr_rd_fifo = rbr_rd_strobe;

  always @(posedge clk or posedge reset)  begin 
   if (reset)
     rbr_rd_nonfifo <= 1'b0;
    else 
     rbr_rd_nonfifo <= rbr_rd_strobe;
   end 

   `ifdef MODEM

   assign  msr_rd = msr_rd_strobe;

   `endif
  
   ////////////////////////////////////////////////////////////////////////////////
   // Registers Read/Write Control Signals
   ////////////////////////////////////////////////////////////////////////////////
 generate
   if (FIFO == 1)
     assign thr_wr_strobe = (adr_i[ADDRWIDTH-1:0] == A_THR) &&  cyc_i && stb_i &&  we_i && ~fifo_full_thr && ~ack_o;
   else
     assign thr_wr_strobe = (adr_i[ADDRWIDTH-1:0] == A_THR) &&  cyc_i && stb_i &&  we_i;
 endgenerate

 generate
   if (FIFO == 1)
     assign rbr_rd_strobe = (adr_i[ADDRWIDTH-1:0] == A_RBR) &&  cyc_i && stb_i && ~we_i && ~fifo_empty && ~ack_o;
   else
     assign rbr_rd_strobe = (adr_i[ADDRWIDTH-1:0] == A_RBR) &&  cyc_i && stb_i && ~we_i;
 endgenerate

 generate
   if (FIFO == 1)
     assign iir_rd_strobe = (adr_i[ADDRWIDTH-1:0] == A_IIR) &&  cyc_i && stb_i && ~we_i && ~ack_o;  
   else 	   
     assign iir_rd_strobe = (adr_i[ADDRWIDTH-1:0] == A_IIR) &&  cyc_i && stb_i && ~we_i;
 endgenerate 

 generate
   if (FIFO == 1)
     assign lsr_rd_strobe = (adr_i[ADDRWIDTH-1:0] == A_LSR) &&  cyc_i && stb_i && ~we_i && ~ack_o; 
   else    	
     assign lsr_rd_strobe = (adr_i[ADDRWIDTH-1:0] == A_LSR) &&  cyc_i && stb_i && ~we_i;  
 endgenerate 
 
 `ifdef MODEM 
   assign msr_rd_strobe = (adr_i[ADDRWIDTH-1:0] == A_MSR) &&  cyc_i && stb_i && ~we_i;
 `endif
 
   assign div_wr_strobe = (adr_i[ADDRWIDTH-1:0] == A_DIV) &&  cyc_i && stb_i &&  we_i;  
   
  ////////////////////////////////////////////////////////////////////////////////
  // Registers Read/Write Operation
  ////////////////////////////////////////////////////////////////////////////////
 generate
  if (FIFO == 1) begin
   always @(cyc_i or stb_i or we_i or adr_i or rbr_fifo or iir 
	   `ifdef MODEM
           or msr
	   `endif 
	   or lsr)
   begin
     case (adr_i[ADDRWIDTH-1:0])
       A_RBR: data_8bit <= rbr_fifo;
       A_IIR: data_8bit <= {4'b0000, iir};
       A_LSR: data_8bit <= {1'b0,lsr};
       `ifdef MODEM
       A_MSR: data_8bit <= msr;
       `endif
       default: data_8bit <= 8'b11111111;
   endcase	   
   end
  end
  else begin	  
  // Register Read
 always @(posedge clk or posedge reset)  begin 
   if (reset)
     data_8bit <= 8'b11111111;
    else if (cyc_i && stb_i && ~we_i)
     case (adr_i[ADDRWIDTH-1:0])
       A_RBR: data_8bit <= rbr;
       A_IIR: data_8bit <= {4'b0000, iir};
       A_LSR: data_8bit <= {1'b0,lsr};
       `ifdef MODEM
       A_MSR: data_8bit <= msr;
       `endif
       default: data_8bit <= 8'b11111111;
   endcase
 end
 end
 endgenerate 
  assign dat_o = {24'h000000,data_8bit};   


generate
if (FIFO == 0)
begin
    always @(posedge clk or posedge reset)  begin
        if (reset) begin
            thr_nonfifo <= 0;
             `ifdef MODEM
           ier <= 4'b0000;
           mcr <= 2'b00;
            `else
           ier <= 3'b000;
            `endif
            lcr <= 7'h00; end 
       else if (cyc_i && stb_i && we_i)
          case (adr_i[ADDRWIDTH-1:0])  
             	A_THR: thr_nonfifo <= dat_i[7:0];
                `ifdef MODEM
               A_IER: ier <= dat_i[3:0];
               A_MCR: mcr <= dat_i[1:0];
                `else 
               A_IER: ier <= dat_i[2:0];
                `endif
               A_LCR: lcr <= dat_i[6:0];
           default: ;
          endcase
      end
end
else 
begin
    always @(posedge clk or posedge reset)  begin
        if (reset) begin
             `ifdef MODEM
           ier <= 4'b0000;
           mcr <= 2'b00;
            `else
           ier <= 3'b000;
            `endif
            lcr <= 7'h00; end 
       else if (cyc_i && stb_i && we_i)
          case (adr_i[ADDRWIDTH-1:0])  
                `ifdef MODEM
               A_IER: ier <= dat_i[3:0];
               A_MCR: mcr <= dat_i[1:0];
                `else 
               A_IER: ier <= dat_i[2:0];
                `endif
               A_LCR: lcr <= dat_i[6:0];
           default: ;
          endcase
end

end

endgenerate

 generate
 if (FIFO == 1) begin
   assign fifo_wr_pulse_thr = thr_wr_strobe;
   assign fifo_din_thr      = dat_i[7:0];
   txcver_fifo TX_FIFO(
	              .Data        (fifo_din_thr),
		      			.Clock       (clk),
		      .WrEn        (fifo_wr_pulse_thr),
		      .RdEn        (thr_rd),
		      .Reset       (reset),
		      .Q           (thr_fifo),
		      .Empty       (fifo_empty_thr),
		      .Full        (fifo_full_thr),
		      .AlmostEmpty (fifo_almost_empty_thr),
		      .AlmostFull  (fifo_almost_full_thr)   
	              );
 end
endgenerate

   ////////////////////////////////////////////////////////////////////////////////
   //  Line Control Register
   ////////////////////////////////////////////////////////////////////////////////
   
   // databits : "00"=5-bit, "01"=6-bit, "10"=7-bit, "11"=8-bit
   assign databits = lcr[1:0];
   
   // stopbits : "00"=1-bit, "01"=1.5-bit(5-bit data), "10"=2-bit(6,7,8-bit data)
   assign stopbits = (lcr[2] == 1'b0) ? 2'b00 : (lcr[2:0] == 3'b100) ? 2'b01 : 2'b10;
   
   // parity_en : '0'=Parity Bit Enable, '1'=Parity Bit Disable
   assign parity_en = lcr[3];
   
   // parity_even : '0'=Even Parity Selected, '1'=Odd Parity Selected
   assign parity_even = lcr[4];
   
   // parity_stick : '0'=Stick Parity Disable, '1'=Stick Parity Enable
   assign parity_stick = lcr[5];
   
   // tx_break : '0'=Disable BREAK assertion, '1'=Assert BREAK
   assign tx_break = lcr[6];
   
   ////////////////////////////////////////////////////////////////////////////////
   //  Line Status Register
   ////////////////////////////////////////////////////////////////////////////////
 generate
 if (FIFO == 1) begin
  
   always @(posedge clk or posedge reset)
     if (reset)
       lsr2_r <= 1'b0;
     else if (parity_err)
       lsr2_r  <= 1'b1;
     else if (lsr_rd_strobe)
       lsr2_r  <= 1'b0;

   always @(posedge clk or posedge reset)
     if (reset)
       lsr3_r <= 1'b0;
     else if (frame_err)
       lsr3_r  <= 1'b1;
     else if (lsr_rd_strobe)
       lsr3_r  <= 1'b0;

   always @(posedge clk or posedge reset)
     if (reset)
       lsr4_r <= 1'b0;
     else if (break_int)
       lsr4_r  <= 1'b1;
     else if (lsr_rd_strobe)
       lsr4_r  <= 1'b0;

   always @(posedge clk)
      lsr <= {temt , thre , lsr4_r , lsr3_r , lsr2_r , overrun_err , rx_rdy};
 end
 
 else
 
   always @(temt or thre or break_int or frame_err or parity_err or overrun_err or rx_rdy)
      lsr =  {temt , thre , break_int , frame_err , parity_err , overrun_err , rx_rdy};
       
endgenerate

   ////////////////////////////////////////////////////////////////////////////////
   // Interrupt Arbitrator
   ////////////////////////////////////////////////////////////////////////////////
   
   // Int is the common interrupt line for all internal UART events
`ifdef MODEM
   assign intr = rx_rdy_int | thre_int | dataerr_int | modem_int;
`else
   assign intr = rx_rdy_int | thre_int | dataerr_int;
`endif
   
   // Receiving Data Error Flags including Overrun, Parity, Framing and Break
 generate
   if (FIFO == 1)
     assign data_err = overrun_err | lsr2_r | lsr3_r | lsr4_r;
   else	   
     assign data_err = overrun_err | parity_err | frame_err | break_int;
 endgenerate

   // Whenever bit0, 1, 2,or 3 is set to '1', A_WB Modem Status Interrupt is generated
`ifdef MODEM
   assign modem_stat = msr[0] | msr[1] | msr[2] | msr[3];
`endif

generate
 if (FIFO == 1)
  always @(posedge clk or posedge reset)
     if (reset)
       iir_rd_strobe_delay <= 1'b0;
     else
       iir_rd_strobe_delay <= iir_rd_strobe;
endgenerate

   // State Machine Definition
   localparam  idle = 3'b000;
   localparam  int0 = 3'b001;
   localparam  int1 = 3'b010;
   localparam  int2 = 3'b011;
   localparam  int3 = 3'b100;
   reg [2:0]  cs_state;


generate 
if (FIFO == 1) begin
   always @(posedge clk or posedge reset) begin
        if (reset)
          cs_state <= idle;
        else
          case (cs_state)
            idle: begin
               if (ier[2] == 1'b1 && data_err == 1'b1 )
                 cs_state <= int0;
               else if (ier[0] == 1'b1 && (fifo_almost_full || !fifo_empty) )	       
                 cs_state <= int1;
               else if (ier[1] == 1'b1 && thre == 1'b1)
                 cs_state <= int2;
            `ifdef MODEM
               else if (ier[3] == 1'b1 && modem_stat == 1'b1)
                 cs_state <= int3;
            `endif
            end
            int0: begin
	       if ((lsr_rd_strobe == 1'b1) || (ier[2] == 1'b0)) begin    
		 if (ier[0] == 1'b1 && fifo_almost_full) 
		   cs_state <= int1;
                else	      
	           cs_state <= idle; end
            end
            int1: begin
               if (data_err == 1'b1 && ier[2] == 1'b1)
		  cs_state <= int0;     
	       else if (!fifo_almost_full || (ier[0] == 1'b0))	     	       
	          cs_state <= idle;
            end
            int2: begin
               if (iir_rd_strobe_delay || (thre == 1'b0) || (ier[1] == 1'b0))  	       
                 cs_state <= idle;
            end
           `ifdef MODEM
            int3: begin
               if ((msr_rd_strobe)|| (ier[3] == 1'b0))
                 cs_state <= idle;
            end
           `endif
            default: cs_state <= idle;
          endcase
       end end
       
else  begin
   
   always @(posedge clk or posedge reset) begin
        if (reset)
          cs_state <= idle;
        else
          case (cs_state)
            idle: begin
               if (ier[2] == 1'b1 && data_err == 1'b1)
                 cs_state <= int0;
               else if (ier[0] == 1'b1 && rx_rdy == 1'b1)       	       
                 cs_state <= int1;
               else if (ier[1] == 1'b1 && thre == 1'b1)
                 cs_state <= int2;
            `ifdef MODEM
               else if (ier[3] == 1'b1 && modem_stat == 1'b1)
                 cs_state <= int3;
            `endif
            end
            int0: begin
	       if ((lsr_rd_strobe == 1'b1) || (ier[2] == 1'b0)) begin	      
		 if (ier[0] == 1'b1 && rx_rdy) 
		   cs_state <= int1;
                else	      
	           cs_state <= idle; end
            end
            int1: begin
               if (data_err == 1'b1 && ier[2] == 1'b1)
		  cs_state <= int0;     
	       else if ((rx_rdy == 1'b0) || (ier[0] == 1'b0)) 	       
	           cs_state <= idle;
            end
            int2: begin 	       
	       if (iir_rd_strobe || (thre == 1'b0) || (ier[1] == 1'b0))	       
                 cs_state <= idle;
            end
           `ifdef MODEM
            int3: begin
               if ((msr_rd_strobe)|| (ier[3] == 1'b0))
                 cs_state <= idle;
            end
           `endif
            default: cs_state <= idle;
          endcase
     end
end
endgenerate

 // ACK signal generate
  always @(posedge clk or posedge reset) begin
    if (reset)
      ack_o <= 1'b0;
    else if (ack_o)
      ack_o <= 1'b0;
    else if (cyc_i & stb_i) 
      ack_o <= 1'b1;
 end
   
   // Set Receiver Line Status Interrupt
   assign dataerr_int = (cs_state == int0) ? 1'b1 : 1'b0;
   
   // Set Received Data Available Interrupt
   assign rx_rdy_int  = (cs_state == int1) ? 1'b1 : 1'b0;
   
   // Set thr Empty Interrupt
   assign thre_int    = (cs_state == int2) ? 1'b1 : 1'b0;
   
   // Set MODEM Status Interrupt
   `ifdef MODEM
   assign modem_int   = (cs_state == int3) ? 1'b1 : 1'b0;
   `endif
   
   // Update IIR
   assign iir = (cs_state == int0) ? 4'b0110 :
                (cs_state == int1) ? 4'b0100 :
                (cs_state == int2) ? 4'b0010 :
                 `ifdef MODEM
                (cs_state == int3) ? 4'b0000 :
                 `endif
                4'b0001 ;  // No Interrupt Pending

endmodule
`endif // INTFACE_FILE
