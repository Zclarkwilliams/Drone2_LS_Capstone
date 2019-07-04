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
// --------------------------------------------------------------------
// Code Revision History :
// --------------------------------------------------------------------
// Ver: | Author |Mod. Date |Changes Made:
// V1.0 |        |          | Initial ver
// V1.1 | S.R.   |18/12/08  | modified to support Mico8
// --------------------------------------------------------------------

`ifndef RXFIFO_FILE
`define RXFIFO_FILE
`timescale 1ns / 10ps
module rxcver_fifo (Data, Clock, WrEn, RdEn, Reset, Q, Q_error, Empty, Full,
                    AlmostEmpty, AlmostFull);

   input [10:0] Data;
   input Clock;
   input WrEn;
   input RdEn;
   input Reset;
   output [7:0] Q;
   output [2:0] Q_error;
   output Empty;
   output Full;
   output AlmostEmpty;
   output AlmostFull;
   wire[7:0] Q_node; 
   generate
      begin

           pmi_fifo #(
                      .pmi_data_width(8),
                      .pmi_data_depth(16),
                      .pmi_full_flag(16),
                      .pmi_empty_flag(0),
                      .pmi_almost_full_flag(1),
                      .pmi_almost_empty_flag(0),
                      .pmi_regmode("noreg"),
                      .pmi_family("ECP3"),
                      .module_type("pmi_fifo"),
                      .pmi_implementation("EBR"))
           rx_fifo_inst        
                     (.Data(Data[10:3]),
                      .Clock(Clock),
                      .WrEn(WrEn),
                      .RdEn(RdEn),
                      .Reset(Reset),
                      .Q(Q_node),
                      .Empty(Empty),
                      .Full(Full),
                      .AlmostEmpty(AlmostEmpty),
                      .AlmostFull(AlmostFull));   
     end  
   endgenerate 
   reg [2:0] fifo [15:0];
   

   reg [4:0] wr_pointer1 = 0;
   reg [4:0] rd_pointer1 = 0;
   reg [4:0] rd_pointer_prev = 0;
   reg valid_RdEn;
   

   always @(posedge Clock or posedge Reset)
     begin
        if (Reset)
       begin
            wr_pointer1 <=  0;
                rd_pointer1 <=  0; 
                valid_RdEn <=  0;
                rd_pointer_prev <= 0;        
         fifo[0] <=  0;
        fifo[1] <=  0;
        fifo[2] <=  0;
        fifo[3] <=  0;
        fifo[4] <=  0;
        fifo[5] <=  0;
        fifo[6] <=  0;
        fifo[7] <=  0;
        fifo[8] <=  0;
        fifo[9] <=  0;
        fifo[10] <=  0;
        fifo[11] <=  0;
        fifo[12] <=  0;
        fifo[13] <=  0;
        fifo[14] <=  0;
        fifo[15] <=  0;    
           end    
        else
           begin
         if (WrEn == 1 && RdEn !=1 && Full !=1) begin
           fifo[wr_pointer1%16] <=  Data[2:0];
           wr_pointer1          <=  wr_pointer1 + 1; end

         else if (WrEn != 1 && RdEn ==1 && Empty !=1) begin
               valid_RdEn          <=  1'b1;
           rd_pointer_prev     <=  rd_pointer1;
           rd_pointer1          <=  rd_pointer1 +1; end 

         else if (WrEn == 1 && RdEn ==1) begin
           rd_pointer_prev     <=  rd_pointer1;
               valid_RdEn          <=  1'b1;         
           fifo[wr_pointer1%16] <=  Data[2:0];
           rd_pointer1          <=  rd_pointer1 + 1;
           wr_pointer1          <=  wr_pointer1 + 1; 
              end  
//         else
//           valid_RdEn          <= 1'b0;         

             if (valid_RdEn)  begin   
           fifo[rd_pointer_prev%16] <= 0;
           valid_RdEn          <= 1'b0;
         end    
       end
     end


  // Data is only valid for single clock cycle after read to the FIFO occurs  
//    assign   Q = {Q_node, fifo[rd_pointer_prev%16]};
   assign Q = Q_node;
   assign Q_error = fifo[rd_pointer_prev%16]; 

endmodule
   
`endif
