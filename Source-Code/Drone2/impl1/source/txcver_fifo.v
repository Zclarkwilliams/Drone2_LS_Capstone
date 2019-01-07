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
// --------------------------------------------------------------------
//`ifndef TXFIFO_FILE
//`define TXFIFO_FILE
//`include "system_conf.v"
//`include "pmi_fifo.v"
`timescale 1ns / 10ps
module txcver_fifo (Data, Clock, WrEn, RdEn, Reset, Q, Empty, Full,
                    AlmostEmpty, AlmostFull);

   input [7:0] Data;
   input Clock;
   input WrEn;
   input RdEn;
   input Reset;
   output [7:0] Q;
   output Empty;
   output Full;
   output AlmostEmpty;
   output AlmostFull;
        pmi_fifo #(
                 .pmi_data_width(8),
                 .pmi_data_depth(16),
                 .pmi_full_flag(16),
                 .pmi_empty_flag(0),
                 .pmi_almost_full_flag(8),
                 .pmi_almost_empty_flag(4),
                 .pmi_regmode("noreg"),
                 .pmi_family("ECP3"),
                 .module_type("pmi_fifo"),
                 .pmi_implementation("EBR")
                 )
     		tx_fifo_inst
                 (
                 	.Data(Data),
                  .Clock(Clock),
                  .WrEn(WrEn),
                  .RdEn(RdEn),
                  .Reset(Reset),
                  .Q(Q),
                  .Empty(Empty),
                  .Full(Full),
                  .AlmostEmpty(AlmostEmpty),
                  .AlmostFull(AlmostFull)
                  );
//
endmodule
//`endif
