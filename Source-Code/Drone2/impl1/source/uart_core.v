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

`ifndef UART_CORE_FILE
`define UART_CORE_FILE
//`include "txmitt.v"
`define MODEM
//`endif
`timescale 1ns/10ps
module uart_core
      #(parameter CLK_IN_MHZ = 25,
        parameter BAUD_RATE  = 115200,
        parameter ADDRWIDTH = 3,
        parameter DATAWIDTH = 8,
        parameter FIFO = 0
       )
      (
       // Global reset and clock
       RESET    ,
       CLK      ,
      
       // wishbone interface
       UART_ADR_I   ,
       UART_DAT_I   ,
       UART_DAT_O   ,
       UART_STB_I   ,
       UART_CYC_I   ,
       UART_WE_I    ,
       UART_SEL_I   ,
       UART_CTI_I   ,
       UART_BTE_I   ,     
       UART_ACK_O   ,
       //UART_RTY_O   ,
       //UART_ERR_O,
       INTR     ,
      
       // Receiver interface
       SIN      ,
       RXRDY_N  ,
       
       // Modem interface
       `ifdef MODEM
       DCD_N    ,
       CTS_N    ,
       DSR_N    ,
       RI_N     ,
       DTR_N    ,
       RTS_N    ,      
       `endif
        
       // Transmitter interface
       SOUT     ,
       TXRDY_N        
       );

 input                RESET  ;
 input                CLK    ;

 input [7:0]         UART_ADR_I; 
 input [15:0]         UART_DAT_I; 
 input                SIN ;
                      
 input                UART_STB_I ;
 input                UART_CYC_I ;
 input                UART_WE_I;
 input [1:0]          UART_BTE_I;
 input [3:0]          UART_SEL_I;
 input [2:0]          UART_CTI_I;
  `ifdef MODEM        
 input                DCD_N;
 input                CTS_N;
 input                DSR_N;
 input                RI_N;     
 `endif               
 output[15:0]         UART_DAT_O;
 output               UART_ACK_O;
// output               UART_RTY_O;
// output               UART_ERR_O;
 output               INTR;
                      
 output               RXRDY_N;
 output               SOUT;
 output               TXRDY_N;
`ifdef MODEM          
 output               DTR_N;
 output               RTS_N;
 wire [DATAWIDTH-1:0] MSR;
 wire [1:0]           MCR;  
 `endif
 
 wire [7:0]          RBR_FIFO;
 wire [DATAWIDTH-1:0] RBR;
 wire [DATAWIDTH-1:0] THR;
 wire [1:0]           databits;
 wire [1:0]           stopbits;
 wire                 parity_en;
 wire                 parity_stick;
 wire                 tx_break;
 wire                 thr_wr;
 wire                 rbr_rd;
 wire                 lsr_rd;
 wire                 rx_rdy;
 wire                 parity_err;
 wire                 frame_err;
 wire                 overrun_err;
 wire                 break_int;
 wire                 THRE;
 wire                 TEMT; 
 wire[15:0]           UART_DAT_O; 

 wire                 fifo_empty;
 wire                 fifo_empty_thr;
 wire                 thr_rd;
 wire                 fifo_almost_full;
                      
 wire [15:0]          divisor;

 //assign               UART_RTY_O = 1'b0;
// assign               UART_ERR_O = 1'b0;
   
 `ifdef MODEM
     intface  #(.CLK_IN_MHZ(CLK_IN_MHZ),
                .BAUD_RATE(BAUD_RATE),
                .ADDRWIDTH(ADDRWIDTH),
                .DATAWIDTH(DATAWIDTH),
                .FIFO(FIFO)) 
         u_intface (
        .reset            (RESET       ),
        .clk              (CLK         ),
        .adr_i            (UART_ADR_I  ),
        .dat_i            (UART_DAT_I  ),
        .dat_o            (UART_DAT_O  ),
        .stb_i            (UART_STB_I  ),
        .cyc_i            (UART_CYC_I  ),
        .we_i             (UART_WE_I   ),
        .sel_i            (UART_SEL_I  ),        
        .bte_i            (UART_BTE_I  ),
        .ack_o            (UART_ACK_O  ),
        .intr             (intr        ),
        .rbr              (RBR         ),
		.rbr_fifo         (RBR_FIFO    ),
        .thr              (THR         ),
        .rbr_rd           (rbr_rd      ),
        .thr_wr           (thr_wr      ),
        .lsr_rd           (lsr_rd      ),
        .msr_rd           (msr_rd      ),
        .msr              (MSR         ),
        .mcr              (MCR         ),
        .databits         (databits    ),
        .stopbits         (stopbits    ),
        .parity_en        (parity_en   ),
        .parity_even      (parity_even ),
        .parity_stick     (parity_stick),
        .tx_break         (tx_break    ),
        .rx_rdy           (rx_rdy      ),
        .overrun_err      (overrun_err ),
        .parity_err       (parity_err  ),
        .frame_err        (frame_err   ),
        .break_int        (break_int   ),
        .thre             (THRE        ),
        .temt             (TEMT        ),
	.fifo_empty       (fifo_empty  ),
	.fifo_empty_thr   (fifo_empty_thr),
	.thr_rd           (thr_rd),
	.fifo_almost_full (fifo_almost_full),
	.divisor          (divisor)
  );
`else
     intface  #(.CLK_IN_MHZ(CLK_IN_MHZ),
                .BAUD_RATE(BAUD_RATE),
                .ADDRWIDTH(ADDRWIDTH),
                .DATAWIDTH(DATAWIDTH),
                .FIFO(FIFO)) 
          u_intface (
        .reset            (RESET         ),
        .clk              (CLK           ),
        .adr_i            (UART_ADR_I    ),
        .dat_i            (UART_DAT_I    ),
        .dat_o            (UART_DAT_O    ),
        .stb_i            (UART_STB_I    ),
        .cyc_i            (UART_CYC_I    ),
        .we_i             (UART_WE_I     ),
        .sel_i            (UART_SEL_I    ),
        .bte_i            (UART_BTE_I    ),
        .ack_o            (UART_ACK_O    ),
        .intr             (intr          ),
        .rbr              (RBR           ),
		.rbr_fifo         (RBR_FIFO      ),
        .thr              (THR           ),
        .rbr_rd           (rbr_rd        ),
        .thr_wr           (thr_wr        ),
        .lsr_rd           (lsr_rd        ),
        .databits         (databits      ),
        .stopbits         (stopbits      ),
        .parity_en        (parity_en     ),
        .parity_even      (parity_even   ),
        .parity_stick     (parity_stick  ),
        .tx_break         (tx_break      ),
        .rx_rdy           (rx_rdy        ),
        .overrun_err      (overrun_err   ),
        .parity_err       (parity_err    ),
        .frame_err        (frame_err     ),
        .break_int        (break_int     ),
        .thre             (THRE          ),
        .temt             (TEMT          ),
	.fifo_empty       (fifo_empty  ),
	.fifo_empty_thr   (fifo_empty_thr),
	.thr_rd           (thr_rd),
	.fifo_almost_full (fifo_almost_full),
        .divisor          (divisor)	
  );
`endif
   
   
  rxcver #(.DATAWIDTH(DATAWIDTH),
           .FIFO(FIFO)) 
     u_rxcver (
        .reset            (RESET         ),
        .clk              (CLK          ),
        .rbr              (RBR           ),
	.rbr_fifo         (RBR_FIFO      ),
        .rbr_rd           (rbr_rd        ),
        .lsr_rd           (lsr_rd        ),
        .sin              (SIN           ),
        .databits         (databits      ),   
        .parity_en        (parity_en     ),
        .parity_even      (parity_even   ),
        .parity_stick     (parity_stick  ),
        .rx_rdy           (rx_rdy        ),
        .overrun_err      (overrun_err   ),
        .parity_err       (parity_err    ),
        .frame_err        (frame_err     ),
        .break_int        (break_int     ),
	.fifo_empty       (fifo_empty    ),
	.fifo_almost_full (fifo_almost_full),
	.divisor          (divisor       )
  ); 
  
  txmitt #(.DATAWIDTH(DATAWIDTH),
           .FIFO(FIFO)) 
      u_txmitt (
        .reset          (RESET         ),
        .clk            (CLK           ),
        .thr            (THR           ),
        .thr_wr         (thr_wr        ),
        .sout           (sout          ),
        .databits       (databits      ),
        .stopbits       (stopbits      ),
        .parity_en      (parity_en     ),
        .parity_even    (parity_even   ),
        .parity_stick   (parity_stick  ),
        .tx_break       (tx_break      ),
        .thre           (THRE          ),
        .temt           (TEMT          ),
	.fifo_empty_thr (fifo_empty_thr),
	.thr_rd         (thr_rd),
	.divisor        (divisor)
  );
  
 `ifdef MODEM
  modem #(.DATAWIDTH(DATAWIDTH)) u_modem 
        ( 
        .reset        (RESET         ), 
        .clk          (CLK           ), 
        .msr          (MSR           ), 
        .mcr          (MCR           ),
        .msr_rd       (msr_rd        ), 
        .dcd_n        (DCD_N         ), 
        .cts_n        (CTS_N         ), 
        .dsr_n        (DSR_N         ), 
        .ri_n         (RI_N          ), 
        .dtr_n        (dtr_n         ), 
        .rts_n        (rts_n         )  
  );
  `endif

  // TXRDY_N, RXRDY_N is low active output
  assign #5 TXRDY_N = ~THRE;
  assign #5 RXRDY_N = ~rx_rdy; 
  assign #5 INTR    = intr;
  assign #5 SOUT    = sout;
  `ifdef MODEM
  assign #5 DTR_N   = dtr_n;
  assign #5 RTS_N   = rts_n;
  `endif 
endmodule
`endif // UART_CORE_FILE
