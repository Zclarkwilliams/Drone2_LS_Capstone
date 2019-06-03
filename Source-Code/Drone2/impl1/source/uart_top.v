/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell,
 * Brett Creeley,
 * Daniel Christiansen,
 * Kirk Hooper,
 * Zachary Clark-Williams
 */

/**
 * drone2 - Top level module for the debug UART.
 This version takes 1098 LUTs - Dec 25, 2018
 *
  */
`timescale 1ns / 1ns
`include "common_defines.v"

module uart_top
(
	input wire resetn,
	input wire clk,
	input wire start,
	input wire sin,
	output wire rxrdy_n,
	output wire sout,
	output wire txrdy_n,

	input wire [`IMU_VAL_BIT_WIDTH-1:0]
		imu_x_rotation_angle,
		imu_y_rotation_angle,
		imu_z_rotation_angle,
		imu_x_rotation_rate,
		imu_y_rotation_rate,
		imu_z_rotation_rate,
	input wire [`REC_VAL_BIT_WIDTH-1:0]
		imu_calibration_status,
		rec_throttle_val,
		rec_yaw_val,
		rec_roll_val,
		rec_pitch_val,
		rec_aux1_val,
		rec_aux2_val,
        //Changed size to 16 bits to debug additional inputs (z_linear_accel)
	input wire [`RATE_BIT_WIDTH-1:0]
		rec_swa_swb_val,
	input wire [`RATE_BIT_WIDTH-1:0]
		yaac_yaw_angle_error,
		yaac_yaw_angle_target,
		amc_z_linear_velocity,
		debug_17_in_16_bits,
		debug_18_in_16_bits
);

	localparam CLK_IN_MHZ = 38;
	localparam BAUD_RATE  = 115200;
	localparam ADDRWIDTH  = 3;
	localparam DATAWIDTH  = 8;
	localparam FIFO       = 0;//            --> To enable FIFO option, change this value to 1

	
    localparam RBR = 4'b1000;//08
    localparam THR = 4'b1000;//08
    localparam IER = 4'b0001;//01
    localparam IIR = 4'b0010;//02
    localparam LCR = 4'b0011;//03
    localparam LSR = 4'b0101;//05
    localparam DIV = 4'b0111;//07 
	
	localparam [9:0] INIT0        = 10'h1<<0, //0x001
					 INIT_IER     = 10'h1<<1, //0x002
					 INIT_LCR     = 10'h1<<2, //0x004
					 WAIT         = 10'h1<<3, //0x008
					 TX_INC       = 10'h1<<4, //0x010
					 TX_SET       = 10'h1<<5, //0x020
					 TX_ACK       = 10'h1<<6, //0x040
					 TX_WAIT      = 10'h1<<7, //0x080
					 TX_NEXT_WORD = 10'h1<<8, //0x100
					 TX_STOP      = 10'h1<<9; //0x200
					 

	
	localparam [3:0]TX_BYTE_INDEX_MAX = 4'd8;
	localparam [7:0]TX_WORD_INDEX_MAX = 8'd19;

	reg         stb_i;
	reg         next_cyc_i, cyc_i;
	reg  [15:0] next_dat_i, dat_i;
	reg  [7:0]  next_adr_i, adr_i;
	reg  [3:0]  next_sel_i, sel_i;
	reg  [2:0]  next_cti_i, cti_i;
	reg  [1:0]  next_bte_i, bte_i;
	reg         next_we_i , we_i ;
	reg  [9:0]	next_state, state;
	wire [15:0] dat_o;     
	wire        ack_o;
	wire        intr;
	reg [3:0]	tx_byte_index, next_tx_byte_index;
	reg [7:0]	tx_word_index, next_tx_word_index;
	wire rts_bypass;
	wire dtr_bypass;
	
	
	uart_core
	
      #(.CLK_IN_MHZ(CLK_IN_MHZ),
        .BAUD_RATE(BAUD_RATE),
        .ADDRWIDTH(ADDRWIDTH),
        .DATAWIDTH(DATAWIDTH),
        .FIFO(FIFO)
       )	
	
	uart_core_inst(
       // Global reset and clock
       .RESET    (~resetn ),
       .CLK      (clk),
      
      // wishbone interface
       .UART_ADR_I   (adr_i ),
       .UART_DAT_I   (dat_i ),
       .UART_STB_I   (stb_i ),
       .UART_CYC_I   (cyc_i ),
       .UART_WE_I    (we_i  ),
       .UART_SEL_I   (sel_i ),
       .UART_BTE_I   (bte_i ),
       .UART_DAT_O   (dat_o ),
       .UART_ACK_O   (ack_o ),      
       .INTR         (intr  ),
	   .UART_CTI_I   (cti_i),

      `ifdef MODEM
       // Modem interface
	   
	   //Handshaking signals
       //.DCD_N    (dcd_n ),
       //.CTS_N    (cts_n ),
       //.DSR_N    (dsr_n ),
       //.RI_N     (ri_n  ),
       //.DTR_N    (dtr_n ),
       //.RTS_N    (rts_n ),
	   
	   //Bypass handshaking signals
       .DCD_N    (dtr_bypass ),
       .CTS_N    (rts_bypass ),
       .DSR_N    (dtr_bypass ),
       .RI_N     (1'b1  ),
       .DTR_N    (dtr_bypass ),
       .RTS_N    (rts_bypass ),
      `endif
      
       // Receiver interface
       .SIN      (sin    ),
       .RXRDY_N  (rxrdy_n),
       
       // Transmitter interface
       .SOUT     (sout   ),
       .TXRDY_N  (txrdy_n)
      
    );

 	// Advance state and registered data at each positive clock edge
	always@(posedge clk, negedge resetn) begin
		if(~resetn) begin
			cyc_i <= 1'h0;
			stb_i <= 1'h0;
			dat_i <= 16'h0000;
			adr_i <= 8'h00;
			sel_i <= 4'h0;
			cti_i <= 3'h0;
			bte_i <= 2'h0;
			we_i  <= 1'h0;
			state <= 8'h00;
			tx_byte_index <= 4'd0;
			tx_word_index <= 8'd0;
		end
		else begin
			cyc_i <= next_cyc_i;
			stb_i <= next_cyc_i;
			dat_i <= next_dat_i;
			adr_i <= next_adr_i;
			sel_i <= next_sel_i;
			cti_i <= next_cti_i;
			bte_i <= next_bte_i;
			we_i  <= next_we_i ;
			state <= next_state;
			tx_byte_index <= next_tx_byte_index;
			tx_word_index <= next_tx_word_index;
		end
	end
	
	always@(*) begin
		if(~resetn) begin
			next_state = INIT0;
		end
		else begin
			case(state)
				INIT0:	 		next_state = INIT_IER;                                                                              //0x001
				INIT_IER:		next_state = (ack_o)              ? INIT_LCR : INIT_IER;                                            //0x002
				INIT_LCR:		next_state = (ack_o)              ? WAIT     : INIT_LCR;                                            //0x004
				WAIT:			next_state = (start  && ~txrdy_n) ? TX_SET   : WAIT;                                                //0x008
				TX_INC:	 		next_state = TX_SET;                                                                                //0x010
				TX_SET:	 		next_state = (ack_o)              ? TX_ACK   : TX_SET;                                              //0x020
				TX_ACK:	 		next_state = (txrdy_n)            ? TX_WAIT  : TX_ACK;                                              //0x040
				TX_WAIT: 		next_state = (~txrdy_n) ? ((tx_byte_index == TX_BYTE_INDEX_MAX) ? TX_NEXT_WORD : TX_INC) : TX_WAIT; //0x080
				TX_NEXT_WORD:	next_state = (tx_word_index == TX_WORD_INDEX_MAX) ? TX_STOP : TX_SET;                               //0x100
				TX_STOP: 		next_state = WAIT;                                                                                  //0x200
				default:        next_state = INIT0;
			endcase
		end
	end
	
	function automatic [15:0] hex_convert;
		input reg [3:0]tx_nibble;
		begin
			case(tx_nibble)
				4'd0:    hex_convert = {9'd0, "0"};
				4'd1:    hex_convert = {9'd0, "1"};
				4'd2:    hex_convert = {9'd0, "2"};
				4'd3:    hex_convert = {9'd0, "3"};
				4'd4:    hex_convert = {9'd0, "4"};
				4'd5:    hex_convert = {9'd0, "5"};
				4'd6:    hex_convert = {9'd0, "6"};
				4'd7:    hex_convert = {9'd0, "7"};
				4'd8:    hex_convert = {9'd0, "8"};
				4'd9:    hex_convert = {9'd0, "9"};
				4'd10:   hex_convert = {9'd0, "A"};
				4'd11:   hex_convert = {9'd0, "B"};
				4'd12:   hex_convert = {9'd0, "C"};
				4'd13:   hex_convert = {9'd0, "D"};
				4'd14:   hex_convert = {9'd0, "E"};
				4'd15:   hex_convert = {9'd0, "F"};
				4'd16:   hex_convert = {1'b0, "1",1'b0, "0"};
                default: hex_convert = {16'd0};
			endcase
		end
	endfunction
	
	
	task transmit_word;
		input reg[7:0] tx_word_index;
		input reg[3:0] tx_byte_index;
		input reg[15:0] tx_word;
		begin
			case(tx_byte_index)
				4'd0:    next_dat_i = hex_convert(tx_word_index[7:4]);
				4'd1:    next_dat_i = hex_convert(tx_word_index[3:0]);
				4'd2:    next_dat_i = hex_convert(tx_word[15:12]);
				4'd3:    next_dat_i = hex_convert(tx_word[11: 8]);
				4'd4:    next_dat_i = hex_convert(tx_word[ 7: 4]);
				4'd5:    next_dat_i = hex_convert(tx_word[ 3: 0]);
				4'd6:    next_dat_i = {9'b0, "\n"};
				4'd7:    next_dat_i = {9'b0, "\r"};
                default: next_dat_i = 16'b0;
			endcase
		end
	endtask
	
	always@(*) begin
		if(~resetn) begin
			next_cyc_i = 1'h0;
			next_dat_i = 16'h0000;
			next_adr_i = 8'h00;
			next_sel_i = 4'h0;
			next_cti_i = 3'h0;
			next_bte_i = 2'h0;
			next_we_i  = 1'h0;
			next_tx_byte_index = 4'd0;
			next_tx_word_index = 8'd0;
		end
		else begin
			next_sel_i = 4'h0;
			next_bte_i = 2'h0; 
			next_cti_i = 3'h0;
			next_dat_i = dat_i;
			next_tx_byte_index = tx_byte_index;
			next_tx_word_index = tx_word_index;
			case(state)
				INIT0: begin
					next_cyc_i = 1'h0;
					next_dat_i = 16'h0000;
					next_adr_i = 8'h00;
					next_we_i  = 1'h0;
				end
				INIT_IER: begin
					next_cyc_i = ack_o? 1'h0 : 1'h1;
					next_dat_i = 16'h0000;
					next_adr_i = IER;
					next_we_i  = 1'h1;
				end
				INIT_LCR: begin
					next_cyc_i = ack_o? 1'h0 : 1'h1;
					next_dat_i = 16'h0003; //8-bit word, 1 stop bit, no parity, no sticky parity, no Tx break assertion
					next_adr_i = LCR;
					next_we_i  = 1'h1;
				end
				WAIT: begin
					next_cyc_i = 1'h0;
					next_dat_i = 16'h0000;
					next_adr_i = 8'hff;
					next_we_i  = 1'h1;
				end
				TX_INC: begin
					next_cyc_i = 1'h0;
					next_adr_i = THR;
					next_we_i  = 1'h1;
					next_tx_byte_index = tx_byte_index + 4'd1;
				end
				TX_SET: begin
					next_cyc_i = ack_o ? 1'h0 : 1'h1;
					next_adr_i = THR;
					next_we_i  = 1'h1;
					case(tx_word_index)
						8'd0:  transmit_word(tx_word_index, tx_byte_index, imu_x_rotation_angle);
						8'd1:  transmit_word(tx_word_index, tx_byte_index, imu_y_rotation_angle);
						8'd2:  transmit_word(tx_word_index, tx_byte_index, imu_z_rotation_angle);
						8'd3:  transmit_word(tx_word_index, tx_byte_index, imu_x_rotation_rate);
						8'd4:  transmit_word(tx_word_index, tx_byte_index, imu_y_rotation_rate);
						8'd5:  transmit_word(tx_word_index, tx_byte_index, imu_z_rotation_rate);
						8'd6:  transmit_word(tx_word_index, tx_byte_index, imu_calibration_status);
						8'd7:  transmit_word(tx_word_index, tx_byte_index, {8'd0, rec_throttle_val});
						8'd8:  transmit_word(tx_word_index, tx_byte_index, {8'd0, rec_yaw_val});
						8'd9:  transmit_word(tx_word_index, tx_byte_index, {8'd0, rec_roll_val});
						8'd10: transmit_word(tx_word_index, tx_byte_index, {8'd0, rec_pitch_val});
						8'd11: transmit_word(tx_word_index, tx_byte_index, {8'd0, rec_aux1_val});
						8'd12: transmit_word(tx_word_index, tx_byte_index, {8'd0, rec_aux2_val});
						//8'd13: transmit_word(tx_word_index, tx_byte_index, {8'd0, rec_swa_swb_val});
                        //Changed size to 16 bits to debug additional bits for (z_linear_accel)
						8'd13: transmit_word(tx_word_index, tx_byte_index, rec_swa_swb_val);
						8'd14: transmit_word(tx_word_index, tx_byte_index, yaac_yaw_angle_error);
						8'd15: transmit_word(tx_word_index, tx_byte_index, yaac_yaw_angle_target);
						8'd16: transmit_word(tx_word_index, tx_byte_index, amc_z_linear_velocity);
						8'd17: transmit_word(tx_word_index, tx_byte_index, debug_17_in_16_bits);
						8'd18: transmit_word(tx_word_index, tx_byte_index, debug_18_in_16_bits);
						default:  next_dat_i = 16'd0;
					endcase
				end
				TX_ACK: begin
					next_cyc_i = 1'h0;
					next_adr_i = THR;
					next_we_i  = 1'h1;
				end
				TX_WAIT: begin
					next_cyc_i = 1'h0;
					next_adr_i = THR;
					next_we_i  = 1'h1;
				end
				TX_NEXT_WORD: begin
					next_cyc_i = 1'h0;
					next_adr_i = THR;
					next_we_i  = 1'h1;
					next_tx_byte_index = 4'd0;
					next_tx_word_index = tx_word_index + 8'd1;
				end
				TX_STOP: begin
			        next_cyc_i = 1'h0;
			        next_dat_i = 16'h0000;
			        next_adr_i = 8'h00;
			        next_sel_i = 4'h0;
			        next_cti_i = 3'h0;
			        next_bte_i = 2'h0;
					next_we_i  = 1'h1;
			        next_tx_byte_index = 4'd0;
			        next_tx_word_index = 8'd0;
                end
				default: begin
			        next_cyc_i = 1'h0;
			        next_dat_i = 16'h0000;
			        next_adr_i = 8'h00;
			        next_sel_i = 4'h0;
			        next_cti_i = 3'h0;
			        next_bte_i = 2'h0;
					next_we_i  = 1'h1;
			        next_tx_byte_index = 4'd0;
			        next_tx_word_index = 8'd0;
				end
			endcase
		end
	end
endmodule