/////////////////////////////////////////////////////////////////////
////                                                             ////
////  WISHBONE rev.B2 compliant synthesizable I2C Slave model    ////
////                                                             ////
////                                                             ////
////  Authors: Richard Herveille (richard@asics.ws) www.asics.ws ////
////           John Sheahan (jrsheahan@optushome.com.au)         ////
////                                                             ////
////  Downloaded from: http://www.opencores.org/projects/i2c/    ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
////                                                             ////
//// Copyright (C) 2001,2002 Richard Herveille                   ////
////                         richard@asics.ws                    ////
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
//               Revision 1.6  2005/02/28 11:33:48  rherveille
//               Fixed Tsu:sta timing check.
//               Added Thd:sta timing check.
//
//               Revision 1.5  2003/12/05 11:05:19  rherveille
//               Fixed slave address MSB='1' bug
//
//               Revision 1.4  2003/09/11 08:25:37  rherveille
//               Fixed a bug in the timing section. Changed 'tst_scl' into 'tst_sto'.
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

`timescale 1ns / 1ns

module i2c_slave_model_2B_reg (resetn, scl, sda);

	//
	// parameters
	//
	parameter I2C_ADR = 7'b001_0000;

	//
	// input && outputs
	//
	input wire resetn;
	input wire scl;
	inout wire sda;

	//
	// Variable declaration
	//
	reg debug;

	reg [15:0] mem [0:65534]; // initiate memory 2 byte address (16 bit)
	reg [15:0] mem_adr;       // memory address
	reg [7:0]  mem_do;        // memory data output

	reg sta, d_sta;
	reg sto, d_sto;

	reg [7:0] sr;        // 8bit shift register
	reg       rw;        // read/write direction

	wire      my_adr;    // my address called ??
	wire      i2c_reset; // i2c-statemachine reset
	reg [2:0] bit_cnt;   // 3bit downcounter
	wire      acc_done;  // 8bits transfered
	reg       ld;        // load downcounter

	reg       sda_o;     // sda-drive level
	wire      sda_dly;   // delayed version of sda
    reg       is_2nd_reg_addr_byte;
	

	// statemachine declaration
	parameter idle         = 3'b000;
	parameter slave_ack    = 3'b001;
	parameter get_mem_adr1 = 3'b010;
	parameter gma_ack1     = 3'b011;
	parameter get_mem_adr2 = 3'b100;
	parameter gma_ack2     = 3'b101;
	parameter data         = 3'b110;
	parameter data_ack     = 3'b111;

	reg [2:0] state; // synopsys enum_state

	//
	// module body
	//

	initial
	begin
	   sda_o = 1'b1;
	   state = idle;
	   $readmemh("/../../testing_files/i2c_sim/i2c_slave_mem_init_2B_reg_2B_words.txt", mem);
	end

	// generate shift register
	always @(posedge scl) begin
	  sr <= #1 {sr[6:0],sda};
    end

	//detect my_address
	assign my_adr = (sr[7:1] == I2C_ADR);
	// FIXME: This should not be a generic assign, but rather
	// qualified on address transfer phase and probably reset by stop

    
    // Only debug if time is non-zero (Not at initialization)
    initial begin
      debug = 1'b0;
      #1;
      debug = 1'b1;
    end
    
    
    always @(debug)
      if(~debug)
        $display("%t i2c_slave 0x%h; DEBUG disabled", $time, I2C_ADR);
      else
        $display("%t i2c_slave 0x%h; DEBUG enabled", $time, I2C_ADR);


	//generate bit-counter
	always @(posedge scl, negedge resetn) begin
      if(~resetn || ld)
	    bit_cnt <= #1 3'b111;
	  else
	    bit_cnt <= #1 bit_cnt - 3'h1;
    end

	//generate access done signal
	assign acc_done = !(|bit_cnt);

	// generate delayed version of sda
	// this model assumes a hold time for sda after the falling edge of scl.
	// According to the Phillips i2c spec, there s/b a 0 ns hold time for sda
	// with regards to scl. If the data changes coincident with the clock, the
	// acknowledge is missed
	// Fix by Michael Sosnoski
	assign #1 sda_dly = sda;


	//detect start condition
	always @(negedge sda, negedge resetn)
      if(~resetn) begin
	    sta   <= #1 1'b0;
        d_sta <= #1 1'b0;
        sto   <= #1 1'b0;
      end
      else if(scl)
	    begin
	        sta   <= #1 1'b1;
            d_sta <= #1 1'b0;
            sto   <= #1 1'b0;

	        if(debug)
	          $display("%t i2c_slave 0x%h; start condition detected", $time, I2C_ADR);
	    end
	  else
	    sta <= #1 1'b0;

	always @(posedge scl)
	  d_sta <= #1 sta;

	// detect stop condition
	always @(posedge sda, negedge resetn)
      if(~resetn) begin
	    sta <= #1 1'b0;
	    sto <= #1 1'b0;
      end
      else if(scl)
	    begin
	       sta <= #1 1'b0;
	       sto <= #1 1'b1;

	       if(debug)
	         $display("%t i2c_slave 0x%h; stop condition detected", $time, I2C_ADR);
	    end
	  else
	    sto <= #1 1'b0;

	//generate i2c_reset signal
	assign i2c_reset = sta || sto;
    
    
    // Print debugs of current state
	always @(state) begin
      if(debug) begin
	    case(state)
	      idle:         $display("%t i2c_slave 0x%h; STATE=IDLE", $time, I2C_ADR);
	      slave_ack:    $display("%t i2c_slave 0x%h; STATE=SLAVE_ACK", $time, I2C_ADR);
	      get_mem_adr1: $display("%t i2c_slave 0x%h; STATE=GET_MEM_ADDR first byte", $time, I2C_ADR);
	      gma_ack1:     $display("%t i2c_slave 0x%h; STATE=SLAVE_ACK first byte", $time, I2C_ADR);
	      get_mem_adr2: $display("%t i2c_slave 0x%h; STATE=GET_MEM_ADDR second byte", $time, I2C_ADR);
	      gma_ack2:     $display("%t i2c_slave 0x%h; STATE=GMA_ACK second byte", $time, I2C_ADR);
	      data:         $display("%t i2c_slave 0x%h; STATE=DATA", $time, I2C_ADR);
	      data_ack:     $display("%t i2c_slave 0x%h; STATE=DATA_ACK", $time, I2C_ADR);
	    endcase
      end
	end


	// generate statemachine
	always @(negedge scl, posedge sto, negedge resetn)
	  if (~resetn) begin
	        state   <= #1 idle;
	        sda_o   <= #1 1'b1;
	        ld      <= #1 1'b0;
            is_2nd_reg_addr_byte <= 1'b0;
            mem_adr <= 'd0;
            mem_do  <= 'd0;
            rw      <= 1'b0;
      end
      else if (sto || (sta && !d_sta) )
	    begin
	        state <= #1 idle; // reset statemachine

	        sda_o <= #1 1'b1;
	        ld    <= #1 1'b1;
            is_2nd_reg_addr_byte <= 1'b0;
	    end
	  else
	    begin
	        // initial settings
	        sda_o <= #1 1'b1;
	        ld    <= #1 1'b0;
            is_2nd_reg_addr_byte <= 1'b0;

	        case(state) // synopsys full_case parallel_case
	            idle: // idle state
                  begin
	              if (acc_done && my_adr)
	                begin
	                    state <= #1 slave_ack;
	                    rw <= #1 sr[0];
	                    sda_o <= #1 1'b0; // generate i2c_ack

	                    #2;
	                    if(debug && rw)
	                      $display("%t i2c_slave 0x%h; command byte received (read) ", $time, I2C_ADR);
	                    if(debug && !rw)
	                      $display("%t i2c_slave 0x%h; command byte received (write)", $time, I2C_ADR);

	                    if(rw && is_2nd_reg_addr_byte)
	                      begin
	                          mem_do <= #1 mem_adr[0]? (((mem[mem_adr]) & 16'hFF00)>>8) : ((mem[mem_adr]) & 16'hFF00) ;

	                          if(debug)
	                            begin
	                                #2 $display("%t i2c_slave 0x%h; data block read 0x%h from address 0x%h (1) in idle state", $time, I2C_ADR, mem_do, mem_adr);
	                            end
	                      end
                        //mem_adr <= 16'd0;
                        //$stop;
	                end
                  end
	            slave_ack:
	              begin
	                  if(rw)
	                    begin
	                        state <= #1 data;
	                        sda_o <= #1 mem_do[7];
	                    end
	                  else
	                    state <= #1 get_mem_adr1;

	                  ld    <= #1 1'b1;
	              end

	            get_mem_adr1: // wait for memory address
	              begin
	              if(acc_done)
	                begin
                        is_2nd_reg_addr_byte <= 1'b0;
	                    state <= #1 gma_ack1;
	                    mem_adr <= #1 {8'h00, sr}; // store memory address
	                    sda_o <= #1 1'b0;          // generate i2c_ack, for valid address

	                    if(debug)
	                      #5 $display("%t i2c_slave 0x%h; address received. adr=0x%h, ack=%b", $time, I2C_ADR, mem_adr, sda_o);
	                end
                  end
	            gma_ack1:
	              begin
	                  state <= #1 get_mem_adr2;
	                  ld    <= #1 1'b1;
	              end

	            get_mem_adr2: // wait for memory address
	              begin
	              if(acc_done)
	                begin
                        is_2nd_reg_addr_byte <= 1'b1;
	                    state <= #1 gma_ack2;
	                    mem_adr <= #1 {mem_adr[7:0], sr};// store memory address
	                    sda_o <= #1 1'b0;                // generate i2c_ack, for valid address

	                    if(debug)
	                      #5 $display("%t i2c_slave 0x%h; address received. adr=0x%h, ack=%b", $time, I2C_ADR, mem_adr, sda_o);
	                end
                  end
	            gma_ack2:
	              begin
	                  state <= #1 data;
	                  ld    <= #1 1'b1;
	              end

	            data: // receive or drive data
	              begin
	                  if(rw)
	                    sda_o <= #1 mem_do[7];

	                  if(acc_done)
	                    begin
	                        state   <= #1 data_ack;
	                        mem_adr <= #2 mem_adr + 8'h1;
	                        sda_o   <= #1 (rw); // send ack on write, receive ack on read

	                        if(rw)
	                          begin
                                  $display("@0 Memory read word 0x%h", mem[mem_adr]);
                                  $display("@0 mem_adr                           0x%h", mem_adr);
                                  $display("@0 mem_adr[0]                        0x%h", mem_adr[0]);
                                  $display("@0 (((mem[mem_adr]) & 16'hFF00)>>8)  0x%h", (((mem[mem_adr]) & 16'hFF00)>>8) );
                                  $display("@0 ((mem[mem_adr]) & 16'hFF00)       0x%h", ((mem[mem_adr]) & 16'h00FF));
	                              #3 mem_do <= #1 mem_adr[0]? (((mem[mem_adr]) & 16'hFF00)>>8) : ((mem[mem_adr]) & 16'h00FF) ;
                                  #3 $display("@3 Memory read word 0x%h", mem[mem_adr]);
                                  //$stop;
	                              if(debug)
	                                #5 $display("%t i2c_slave 0x%h; data block read 0x%h from address 0x%h (2) in data state", $time, I2C_ADR, mem_do, mem_adr);
	                          end

	                        if(!rw)
	                          begin
	                              mem[mem_adr] <= #1 mem_adr[0] ? {sr, ((mem[mem_adr]) & 16'hFF00)} : {(((mem[mem_adr]) & 16'hFF00)>>8), sr} ; // store data in memory
                                  //$stop;

	                              if(debug)
	                                #2 $display("%t i2c_slave 0x%h; data block write 0x%h to address 0x%h in data state", $time, I2C_ADR, sr, mem_adr);
	                          end
	                    end
	              end

	            data_ack:
	              begin
	                  ld <= #1 1'b1;

	                  if(rw)
	                    if(sr[0]) // read operation && master send NACK
	                      begin
	                          state <= #1 idle;
	                          sda_o <= #1 1'b1;
	                      end
	                    else
	                      begin
	                          state <= #1 data;
	                          sda_o <= #1 mem_do[7];
	                      end
	                  else
	                    begin
	                        state <= #1 data;
	                        sda_o <= #1 1'b1;
	                    end
	              end

	        endcase
	    end

	// read data from memory
	always @(posedge scl)
	  if(!acc_done && rw)
	    mem_do <= #1 {mem_do[6:0], 1'b1}; // insert 1'b1 for host ack generation

	// generate tri-states
	assign sda = sda_o ? 1'bz : 1'b0;


	//
	// Timing checks
	//

	wire tst_sto = sto;
	wire tst_sta = sta;

	specify
	  specparam normal_scl_low  = 4700, 
	            normal_scl_high = 4000,
	            normal_tsu_sta  = 2000, //4700, change to relax simulation
	            normal_thd_sta  = 2000, //4000,
	            normal_tsu_sto  = 2000, //4000,
	            normal_tbuf     = 4700,    
	            

	            fast_scl_low  = 1300,
	            fast_scl_high =  600,
	            fast_tsu_sta  = 1300,
	            fast_thd_sta  =  600,
	            fast_tsu_sto  =  600,
	            fast_tbuf     = 1300;

	  $width(negedge scl, normal_scl_low);  // scl low time
	  $width(posedge scl, normal_scl_high); // scl high time

	  $setup(posedge scl, negedge sda &&& scl, normal_tsu_sta); // setup start
	  $setup(negedge sda &&& scl, negedge scl, normal_thd_sta); // hold start
	  $setup(posedge scl, posedge sda &&& scl, normal_tsu_sto); // setup stop

	  $setup(posedge tst_sta, posedge tst_sto, normal_tbuf); // stop to start time
	endspecify

endmodule


