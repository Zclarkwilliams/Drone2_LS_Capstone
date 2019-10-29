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

module i2c_slave_model (resetn, scl, sda);

	//
	// parameters
	//
	parameter I2C_ADR = 7'b001_0000;

	//
	// input && outpus
	//
    input wire resetn;
	input wire scl;
	inout wire sda;

	//
	// Variable declaration
	//
	reg debug;

	reg [7:0] mem [0:254]; // initiate memory
	reg [7:0] mem_adr;     // memory address
	reg [7:0] mem_do;      // memory data output

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
	

	// statemachine declaration
	parameter idle        = 3'b000;
	parameter slave_ack   = 3'b001;
	parameter get_mem_adr = 3'b010;
	parameter gma_ack     = 3'b011;
	parameter data        = 3'b100;
	parameter data_ack    = 3'b101;

	reg [2:0] state; // synopsys enum_state
    
    

	//
	// module body
	//

	initial
	begin
	   sda_o = 1'b1;
	   state = idle;
	   $readmemh("/../../testing_files/i2c_sim/i2c_slave_mem_init.txt", mem);

	end

	// generate shift register
	always @(posedge scl)
	  sr <= #1 {sr[6:0],sda};

	//detect my_address
	assign my_adr = (sr[7:1] == I2C_ADR);
	// FIXME: This should not be a generic assign, but rather
	// qualified on address transfer phase and probably reset by stop
    
    
    // Only debug if time is non-zero (Not at initialization)
    initial begin
      debug = 1'b0;
      #1;
      //debug = 1'b1;
      debug = 1'b0;
    end
    
    
    always @(debug)
      if(~debug)
        $display("%t i2c_slave 0x%h; DEBUG disabled", $time, I2C_ADR);
      else
        $display("%t i2c_slave 0x%h; DEBUG enabled", $time, I2C_ADR);


	//generate bit-counter
	always @(posedge scl)
	  if(ld)
	    bit_cnt <= #1 3'b111;
	  else
	    bit_cnt <= #1 bit_cnt - 3'h1;

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
	always @(negedge sda)
	  if(scl)
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
	always @(posedge sda)
	  if(scl)
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
	      get_mem_adr:  $display("%t i2c_slave 0x%h; STATE=GET_MEM_ADDR", $time, I2C_ADR);
	      gma_ack:      $display("%t i2c_slave 0x%h; STATE=SLAVE_ACK", $time, I2C_ADR);
	      data:         $display("%t i2c_slave 0x%h; STATE=DATA", $time, I2C_ADR);
	      data_ack:     $display("%t i2c_slave 0x%h; STATE=DATA_ACK", $time, I2C_ADR);
	    endcase
      end
	end


	// generate statemachine
	always @(negedge scl or posedge sto)
	  if (sto || (sta && !d_sta) )
	    begin
	        state <= #1 idle; // reset statemachine

	        sda_o <= #1 1'b1;
	        ld    <= #1 1'b1;
	    end
	  else
	    begin
	        // initial settings
	        sda_o <= #1 1'b1;
	        ld    <= #1 1'b0;

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

	                    if(rw)
	                      begin
	                          mem_do <= #1 mem[mem_adr];

	                          if(debug)
	                            begin
	                                #2 $display("%t i2c_slave 0x%h; data block read %x from address %x (1)", $time, I2C_ADR, mem_do, mem_adr);
	                                #2 $display("%t i2c_slave 0x%h; memcheck [0]=%x, [1]=%x, [2]=%x, [3]=%x", $time, I2C_ADR, mem[4'h0], mem[4'h1], mem[4'h2], mem[4'h3]);
	                            end
	                      end
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
	                    state <= #1 get_mem_adr;

	                  ld    <= #1 1'b1;
	              end

	            get_mem_adr: // wait for memory address
	              begin
	              if(acc_done)
	                begin
	                    state <= #1 gma_ack;
	                    mem_adr <= #1 sr; // store memory address
	                    sda_o <= #1 1'b0; // generate i2c_ack, for valid address

	                    if(debug)
	                      #1 $display("%t i2c_slave 0x%h; address received. adr=%x, ack=%b", $time, I2C_ADR, sr, sda_o);
	                end
                  end
	            gma_ack:
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
	                              #3 mem_do <= mem[mem_adr];

	                              if(debug)
	                                #5 $display("%t i2c_slave 0x%h; data block read %x from address %x (2)", $time, I2C_ADR, mem_do, mem_adr);
	                          end

	                        if(!rw)
	                          begin
	                              mem[ mem_adr[3:0] ] <= #1 sr; // store data in memory

	                              if(debug)
	                                #2 $display("%t i2c_slave 0x%h; data block write %x to address %x", $time, I2C_ADR, sr, mem_adr);
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
	  specparam normal_scl_low  = 500, //4700, 
	            normal_scl_high = 500, //4000,
	            normal_tsu_sta  = 500, //4700, change to relax simulation
	            normal_thd_sta  = 500, //4000,
	            normal_tsu_sto  = 500, //4000,
	            normal_tbuf     = 500, //4700,    
	            

	            fast_scl_low  =  600,
	            fast_scl_high =  600,
	            fast_tsu_sta  =  600,
	            fast_thd_sta  =  600,
	            fast_tsu_sto  =  600,
	            fast_tbuf     =  600;

	  // $width(negedge scl, normal_scl_low);  // scl low time
	  // $width(posedge scl, normal_scl_high); // scl high time

	  // $setup(posedge scl, negedge sda &&& scl, normal_tsu_sta); // setup start
	  // $setup(negedge sda &&& scl, negedge scl, normal_thd_sta); // hold start
	  // $setup(posedge scl, posedge sda &&& scl, normal_tsu_sto); // setup stop

	  // $setup(posedge tst_sta, posedge tst_sto, normal_tbuf); // stop to start time

	  $width(negedge scl, fast_scl_low);  // scl low time
	  $width(posedge scl, fast_scl_high); // scl high time

	  $setup(posedge scl, negedge sda &&& scl, fast_tsu_sta); // setup start
	  $setup(negedge sda &&& scl, negedge scl, fast_thd_sta); // hold start
	  $setup(posedge scl, posedge sda &&& scl, fast_tsu_sto); // setup stop

	  $setup(posedge tst_sta, posedge tst_sto, fast_tbuf); // stop to start time
	endspecify

endmodule


