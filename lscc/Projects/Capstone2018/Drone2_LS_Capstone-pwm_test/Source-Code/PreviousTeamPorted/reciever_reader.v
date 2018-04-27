/*

Code from ECE412/3 Capstone team from 2017, Lucas Myers,  Svyatoslav Zhuchenya, Casey Montgomery, Charley  Hill
File downloaded from: https://github.com/lucas709/drone_soc/blob/master/verilog_src/reciever_reader.v

*/

//---------------------------------------------------------------------------
// ECE 412-413 Capstone Winter/Spring 2017
// Team 05 Drone SOC
// Lucas Myers, Svyatoslav Zhuchenya, Casey Montgomery, Charley Hill
//
// with a period of 50 hz and a max 10% duty cycle the max period high will be 2ms
// part that will be fluctuating will fluctuate only over 5 % or by a max variation of 1ms
// I recommend to get a resolution of 100 ticks to pole with clock frequency of 100 kHz
//----------------------------------------------------------------------------
module reciever_reader(
		       sys_clk,
		       pwm_in,
		       pwm_out
                       );
   
   //Parameters
   // 
   parameter COUNTER_SIZE = 10;
   // determined by the input clock frequency (Want divided period to happen about 400 times per millisecond)
   parameter DIVIDER_SIZE  = 133;                // to change to 500, 4/5 * 133
   // Number of samples per millisecond
   parameter MAX_COUNT  = 400;                   // to change to 500, add 100
   parameter LONG_SEQUENCE = 10'b0000000000;     // must must must change to zeros later
   parameter SHORT_SEQUENCE = 10'b0000000000;
   //   inputs
   input wire sys_clk, pwm_in;
   
   //  output
   output wire [COUNTER_SIZE-1 : 0] pwm_out; 
   
   // internal registers
   reg [COUNTER_SIZE-1 : 0] 	    counter_int; // internal
   reg [COUNTER_SIZE : 0] 		    counter_div; // for dividing clock
   reg [COUNTER_SIZE-1 : 0] 	    out_holder;
   reg 				    pwm_h;
   
   // output has a continuously assigned value
   assign pwm_out = out_holder;
   
   // all sequential logic for reader in one always block
   always @ (posedge sys_clk)
     begin
	if(~pwm_in)
	  begin
	     if(counter_int > MAX_COUNT)
	       begin
		  out_holder[COUNTER_SIZE-1 : 0] <= (counter_int[COUNTER_SIZE-1 :0] - MAX_COUNT);	  
		  counter_int <= SHORT_SEQUENCE;	
	       end
	     else
	       begin
		  counter_int <= SHORT_SEQUENCE;	
	       end
	  end
	
	else
	  begin
	     out_holder <= out_holder;
	  end	
	
	if (pwm_in && (counter_div == 0))
	  begin
	     counter_int <= counter_int + 1;
	     counter_div <= DIVIDER_SIZE;
	  end
	else
	  begin
	     counter_div <= counter_div - 1;
	  end
     end
endmodule