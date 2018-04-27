/*

Code from ECE412/3 Capstone team from 2017, Lucas Myers,  Svyatoslav Zhuchenya, Casey Montgomery, Charley  Hill
File downloaded from: https://github.com/lucas709/drone_soc/blob/master/verilog_src/pwm_generator.v

*/
module pwm_generator(PWM_out, x_in, clk_in);   	
   output reg PWM_out = 0;          //PWM signal out
   input [10:0] x_in;                //control value that defines pulse width
   input        clk_in;              //clock for counter
   
   parameter DIVIDER_SIZE  = 100;   // divider to get the frequency of PWM down to 500Hz
   reg [9:0]  counter = 0;
   reg [15:0]  divider = 0;
   
   always@ (posedge clk_in )
     begin
	   if ( divider == DIVIDER_SIZE )
	     begin   
	       if ( counter < x_in + 12)
             PWM_out <= 1;
           else
             PWM_out <= 0;
	     counter <= counter+1;
	     divider <= 0;
	     end
	   else 
	     divider <= divider+1;
    end
endmodule