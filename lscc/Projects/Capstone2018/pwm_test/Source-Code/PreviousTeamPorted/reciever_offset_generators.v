/*

Code from ECE412/3 Capstone team from 2017, Lucas Myers,  Svyatoslav Zhuchenya, Casey Montgomery, Charley  Hill
File downloaded from: https://github.com/lucas709/drone_soc/blob/master/verilog_src/reciever_offset_generators.v

*/
module throttle_offset_generator(motor_1_offset, motor_2_offset, motor_3_offset, motor_4_offset, led, throttle_offset, switch_state, clk);
	output reg [10:0] motor_1_offset;
 	output reg [10:0] motor_2_offset;
 	output reg [10:0] motor_3_offset;
 	output reg [10:0] motor_4_offset;
 	output reg  [7:0]led;
	reg             [7:0] int_led;
	
 	input [9:0] 	  throttle_offset;
	input [9:0]     switch_state;
 	input             clk;
 
 	parameter OFFSET1 = 86;            // motor 1 offset
	parameter OFFSET2 = 82;            // motor 2 offset
	parameter OFFSET3 = 90;             // motor 3 offset
	parameter OFFSET4 = 70;             // motor 4 offset
	parameter STEP1 = 5;
	parameter STEP2 = 8;
	parameter STEP3 = 8;
	parameter STEP4 = 0;
	parameter INVERT_LED_CMD = 1; //Drive the LEDs with negative or positive logic	


	always@ (posedge clk) 
 	  begin
		if (switch_state < 250) 
	      begin
 		    if (throttle_offset <= 20) 
 		      begin
				  int_led <= 8'b1111_1111;
 			      motor_1_offset <= throttle_offset;
 			      motor_2_offset <= throttle_offset;
 			      motor_3_offset <= throttle_offset;
 			      motor_4_offset <= throttle_offset;
 		      end
		    else if (throttle_offset > 20 && throttle_offset <= 50) 
 		      begin
				  int_led <= 8'b0111_1111;
 			      motor_1_offset <= throttle_offset+OFFSET1+0*STEP1;
 			      motor_2_offset <= throttle_offset+OFFSET2+0*STEP2;
 			      motor_3_offset <= throttle_offset+OFFSET3+0*STEP3;
 			      motor_4_offset <= throttle_offset+OFFSET4+0*STEP4;
 		      end
 		    else if (throttle_offset > 50 && throttle_offset <= 100)
		      begin
				  int_led <= 8'b0011_1111;
		    	  motor_1_offset <= throttle_offset+OFFSET1+0*STEP1;
 			      motor_2_offset <= throttle_offset+OFFSET2+0*STEP2;
 			      motor_3_offset <= throttle_offset+OFFSET3+0*STEP3;
 			      motor_4_offset <= throttle_offset+OFFSET4+0*STEP4;
		      end
		    else if (throttle_offset > 100 && throttle_offset <= 150)
		      begin
				  int_led <= 8'b0001_1111;
		    	  motor_1_offset <= throttle_offset+OFFSET1+1*STEP1;
 			      motor_2_offset <= throttle_offset+OFFSET2+1*STEP2;
 			      motor_3_offset <= throttle_offset+OFFSET3+1*STEP3;
 			      motor_4_offset <= throttle_offset+OFFSET4+1*STEP4;
		      end
		    else if (throttle_offset > 150 && throttle_offset <= 200) 
 		      begin
				  int_led <= 8'b0000_1111;
 		    	  motor_1_offset <= throttle_offset+OFFSET1+1*STEP1;
 			      motor_2_offset <= throttle_offset+OFFSET2+1*STEP2;
 			      motor_3_offset <= throttle_offset+OFFSET3+1*STEP3;
 			      motor_4_offset <= throttle_offset+OFFSET4+1*STEP4;
 		      end
 		    else if (throttle_offset > 200 && throttle_offset <= 250)
		      begin
				  int_led <= 8'b0000_0111;
		    	  motor_1_offset <= throttle_offset+OFFSET1+2*STEP1;
 			      motor_2_offset <= throttle_offset+OFFSET2+2*STEP2;
 			      motor_3_offset <= throttle_offset+OFFSET3+2*STEP3;
 			      motor_4_offset <= throttle_offset+OFFSET4+2*STEP4;
		      end
		    else if (throttle_offset > 250 && throttle_offset <= 300)
		      begin
				  int_led <= 8'b0000_0011;
		    	  motor_1_offset <= throttle_offset+OFFSET1+2*STEP1;
 			      motor_2_offset <= throttle_offset+OFFSET2+2*STEP2;
 			      motor_3_offset <= throttle_offset+OFFSET3+2*STEP3;
 			      motor_4_offset <= throttle_offset+OFFSET4+2*STEP4;
		      end
		    else if (throttle_offset > 300 && throttle_offset <= 350)
		      begin
				  int_led <= 8'b0000_0001;
		    	  motor_1_offset <= throttle_offset+OFFSET1+3*STEP1;
 			      motor_2_offset <= throttle_offset+OFFSET2+3*STEP2;
 			      motor_3_offset <= throttle_offset+OFFSET3+3*STEP3;
 			      motor_4_offset <= throttle_offset+OFFSET4+3*STEP4;
		      end
		    else if (throttle_offset > 350)
		      begin
				  int_led <= 8'b00000000;
		    	  motor_1_offset <= throttle_offset+OFFSET1+3*STEP1;
 			      motor_2_offset <= throttle_offset+OFFSET2+3*STEP2;
 			      motor_3_offset <= throttle_offset+OFFSET3+3*STEP3;
 			      motor_4_offset <= throttle_offset+OFFSET4+3*STEP4;
		      end
		  end
		else if (switch_state >= 250)
		  begin
			int_led <= 8'b10101010;
			motor_1_offset <= -50;
			motor_2_offset <= -50;
			motor_3_offset <= -50;
			motor_4_offset <= -50;
		  end
		  
//Drive the LED outputs inverted?
        if (INVERT_LED_CMD == 1)
			led = ~int_led;
		else 
			led = int_led;
	  end
endmodule

module pitch_offset_generator(motor_1_offset, motor_2_offset, motor_3_offset, motor_4_offset, pitch_offset, throttle_offset, clk);
	output reg [10:0] motor_1_offset;
	output reg [10:0] motor_2_offset;
	output reg [10:0] motor_3_offset;
	output reg [10:0] motor_4_offset;
	input [9:0] 	  pitch_offset;
	input [9:0] 	  throttle_offset;
	input             clk;
   
    parameter PITCH_MAX = 100;         // maximum pitch value to decrement off motors by at max throttle
	parameter MAX_STEP = 10;           // step to decrease maximum decrement by ( PITCH_MAX / MAX_STEP must be = 10! )
	parameter PITCH_MIN = 5;           // minimum pitch value to increment on motors by at min throttle
	parameter MIN_STEP = 4;            // step to increment up from minimum ( PITCH_MIN + (MIN_STEP*20) = MAX VALUE )
   
    always@ (posedge clk) 
	  begin
	    if (throttle_offset > 50) 
		  begin
		    if (pitch_offset <= 10)
			  begin
			    motor_1_offset <=  PITCH_MIN+19*MIN_STEP;
			    motor_2_offset <= -PITCH_MAX+ 0*MAX_STEP;
			    motor_3_offset <=  PITCH_MIN+19*MIN_STEP;
			    motor_4_offset <= -PITCH_MAX+ 0*MAX_STEP;
			  end
		    else if (pitch_offset > 10 && pitch_offset <= 20)
			  begin
			    motor_1_offset <=  PITCH_MIN+18*MIN_STEP;
			    motor_2_offset <= -PITCH_MAX+ 1*MAX_STEP;
			    motor_3_offset <=  PITCH_MIN+18*MIN_STEP;
			    motor_4_offset <= -PITCH_MAX+ 1*MAX_STEP;
			  end
			else if (pitch_offset > 20 && pitch_offset <= 30)
			  begin
			    motor_1_offset <=  PITCH_MIN+17*MIN_STEP;
			    motor_2_offset <= -PITCH_MAX+ 2*MAX_STEP;
			    motor_3_offset <=  PITCH_MIN+17*MIN_STEP;
			    motor_4_offset <= -PITCH_MAX+ 2*MAX_STEP;
			  end
			else if (pitch_offset > 30 && pitch_offset <= 40)
			  begin
			    motor_1_offset <=  PITCH_MIN+16*MIN_STEP;
			    motor_2_offset <= -PITCH_MAX+ 3*MAX_STEP;
			    motor_3_offset <=  PITCH_MIN+16*MIN_STEP;
			    motor_4_offset <= -PITCH_MAX+ 3*MAX_STEP;
			  end
			else if (pitch_offset > 40 && pitch_offset <= 50)
			  begin
			    motor_1_offset <=  PITCH_MIN+15*MIN_STEP;
			    motor_2_offset <= -PITCH_MAX+ 4*MAX_STEP;
			    motor_3_offset <=  PITCH_MIN+15*MIN_STEP;
			    motor_4_offset <= -PITCH_MAX+ 4*MAX_STEP;
			  end
			else if (pitch_offset > 50 && pitch_offset <= 60)
			  begin
			    motor_1_offset <=  PITCH_MIN+14*MIN_STEP;
			    motor_2_offset <= -PITCH_MAX+ 5*MAX_STEP;
			    motor_3_offset <=  PITCH_MIN+14*MIN_STEP;
			    motor_4_offset <= -PITCH_MAX+ 5*MAX_STEP;
			  end
			else if (pitch_offset > 60 && pitch_offset <= 70)
			  begin
			    motor_1_offset <=  PITCH_MIN+13*MIN_STEP;
			    motor_2_offset <= -PITCH_MAX+ 6*MAX_STEP;
			    motor_3_offset <=  PITCH_MIN+13*MIN_STEP;
			    motor_4_offset <= -PITCH_MAX+ 6*MAX_STEP;
			  end
			else if (pitch_offset > 70 && pitch_offset <= 80)
			  begin
			    motor_1_offset <=  PITCH_MIN+12*MIN_STEP;
			    motor_2_offset <= -PITCH_MAX+ 7*MAX_STEP;
			    motor_3_offset <=  PITCH_MIN+12*MIN_STEP;
			    motor_4_offset <= -PITCH_MAX+ 7*MAX_STEP;
			  end
			else if (pitch_offset > 80 && pitch_offset <= 90)
			  begin
			    motor_1_offset <=  PITCH_MIN+11*MIN_STEP;
			    motor_2_offset <= -PITCH_MAX+ 8*MAX_STEP;
			    motor_3_offset <=  PITCH_MIN+11*MIN_STEP;
			    motor_4_offset <= -PITCH_MAX+ 8*MAX_STEP;
			  end
			else if (pitch_offset > 90 && pitch_offset <= 100)
			  begin
			    motor_1_offset <=  PITCH_MIN+10*MIN_STEP;
			    motor_2_offset <= -PITCH_MAX+ 9*MAX_STEP;
			    motor_3_offset <=  PITCH_MIN+10*MIN_STEP;
			    motor_4_offset <= -PITCH_MAX+ 9*MAX_STEP;
			  end
			else if (pitch_offset > 100 && pitch_offset <= 110)
			  begin
			    motor_1_offset <= PITCH_MIN+ 9*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= PITCH_MIN+ 9*MIN_STEP;
			    motor_4_offset <= 0;
			  end
			else if (pitch_offset > 110 && pitch_offset <= 120)
			  begin
			    motor_1_offset <= PITCH_MIN+ 8*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= PITCH_MIN+ 8*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (pitch_offset > 120 && pitch_offset <= 130)
			  begin
			    motor_1_offset <= PITCH_MIN+ 7*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= PITCH_MIN+ 7*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (pitch_offset > 130 && pitch_offset <= 140)
			  begin
			    motor_1_offset <= PITCH_MIN+ 6*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= PITCH_MIN+ 6*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (pitch_offset > 140 && pitch_offset <= 150)
			  begin
			    motor_1_offset <= PITCH_MIN+ 5*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= PITCH_MIN+ 5*MIN_STEP;
			    motor_4_offset <= 0;
			  end
			else if (pitch_offset > 150 && pitch_offset <= 160)
			  begin
			    motor_1_offset <= PITCH_MIN+ 4*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= PITCH_MIN+ 4*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (pitch_offset > 160 && pitch_offset <= 170)
			  begin
			    motor_1_offset <= PITCH_MIN+ 3*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= PITCH_MIN+ 3*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (pitch_offset > 170 && pitch_offset <= 180)
			  begin
			    motor_1_offset <= PITCH_MIN+ 2*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= PITCH_MIN+ 2*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (pitch_offset > 180 && pitch_offset <= 190)
			  begin
			    motor_1_offset <= PITCH_MIN+ 1*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= PITCH_MIN+ 1*MIN_STEP;
			    motor_4_offset <= 0;
			  end
			else if (pitch_offset > 190 && pitch_offset <= 195)
			  begin
			    motor_1_offset <= PITCH_MIN+ 0*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= PITCH_MIN+ 0*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (pitch_offset > 195 && pitch_offset <= 205)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
			else if (pitch_offset < 205 && pitch_offset <= 210)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= PITCH_MIN+ 0*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= PITCH_MIN+ 0*MIN_STEP;
			  end
		    else if (pitch_offset > 210 && pitch_offset <= 220)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= PITCH_MIN+ 1*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= PITCH_MIN+ 1*MIN_STEP;
			  end
		    else if (pitch_offset > 220 && pitch_offset <= 230)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= PITCH_MIN+ 2*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= PITCH_MIN+ 2*MIN_STEP;
			  end
		    else if (pitch_offset > 230 && pitch_offset <= 240)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= PITCH_MIN+ 3*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= PITCH_MIN+ 3*MIN_STEP;
			  end
		    else if (pitch_offset > 240 && pitch_offset <= 250)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= PITCH_MIN+ 4*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= PITCH_MIN+ 4*MIN_STEP;
			  end
			else if (pitch_offset > 250 && pitch_offset <= 260)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= PITCH_MIN+ 5*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= PITCH_MIN+ 5*MIN_STEP;
			  end
		    else if (pitch_offset > 260 && pitch_offset <= 270)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= PITCH_MIN+ 6*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= PITCH_MIN+ 6*MIN_STEP;
			  end
		    else if (pitch_offset > 270 && pitch_offset <= 280)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= PITCH_MIN+ 7*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= PITCH_MIN+ 7*MIN_STEP;
			  end
		    else if (pitch_offset > 280 && pitch_offset <= 290)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= PITCH_MIN+ 8*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= PITCH_MIN+ 8*MIN_STEP;
			  end
		    else if (pitch_offset > 290 && pitch_offset <= 300)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= PITCH_MIN+ 9*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= PITCH_MIN+ 9*MIN_STEP;
			  end
			else if (pitch_offset > 300 && pitch_offset <= 310)
			  begin
			    motor_1_offset <= -PITCH_MAX+ 9*MAX_STEP;
			    motor_2_offset <=  PITCH_MIN+10*MIN_STEP;
			    motor_3_offset <= -PITCH_MAX+ 9*MAX_STEP;
			    motor_4_offset <=  PITCH_MIN+10*MIN_STEP;
			  end
			else if (pitch_offset > 310 && pitch_offset <= 320)
			  begin
			    motor_1_offset <= -PITCH_MAX+ 8*MAX_STEP;
			    motor_2_offset <=  PITCH_MIN+11*MIN_STEP;
			    motor_3_offset <= -PITCH_MAX+ 8*MAX_STEP;
			    motor_4_offset <=  PITCH_MIN+11*MIN_STEP;
			  end
			else if (pitch_offset > 320 && pitch_offset <= 330)
			  begin
			    motor_1_offset <= -PITCH_MAX+ 7*MAX_STEP;
			    motor_2_offset <=  PITCH_MIN+12*MIN_STEP;
			    motor_3_offset <= -PITCH_MAX+ 7*MAX_STEP;
			    motor_4_offset <=  PITCH_MIN+12*MIN_STEP;
			  end
			else if (pitch_offset > 330 && pitch_offset <= 340)
			  begin
			    motor_1_offset <= -PITCH_MAX+ 6*MAX_STEP;
			    motor_2_offset <=  PITCH_MIN+13*MIN_STEP;
			    motor_3_offset <= -PITCH_MAX+ 6*MAX_STEP;
			    motor_4_offset <=  PITCH_MIN+13*MIN_STEP;
			  end
			else if (pitch_offset > 340 && pitch_offset <= 350)
			  begin
			    motor_1_offset <= -PITCH_MAX+ 5*MAX_STEP;
			    motor_2_offset <=  PITCH_MIN+14*MIN_STEP;
			    motor_3_offset <= -PITCH_MAX+ 5*MAX_STEP;
			    motor_4_offset <=  PITCH_MIN+14*MIN_STEP;
			  end
			else if (pitch_offset > 350 && pitch_offset <= 360)
			  begin
			    motor_1_offset <= -PITCH_MAX+ 4*MAX_STEP;
			    motor_2_offset <=  PITCH_MIN+15*MIN_STEP;
			    motor_3_offset <= -PITCH_MAX+ 4*MAX_STEP;
			    motor_4_offset <=  PITCH_MIN+15*MIN_STEP;
			  end
			else if (pitch_offset > 360 && pitch_offset <= 370)
			  begin
			    motor_1_offset <= -PITCH_MAX+ 3*MAX_STEP;
			    motor_2_offset <=  PITCH_MIN+16*MIN_STEP;
			    motor_3_offset <= -PITCH_MAX+ 3*MAX_STEP;
			    motor_4_offset <=  PITCH_MIN+16*MIN_STEP;
			  end
			else if (pitch_offset > 370 && pitch_offset <= 380)
			  begin
			    motor_1_offset <= -PITCH_MAX+ 2*MAX_STEP;
			    motor_2_offset <=  PITCH_MIN+17*MIN_STEP;
			    motor_3_offset <= -PITCH_MAX+ 2*MAX_STEP;
			    motor_4_offset <=  PITCH_MIN+17*MIN_STEP;
			  end
			else if (pitch_offset > 380 && pitch_offset <= 390)
			  begin
			    motor_1_offset <= -PITCH_MAX+ 1*MAX_STEP;
			    motor_2_offset <=  PITCH_MIN+18*MIN_STEP;
			    motor_3_offset <= -PITCH_MAX+ 1*MAX_STEP;
			    motor_4_offset <=  PITCH_MIN+18*MIN_STEP;
			  end
			else if (pitch_offset > 390)
			  begin
			    motor_1_offset <= -PITCH_MAX+ 0*MAX_STEP;
			    motor_2_offset <=  PITCH_MIN+19*MIN_STEP;
			    motor_3_offset <= -PITCH_MAX+ 0*MAX_STEP;
			    motor_4_offset <=  PITCH_MIN+19*MIN_STEP;
			  end
		  end 
	  end		  
endmodule

module roll_offset_generator(motor_1_offset, motor_2_offset, motor_3_offset, motor_4_offset, roll_offset, throttle_offset, clk);
	output reg [10:0] motor_1_offset;
	output reg [10:0] motor_2_offset;
	output reg [10:0] motor_3_offset;
	output reg [10:0] motor_4_offset;
	input [9:0] 	  roll_offset;
	input [9:0] 	  throttle_offset;
	input             clk;
   
    parameter ROLL_MAX = 100;         // maximum roll value to decrement off motors by at max throttle
	parameter MAX_STEP = 10;           // step to decrease maximum decrement by ( ROLL_MAX / MAX_STEP must be = 10! )
	parameter ROLL_MIN = 5;           // minimum roll value to increment on motors by at min throttle
	parameter MIN_STEP = 4;            // step to increment up from minimum ( ROLL_MIN + (MIN_STEP*20) = MAX VALUE )
   
    always@ (posedge clk) 
	  begin
	    if (throttle_offset > 50) 
		  begin
		    if (roll_offset <= 10)
			  begin
			    motor_1_offset <= -ROLL_MAX+ 0*MAX_STEP;
			    motor_2_offset <=  ROLL_MIN+19*MIN_STEP;
			    motor_3_offset <=  ROLL_MIN+19*MIN_STEP;
			    motor_4_offset <= -ROLL_MAX+ 0*MAX_STEP;
			  end
		    else if (roll_offset > 10 && roll_offset <= 20)
			  begin
			    motor_1_offset <= -ROLL_MAX+ 1*MAX_STEP;
			    motor_2_offset <=  ROLL_MIN+18*MIN_STEP;
			    motor_3_offset <=  ROLL_MIN+18*MIN_STEP;
			    motor_4_offset <= -ROLL_MAX+ 1*MAX_STEP;
			  end
			else if (roll_offset > 20 && roll_offset <= 30)
			  begin
			    motor_1_offset <= -ROLL_MAX+ 2*MAX_STEP;
			    motor_2_offset <=  ROLL_MIN+17*MIN_STEP;
			    motor_3_offset <=  ROLL_MIN+17*MIN_STEP;
			    motor_4_offset <= -ROLL_MAX+ 2*MAX_STEP;
			  end
			else if (roll_offset > 30 && roll_offset <= 40)
			  begin
			    motor_1_offset <= -ROLL_MAX+ 3*MAX_STEP;
			    motor_2_offset <=  ROLL_MIN+16*MIN_STEP;
			    motor_3_offset <=  ROLL_MIN+16*MIN_STEP;
			    motor_4_offset <= -ROLL_MAX+ 3*MAX_STEP;
			  end
			else if (roll_offset > 40 && roll_offset <= 50)
			  begin
			    motor_1_offset <= -ROLL_MAX+ 4*MAX_STEP;
			    motor_2_offset <=  ROLL_MIN+15*MIN_STEP;
			    motor_3_offset <=  ROLL_MIN+15*MIN_STEP;
			    motor_4_offset <= -ROLL_MAX+ 4*MAX_STEP;
			  end
			else if (roll_offset > 50 && roll_offset <= 60)
			  begin
			    motor_1_offset <= -ROLL_MAX+ 5*MAX_STEP;
			    motor_2_offset <=  ROLL_MIN+14*MIN_STEP;
			    motor_3_offset <=  ROLL_MIN+14*MIN_STEP;
			    motor_4_offset <= -ROLL_MAX+ 5*MAX_STEP;
			  end
			else if (roll_offset > 60 && roll_offset <= 70)
			  begin
			    motor_1_offset <= -ROLL_MAX+ 6*MAX_STEP;
			    motor_2_offset <=  ROLL_MIN+13*MIN_STEP;
			    motor_3_offset <=  ROLL_MIN+13*MIN_STEP;
			    motor_4_offset <= -ROLL_MAX+ 6*MAX_STEP;
			  end
			else if (roll_offset > 70 && roll_offset <= 80)
			  begin
			    motor_1_offset <= -ROLL_MAX+ 7*MAX_STEP;
			    motor_2_offset <=  ROLL_MIN+12*MIN_STEP;
			    motor_3_offset <=  ROLL_MIN+12*MIN_STEP;
			    motor_4_offset <= -ROLL_MAX+ 7*MAX_STEP;
			  end
			else if (roll_offset > 80 && roll_offset <= 90)
			  begin
			    motor_1_offset <= -ROLL_MAX+ 8*MAX_STEP;
			    motor_2_offset <=  ROLL_MIN+11*MIN_STEP;
			    motor_3_offset <=  ROLL_MIN+11*MIN_STEP;
			    motor_4_offset <= -ROLL_MAX+ 8*MAX_STEP;
			  end
			else if (roll_offset > 90 && roll_offset <= 100)
			  begin
			    motor_1_offset <= -ROLL_MAX+ 9*MAX_STEP;
			    motor_2_offset <=  ROLL_MIN+10*MIN_STEP;
			    motor_3_offset <=  ROLL_MIN+10*MIN_STEP;
			    motor_4_offset <= -ROLL_MAX+ 9*MAX_STEP;
			  end
			else if (roll_offset > 100 && roll_offset <= 110)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= ROLL_MIN+ 9*MIN_STEP;
			    motor_3_offset <= ROLL_MIN+ 9*MIN_STEP;
			    motor_4_offset <= 0;
			  end
			else if (roll_offset > 110 && roll_offset <= 120)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= ROLL_MIN+ 8*MIN_STEP;
			    motor_3_offset <= ROLL_MIN+ 8*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (roll_offset > 120 && roll_offset <= 130)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= ROLL_MIN+ 7*MIN_STEP;
			    motor_3_offset <= ROLL_MIN+ 7*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (roll_offset > 130 && roll_offset <= 140)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= ROLL_MIN+ 6*MIN_STEP;
			    motor_3_offset <= ROLL_MIN+ 6*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (roll_offset > 140 && roll_offset <= 150)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= ROLL_MIN+ 5*MIN_STEP;
			    motor_3_offset <= ROLL_MIN+ 5*MIN_STEP;
			    motor_4_offset <= 0;
			  end
			else if (roll_offset > 150 && roll_offset <= 160)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= ROLL_MIN+ 4*MIN_STEP;
			    motor_3_offset <= ROLL_MIN+ 4*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (roll_offset > 160 && roll_offset <= 170)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= ROLL_MIN+ 3*MIN_STEP;
			    motor_3_offset <= ROLL_MIN+ 3*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (roll_offset > 170 && roll_offset <= 180)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= ROLL_MIN+ 2*MIN_STEP;
			    motor_3_offset <= ROLL_MIN+ 2*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (roll_offset > 180 && roll_offset <= 190)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= ROLL_MIN+ 1*MIN_STEP;
			    motor_3_offset <= ROLL_MIN+ 1*MIN_STEP;
			    motor_4_offset <= 0;
			  end
			else if (roll_offset > 190 && roll_offset <= 195)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= ROLL_MIN+ 0*MIN_STEP;
			    motor_3_offset <= ROLL_MIN+ 0*MIN_STEP;
			    motor_4_offset <= 0;
			  end
		    else if (roll_offset > 195 && roll_offset <= 205)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
			else if (roll_offset < 205 && roll_offset <= 210)
			  begin
			    motor_1_offset <= ROLL_MIN+ 0*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= ROLL_MIN+ 0*MIN_STEP;
			  end
		    else if (roll_offset > 210 && roll_offset <= 220)
			  begin
			    motor_1_offset <= ROLL_MIN+ 1*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= ROLL_MIN+ 1*MIN_STEP;
			  end
		    else if (roll_offset > 220 && roll_offset <= 230)
			  begin
			    motor_1_offset <= ROLL_MIN+ 2*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= ROLL_MIN+ 2*MIN_STEP;
			  end
		    else if (roll_offset > 230 && roll_offset <= 240)
			  begin
			    motor_1_offset <= ROLL_MIN+ 3*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= ROLL_MIN+ 3*MIN_STEP;
			  end
		    else if (roll_offset > 240 && roll_offset <= 250)
			  begin
			    motor_1_offset <= ROLL_MIN+ 4*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= ROLL_MIN+ 4*MIN_STEP;
			  end
			else if (roll_offset > 250 && roll_offset <= 260)
			  begin
			    motor_1_offset <= ROLL_MIN+ 5*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= ROLL_MIN+ 5*MIN_STEP;
			  end
		    else if (roll_offset > 260 && roll_offset <= 270)
			  begin
			    motor_1_offset <= ROLL_MIN+ 6*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= ROLL_MIN+ 6*MIN_STEP;
			  end
		    else if (roll_offset > 270 && roll_offset <= 280)
			  begin
			    motor_1_offset <= ROLL_MIN+ 7*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= ROLL_MIN+ 7*MIN_STEP;
			  end
		    else if (roll_offset > 280 && roll_offset <= 290)
			  begin
			    motor_1_offset <= ROLL_MIN+ 8*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= ROLL_MIN+ 8*MIN_STEP;
			  end
		    else if (roll_offset > 290 && roll_offset <= 300)
			  begin
			    motor_1_offset <= ROLL_MIN+ 9*MIN_STEP;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= ROLL_MIN+ 9*MIN_STEP;
			  end
			else if (roll_offset > 300 && roll_offset <= 310)
			  begin
			    motor_1_offset <=  ROLL_MIN+10*MIN_STEP;
			    motor_2_offset <= -ROLL_MAX+ 9*MAX_STEP;
			    motor_3_offset <= -ROLL_MAX+ 9*MAX_STEP;
			    motor_4_offset <=  ROLL_MIN+10*MIN_STEP;
			  end
			else if (roll_offset > 310 && roll_offset <= 320)
			  begin
			    motor_1_offset <=  ROLL_MIN+11*MIN_STEP;
			    motor_2_offset <= -ROLL_MAX+ 8*MAX_STEP;
			    motor_3_offset <= -ROLL_MAX+ 8*MAX_STEP;
			    motor_4_offset <=  ROLL_MIN+11*MIN_STEP;
			  end
			else if (roll_offset > 320 && roll_offset <= 330)
			  begin
			    motor_1_offset <=  ROLL_MIN+12*MIN_STEP;
			    motor_2_offset <= -ROLL_MAX+ 7*MAX_STEP;
			    motor_3_offset <= -ROLL_MAX+ 7*MAX_STEP;
			    motor_4_offset <=  ROLL_MIN+12*MIN_STEP;
			  end
			else if (roll_offset > 330 && roll_offset <= 340)
			  begin
			    motor_1_offset <=  ROLL_MIN+13*MIN_STEP;
			    motor_2_offset <= -ROLL_MAX+ 6*MAX_STEP;
			    motor_3_offset <= -ROLL_MAX+ 6*MAX_STEP;
			    motor_4_offset <=  ROLL_MIN+13*MIN_STEP;
			  end
			else if (roll_offset > 340 && roll_offset <= 350)
			  begin
			    motor_1_offset <=  ROLL_MIN+14*MIN_STEP;
			    motor_2_offset <= -ROLL_MAX+ 5*MAX_STEP;
			    motor_3_offset <= -ROLL_MAX+ 5*MAX_STEP;
			    motor_4_offset <=  ROLL_MIN+14*MIN_STEP;
			  end
			else if (roll_offset > 350 && roll_offset <= 360)
			  begin
			    motor_1_offset <=  ROLL_MIN+15*MIN_STEP;
			    motor_2_offset <= -ROLL_MAX+ 4*MAX_STEP;
			    motor_3_offset <= -ROLL_MAX+ 4*MAX_STEP;
			    motor_4_offset <=  ROLL_MIN+15*MIN_STEP;
			  end
			else if (roll_offset > 360 && roll_offset <= 370)
			  begin
			    motor_1_offset <=  ROLL_MIN+16*MIN_STEP;
			    motor_2_offset <= -ROLL_MAX+ 3*MAX_STEP;
			    motor_3_offset <= -ROLL_MAX+ 3*MAX_STEP;
			    motor_4_offset <=  ROLL_MIN+16*MIN_STEP;
			  end
			else if (roll_offset > 370 && roll_offset <= 380)
			  begin
			    motor_1_offset <=  ROLL_MIN+17*MIN_STEP;
			    motor_2_offset <= -ROLL_MAX+ 2*MAX_STEP;
			    motor_3_offset <= -ROLL_MAX+ 2*MAX_STEP;
			    motor_4_offset <=  ROLL_MIN+17*MIN_STEP;
			  end
			else if (roll_offset > 380 && roll_offset <= 390)
			  begin
			    motor_1_offset <=  ROLL_MIN+18*MIN_STEP;
			    motor_2_offset <= -ROLL_MAX+ 1*MAX_STEP;
			    motor_3_offset <= -ROLL_MAX+ 1*MAX_STEP;
			    motor_4_offset <=  ROLL_MIN+18*MIN_STEP;
			  end
			else if (roll_offset > 390)
			  begin
			    motor_1_offset <=  ROLL_MIN+19*MIN_STEP;
			    motor_2_offset <= -ROLL_MAX+ 0*MAX_STEP;
			    motor_3_offset <= -ROLL_MAX+ 0*MAX_STEP;
			    motor_4_offset <=  ROLL_MIN+19*MIN_STEP;
			  end
		  end 
	  end		  
endmodule

module yaw_offset_generator(motor_1_offset, motor_2_offset, motor_3_offset, motor_4_offset, yaw_offset, throttle_offset, clk);
	output reg [10:0] motor_1_offset;
	output reg [10:0] motor_2_offset;
	output reg [10:0] motor_3_offset;
	output reg [10:0] motor_4_offset;
	input [9:0] 	  yaw_offset;
	input [9:0] 	  throttle_offset;
	input             clk;
   
    parameter YAW_MAX = 100;         // maximum yaw value
    parameter MAX_STEP = 10;         // step to increment down from max value ( YAW_MAX / MAX_STEP must be >= 10! )
	parameter YAW_MIN = 5;           // minimum yaw value
	parameter MIN_STEP = 4;          // step to increment up from minimum
	
    always@ (posedge clk) 
	  begin
	    if (throttle_offset > 50) 
		  begin
		    if (yaw_offset <= 10)
			  begin
			    motor_1_offset <= -YAW_MAX+ 0*MAX_STEP;
			    motor_2_offset <= -YAW_MAX+ 0*MAX_STEP;
			    motor_3_offset <=  YAW_MIN+19*MIN_STEP;
			    motor_4_offset <=  YAW_MIN+19*MIN_STEP;
			  end
		    else if (yaw_offset > 10 && yaw_offset <= 20)
			  begin
			    motor_1_offset <= -YAW_MAX+ 1*MAX_STEP;
			    motor_2_offset <= -YAW_MAX+ 1*MAX_STEP;
			    motor_3_offset <=  YAW_MIN+18*MIN_STEP;
			    motor_4_offset <=  YAW_MIN+18*MIN_STEP;
			  end
			else if (yaw_offset > 20 && yaw_offset <= 30)
			  begin
			    motor_1_offset <= -YAW_MAX+ 2*MAX_STEP;
			    motor_2_offset <= -YAW_MAX+ 2*MAX_STEP;
			    motor_3_offset <=  YAW_MIN+17*MIN_STEP;
			    motor_4_offset <=  YAW_MIN+17*MIN_STEP;
			  end
			else if (yaw_offset > 30 && yaw_offset <= 40)
			  begin
			    motor_1_offset <= -YAW_MAX+ 3*MAX_STEP;
			    motor_2_offset <= -YAW_MAX+ 3*MAX_STEP;
			    motor_3_offset <=  YAW_MIN+16*MIN_STEP;
			    motor_4_offset <=  YAW_MIN+16*MIN_STEP;
			  end
			else if (yaw_offset > 40 && yaw_offset <= 50)
			  begin
			    motor_1_offset <= -YAW_MAX+ 4*MAX_STEP;
			    motor_2_offset <= -YAW_MAX+ 4*MAX_STEP;
			    motor_3_offset <=  YAW_MIN+15*MIN_STEP;
			    motor_4_offset <=  YAW_MIN+15*MIN_STEP;
			  end
			else if (yaw_offset > 50 && yaw_offset <= 60)
			  begin
			    motor_1_offset <= -YAW_MAX+ 5*MAX_STEP;
			    motor_2_offset <= -YAW_MAX+ 5*MAX_STEP;
			    motor_3_offset <=  YAW_MIN+14*MIN_STEP;
			    motor_4_offset <=  YAW_MIN+14*MIN_STEP;
			  end
			else if (yaw_offset > 60 && yaw_offset <= 70)
			  begin
			    motor_1_offset <= -YAW_MAX+ 6*MAX_STEP;
			    motor_2_offset <= -YAW_MAX+ 6*MAX_STEP;
			    motor_3_offset <=  YAW_MIN+13*MIN_STEP;
			    motor_4_offset <=  YAW_MIN+13*MIN_STEP;
			  end
			else if (yaw_offset > 70 && yaw_offset <= 80)
			  begin
			    motor_1_offset <= -YAW_MAX+ 7*MAX_STEP;
			    motor_2_offset <= -YAW_MAX+ 7*MAX_STEP;
			    motor_3_offset <=  YAW_MIN+12*MIN_STEP;
			    motor_4_offset <=  YAW_MIN+12*MIN_STEP;
			  end
			else if (yaw_offset > 80 && yaw_offset <= 90)
			  begin
			    motor_1_offset <= -YAW_MAX+ 8*MAX_STEP;
			    motor_2_offset <= -YAW_MAX+ 8*MAX_STEP;
			    motor_3_offset <=  YAW_MIN+11*MIN_STEP;
			    motor_4_offset <=  YAW_MIN+11*MIN_STEP;
			  end
			else if (yaw_offset > 90 && yaw_offset <= 100)
			  begin
			    motor_1_offset <= -YAW_MAX+ 9*MAX_STEP;
			    motor_2_offset <= -YAW_MAX+ 9*MAX_STEP;
			    motor_3_offset <=  YAW_MIN+10*MIN_STEP;
			    motor_4_offset <=  YAW_MIN+10*MIN_STEP;
			  end
			else if (yaw_offset > 100 && yaw_offset <= 110)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= YAW_MIN+ 9*MIN_STEP;
			    motor_4_offset <= YAW_MIN+ 9*MIN_STEP;
			  end
			else if (yaw_offset > 110 && yaw_offset <= 120)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= YAW_MIN+ 8*MIN_STEP;
			    motor_4_offset <= YAW_MIN+ 8*MIN_STEP;
			  end
		    else if (yaw_offset > 120 && yaw_offset <= 130)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= YAW_MIN+ 7*MIN_STEP;
			    motor_4_offset <= YAW_MIN+ 7*MIN_STEP;
			  end
		    else if (yaw_offset > 130 && yaw_offset <= 140)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= YAW_MIN+ 6*MIN_STEP;
			    motor_4_offset <= YAW_MIN+ 6*MIN_STEP;
			  end
		    else if (yaw_offset > 140 && yaw_offset <= 150)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= YAW_MIN+ 5*MIN_STEP;
			    motor_4_offset <= YAW_MIN+ 5*MIN_STEP;
			  end
			else if (yaw_offset > 150 && yaw_offset <= 160)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= YAW_MIN+ 4*MIN_STEP;
			    motor_4_offset <= YAW_MIN+ 4*MIN_STEP;
			  end
		    else if (yaw_offset > 160 && yaw_offset <= 170)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= YAW_MIN+ 3*MIN_STEP;
			    motor_4_offset <= YAW_MIN+ 3*MIN_STEP;
			  end
		    else if (yaw_offset > 170 && yaw_offset <= 180)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= YAW_MIN+ 2*MIN_STEP;
			    motor_4_offset <= YAW_MIN+ 2*MIN_STEP;
			  end
		    else if (yaw_offset > 180 && yaw_offset <= 190)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= YAW_MIN+ 1*MIN_STEP;
			    motor_4_offset <= YAW_MIN+ 1*MIN_STEP;
			  end
			else if (yaw_offset > 190 && yaw_offset <= 195)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= YAW_MIN+ 0*MIN_STEP;
			    motor_4_offset <= YAW_MIN+ 0*MIN_STEP;
			  end
		    else if (yaw_offset > 195 && yaw_offset <= 205)
			  begin
			    motor_1_offset <= 0;
			    motor_2_offset <= 0;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
			else if (yaw_offset < 205 && yaw_offset <= 210)
			  begin
			    motor_1_offset <= YAW_MIN+ 0*MIN_STEP;
			    motor_2_offset <= YAW_MIN+ 0*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
		    else if (yaw_offset > 210 && yaw_offset <= 220)
			  begin
			    motor_1_offset <= YAW_MIN+ 1*MIN_STEP;
			    motor_2_offset <= YAW_MIN+ 1*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
		    else if (yaw_offset > 220 && yaw_offset <= 230)
			  begin
			    motor_1_offset <= YAW_MIN+ 2*MIN_STEP;
			    motor_2_offset <= YAW_MIN+ 2*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
		    else if (yaw_offset > 230 && yaw_offset <= 240)
			  begin
			    motor_1_offset <= YAW_MIN+ 3*MIN_STEP;
			    motor_2_offset <= YAW_MIN+ 3*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
		    else if (yaw_offset > 240 && yaw_offset <= 250)
			  begin
			    motor_1_offset <= YAW_MIN+ 4*MIN_STEP;
			    motor_2_offset <= YAW_MIN+ 4*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
			else if (yaw_offset > 250 && yaw_offset <= 260)
			  begin
			    motor_1_offset <= YAW_MIN+ 5*MIN_STEP;
			    motor_2_offset <= YAW_MIN+ 5*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
		    else if (yaw_offset > 260 && yaw_offset <= 270)
			  begin
			    motor_1_offset <= YAW_MIN+ 6*MIN_STEP;
			    motor_2_offset <= YAW_MIN+ 6*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
		    else if (yaw_offset > 270 && yaw_offset <= 280)
			  begin
			    motor_1_offset <= YAW_MIN+ 7*MIN_STEP;
			    motor_2_offset <= YAW_MIN+ 7*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
		    else if (yaw_offset > 280 && yaw_offset <= 290)
			  begin
			    motor_1_offset <= YAW_MIN+ 8*MIN_STEP;
			    motor_2_offset <= YAW_MIN+ 8*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
		    else if (yaw_offset > 290 && yaw_offset <= 300)
			  begin
			    motor_1_offset <= YAW_MIN+ 9*MIN_STEP;
			    motor_2_offset <= YAW_MIN+ 9*MIN_STEP;
			    motor_3_offset <= 0;
			    motor_4_offset <= 0;
			  end
			else if (yaw_offset > 300 && yaw_offset <= 310)
			  begin
			    motor_1_offset <=  YAW_MIN+10*MIN_STEP;
			    motor_2_offset <=  YAW_MIN+10*MIN_STEP;
			    motor_3_offset <= -YAW_MAX+ 9*MAX_STEP;
			    motor_4_offset <= -YAW_MAX+ 9*MAX_STEP;
			  end
			else if (yaw_offset > 310 && yaw_offset <= 320)
			  begin
			    motor_1_offset <=  YAW_MIN+11*MIN_STEP;
			    motor_2_offset <=  YAW_MIN+11*MIN_STEP;
			    motor_3_offset <= -YAW_MAX+ 8*MAX_STEP;
			    motor_4_offset <= -YAW_MAX+ 8*MAX_STEP;
			  end
			else if (yaw_offset > 320 && yaw_offset <= 330)
			  begin
			    motor_1_offset <=  YAW_MIN+12*MIN_STEP;
			    motor_2_offset <=  YAW_MIN+12*MIN_STEP;
			    motor_3_offset <= -YAW_MAX+ 7*MAX_STEP;
			    motor_4_offset <= -YAW_MAX+ 7*MAX_STEP;
			  end
			else if (yaw_offset > 330 && yaw_offset <= 340)
			  begin
			    motor_1_offset <=  YAW_MIN+13*MIN_STEP;
			    motor_2_offset <=  YAW_MIN+13*MIN_STEP;
			    motor_3_offset <= -YAW_MAX+ 6*MAX_STEP;
			    motor_4_offset <= -YAW_MAX+ 6*MAX_STEP;
			  end
			else if (yaw_offset > 340 && yaw_offset <= 350)
			  begin
			    motor_1_offset <=  YAW_MIN+14*MIN_STEP;
			    motor_2_offset <=  YAW_MIN+14*MIN_STEP;
			    motor_3_offset <= -YAW_MAX+ 5*MAX_STEP;
			    motor_4_offset <= -YAW_MAX+ 5*MAX_STEP;
			  end
			else if (yaw_offset > 350 && yaw_offset <= 360)
			  begin
			    motor_1_offset <=  YAW_MIN+15*MIN_STEP;
			    motor_2_offset <=  YAW_MIN+15*MIN_STEP;
			    motor_3_offset <= -YAW_MAX+ 4*MAX_STEP;
			    motor_4_offset <= -YAW_MAX+ 4*MAX_STEP;
			  end
			else if (yaw_offset > 360 && yaw_offset <= 370)
			  begin
			    motor_1_offset <=  YAW_MIN+16*MIN_STEP;
			    motor_2_offset <=  YAW_MIN+16*MIN_STEP;
			    motor_3_offset <= -YAW_MAX+ 3*MAX_STEP;
			    motor_4_offset <= -YAW_MAX+ 3*MAX_STEP;
			  end
			else if (yaw_offset > 370 && yaw_offset <= 380)
			  begin
			    motor_1_offset <=  YAW_MIN+17*MIN_STEP;
			    motor_2_offset <=  YAW_MIN+17*MIN_STEP;
			    motor_3_offset <= -YAW_MAX+ 2*MAX_STEP;
			    motor_4_offset <= -YAW_MAX+ 2*MAX_STEP;
			  end
			else if (yaw_offset > 380 && yaw_offset <= 390)
			  begin
			    motor_1_offset <=  YAW_MIN+18*MIN_STEP;
			    motor_2_offset <=  YAW_MIN+18*MIN_STEP;
			    motor_3_offset <= -YAW_MAX+ 1*MAX_STEP;
			    motor_4_offset <= -YAW_MAX+ 1*MAX_STEP;
			  end
			else if (yaw_offset > 390)
			  begin
			    motor_1_offset <=  YAW_MIN+19*MIN_STEP;
			    motor_2_offset <=  YAW_MIN+19*MIN_STEP;
			    motor_3_offset <= -YAW_MAX+ 0*MAX_STEP;
			    motor_4_offset <= -YAW_MAX+ 0*MAX_STEP;
			  end
		  end 
	  end	  
endmodule