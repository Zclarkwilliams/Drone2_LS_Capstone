/*

Code from ECE412/3 Capstone team from 2017, Lucas Myers,  Svyatoslav Zhuchenya, Casey Montgomery, Charley  Hill
File downloaded from: https://github.com/lucas709/drone_soc/blob/master/verilog_src/motor_offset_summer.v

*/

/* 
Module sums up 8-bit offset values from reciever_offset_generators.v and 
generates an 8-bit duty cycle value 0%(8'h00) to 100%(8'h64) for pwm_generator.v
*/

module motor_offset_summer (motor_total_offset, pitch_offset, roll_offset, yaw_offset, throttle_offset, clk);
	
	output reg [10:0] motor_total_offset;	// 10-bit value output 
	input      [10:0] throttle_offset; 		// 10-bit value input
	input      [10:0] pitch_offset; 		// 10-bit value input
	input      [10:0] roll_offset; 			// 10-bit value input
	input      [10:0] yaw_offset;			// 10-bit value input
	input            clk;
	
	always@ (posedge clk )begin
		motor_total_offset <= throttle_offset+pitch_offset+roll_offset+yaw_offset+450;
	// 450 is the base duty cycle value (45%) that is required for the ESCs to keep motors idle 
	end
	
endmodule 