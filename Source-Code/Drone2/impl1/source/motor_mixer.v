/**
 * module pid_mixer - takes finalized rates and converts them to motor rates
 *
 * Parameters
 * @RATE_BIT_WIDTH: Number of bits for throttle/yaw/roll/pitch val inputs
 * @MOTOR_RATE_BIT_WIDTH: Number of bits for motor outputs
 *
 * Outputs
 * @motor_1_rate: rate to run motor 1 at (units?)
 * @motor_2_rate: rate to run motor 2 at (units?)
 * @motor_3_rate: rate to run motor 3 at (units?)
 * @motor_4_rate: rate to run motor 4 at (units?)
 *
 * Inputs
 * @throttle_rate: throttle rate (rad/s) in fixed point 2's complement
 * @yaw_rate: yaw rate (rad/s) in fixed point 2's complement
 * @roll_rate: roll rate (rad/s) in fixed point 2's complement
 * @pitch_rate: pitch rate (rad/s) in fixed point 2's complement
 * @sys_clk: system clock
 *
 *
 *	Top level instantiation of module
 *	
 *		pid_mixer #(
 *					PID_RATE_BIT_WIDTH, 
 *					`MOTOR_RATE_BIT_WIDTH) 
 *		pid_mixer	(
 *					.motor_1_rate(motor_1_rate),
 *					.motor_2_rate(motor_2_rate),
 *					.motor_3_rate(motor_3_rate),
 *					.motor_4_rate(motor_4_rate),
 *					.throttle_rate(throttle_target_rate),
 *					.yaw_rate(yaw_rate),
 *					.roll_rate(roll_rate),
 *					.pitch_rate(pitch_rate),
 *					.sys_clk(sys_clk)
 *					);
 *
 *		 		   |  <----			 ---->  |
 *		 		   | Motor_1		Motor_2 |
 *		 		   V 	\			  /     V
 *		 				 \			 /
 *		 				  \			/
 *		 				   /-------\
 *		 				   | DRONE |
 *		 				   | 	   |
 *		 				   \-------/
 *						  /		    \	
 *						 /			 \
 *						/			  \
 *		 		     ---->	| 	   | <---- 
 *		 		    Motor_3 |	   | Motor_4
 *		 		            V      V 
 *
 *		Motors_1 and Motor_4 will spin counter clockwise (CCW)
 *		Motors_2 and Motor_3 will spin clockwise (CW)
 *
 *		TODO Check that the motor spin rotation is correct
 *
 *		Referenceing the image above the following equations are generated
 *			Motor_1 = bias + throttle + (-)yaw/2 + (+)roll/2 + (-)pitch/2
 *			Motor_2 = bias + throttle + (+)yaw/2 + (-)roll/2 + (-)pitch/2
 *			Motor_3 = bias + throttle + (+)yaw/2 + (+)roll/2 + (+)pitch/2
 *			Motor_4 = bias + throttle + (-)yaw/2 + (-)roll/2 + (+)pitch/2
 *
 */
 
`timescale 1ns / 1ns
//`include "pid_mixer_defs.v"



module motor_mixer	#(parameter BIT_WIDTH = 16,
					  parameter MOTOR_RATE_BIT_WIDTH = 8)
					(//	Inputs
					 input  wire rst_n,
					 input  wire sys_clk,
					 input  wire [BIT_WIDTH-1:0] yaw_rate,
					 input  wire [BIT_WIDTH-1:0] roll_rate,
					 input  wire [BIT_WIDTH-1:0] pitch_rate,
					 input  wire [BIT_WIDTH-1:0] throttle_rate,
					 //	Outputs
					 output reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor_1_rate/*,
					 output reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor_2_rate,
					 output reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor_3_rate,
					 output reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor_4_rate*/);
	
	reg  [BIT_WIDTH-1:0] n_m1_yaw_rate;
	reg  [BIT_WIDTH-1:0] n_m2_yaw_rate;
	reg  [BIT_WIDTH-1:0] n_m3_yaw_rate;
	reg  [BIT_WIDTH-1:0] n_m4_yaw_rate;
	
	reg	 [BIT_WIDTH-1:0] n_m1_roll_rate;
	reg	 [BIT_WIDTH-1:0] n_m2_roll_rate;
	reg	 [BIT_WIDTH-1:0] n_m3_roll_rate;
	reg	 [BIT_WIDTH-1:0] n_m4_roll_rate;
	
	reg	 [BIT_WIDTH-1:0] n_m1_pitch_rate;
	reg	 [BIT_WIDTH-1:0] n_m2_pitch_rate;
	reg	 [BIT_WIDTH-1:0] n_m3_pitch_rate;
	reg	 [BIT_WIDTH-1:0] n_m4_pitch_rate;
	
	reg	 [BIT_WIDTH-1:0] n_throttle_rate;
	
	reg	 [BIT_WIDTH-1:0] motor_1_rate_last;
	reg	 [BIT_WIDTH-1:0] motor_2_rate_last;
	reg	 [BIT_WIDTH-1:0] motor_3_rate_last;
	reg	 [BIT_WIDTH-1:0] motor_4_rate_last;
	
	wire [BIT_WIDTH-1:0] motor_1_output;
	wire [BIT_WIDTH-1:0] motor_2_output;
	wire [BIT_WIDTH-1:0] motor_3_output;
	wire [BIT_WIDTH-1:0] motor_4_output;
	
	reg  [BIT_WIDTH-1:0] motor_1_mapped;
	reg  [BIT_WIDTH-1:0] motor_2_mapped;
	reg  [BIT_WIDTH-1:0] motor_3_mapped;
	reg  [BIT_WIDTH-1:0] motor_4_mapped;
	
	reg	 [`MOTOR_MIXER_STATE_BIT_WIDTH-1:0]state;
	
	motor_rate_calculator	#(BIT_WIDTH)
		motor_1_rate_calc	(.bias(`BIAS),
							 .rst_n(rst_n),
							 .yaw_rate(n_m1_yaw_rate),
							 .roll_rate(n_m1_roll_rate),
							 .pitch_rate(n_m1_pitch_rate),
							 .throttle_rate(n_throttle_rate),
							 .motor_rate(motor_1_output));
	/*
	motor_rate_calculator	#(BIT_WIDTH)
		motor_2_rate_calc	(.bias(`BIAS),
							 .rst_n(rst_n),
							 .yaw_rate(n_m2_yaw_rate),
							 .roll_rate(n_m2_roll_rate),
							 .pitch_rate(n_m2_pitch_rate),
							 .throttle_rate(n_throttle_rate),
							 .motor_rate(motor_2_output));
	motor_rate_calculator	#(BIT_WIDTH)
		motor_3_rate_calc	(.bias(`BIAS),
							 .rst_n(rst_n),
							 .yaw_rate(n_m3_yaw_rate),
							 .roll_rate(n_m3_roll_rate),
							 .pitch_rate(n_m3_pitch_rate),
							 .throttle_rate(n_throttle_rate),
							 .motor_rate(motor_3_output));
	motor_rate_calculator	#(BIT_WIDTH)
		motor_4_rate_calc	(.bias(`BIAS),
							 .rst_n(rst_n),
							 .yaw_rate(n_m4_yaw_rate),
							 .roll_rate(n_m4_roll_rate),
							 .pitch_rate(n_m4_pitch_rate),
							 .throttle_rate(n_throttle_rate),
							 .motor_rate(motor_4_output));
	*/
	always @(posedge sys_clk or negedge rst_n) begin
		$display("IN MOTOR_MIXER_1");
		if (!rst_n) begin
			n_m1_yaw_rate		<= `ZERO;
			//n_m4_yaw_rate		<= `ZERO;
			//n_m2_yaw_rate		<= `ZERO;
			//n_m3_yaw_rate		<= `ZERO;
			
			n_m1_roll_rate		<= `ZERO;
			//n_m3_roll_rate		<= `ZERO;
			//n_m2_roll_rate		<= `ZERO;
			//n_m4_roll_rate		<= `ZERO;
			
			n_m1_pitch_rate		<= `ZERO;
			//n_m2_pitch_rate		<= `ZERO;
			//n_m3_pitch_rate		<= `ZERO;
			//n_m4_pitch_rate		<= `ZERO;
			
			n_throttle_rate		<= `ZERO;
			end
		else begin
			n_m1_yaw_rate		<= (~yaw_rate + 1'b1);
			//n_m4_yaw_rate		<= (~yaw_rate + 1'b1);
			//n_m2_yaw_rate		<= yaw_rate;
			//n_m3_yaw_rate		<= yaw_rate;
			
			n_m1_roll_rate		<= roll_rate;
			//n_m3_roll_rate		<= roll_rate;
			//n_m2_roll_rate		<= (~roll_rate + 1'b1);
			//n_m4_roll_rate		<= (~roll_rate + 1'b1);
			
			n_m1_pitch_rate		<= (~pitch_rate + 1'b1);
			//n_m2_pitch_rate		<= (~pitch_rate + 1'b1);
			//n_m3_pitch_rate		<= pitch_rate;
			//n_m4_pitch_rate		<= pitch_rate;
			
			n_throttle_rate		<= throttle_rate;
			end
		end
	
	always @(posedge sys_clk)begin
		$display("IN MOTOR_MIXER_2 %d ", state);
		case(state)
			`STATE_MAP_16_to_8: 	begin 
				$display("IN_MOTOR_MIXER_MAPPING,   %d", motor_1_output);
				if (motor_1_output || motor_2_output || motor_3_output || motor_4_output) begin
					$display("MAPPING  %d  to  %d  ", motor_1_output, motor_1_mapped);
					motor_1_mapped = motor_1_output;//((motor_1_output + `MAP_ROUND_DOWN) >> 8); // Map 16 bit to 8
					//motor_2_mapped = ((motor_2_output + `MAP_ROUND_DOWN) >> 8); // Map 16 bit to 8
					//motor_3_mapped = ((motor_3_output + `MAP_ROUND_DOWN) >> 8); // Map 16 bit to 8
					//motor_4_mapped = ((motor_4_output + `MAP_ROUND_DOWN) >> 8); // Map 16 bit to 8
					state = 1;
					end
				else
					state = `STATE_MAP_16_to_8;
				end
			`STATE_SEND_OUTPUT: 	begin
				$display("IN_MOTOR_MIXER_SEND_OUTPUT");
				if (motor_1_mapped || motor_2_mapped || motor_3_mapped || motor_4_mapped) begin
					if (motor_1_mapped < `ESC_MIN || motor_1_mapped > `ESC_MAX) begin
						$display("SET_TO_OUT", motor_1_mapped);
						motor_1_rate 		= motor_1_rate_last;
						end
					else begin 
						$display("SET_TO_OUT", motor_1_mapped);
						motor_1_rate 		= motor_1_mapped;
						motor_1_rate_last 	= motor_1_mapped;
						end
					/*
					if (motor_2_mapped < `ESC_MIN || motor_2_mapped > `ESC_MAX)
						motor_2_rate 		= motor_2_rate_last;
					else begin
						motor_2_rate 		= motor_2_mapped;
						motor_2_rate_last 	= motor_2_mapped;
						end
					if (motor_3_mapped < `ESC_MIN || motor_3_mapped > `ESC_MAX)
						motor_3_rate 		= motor_3_rate_last;
					else begin
						motor_3_rate 		= motor_3_mapped;
						motor_3_rate_last 	= motor_3_mapped;
						end
					if (motor_4_mapped < `ESC_MIN || motor_4_mapped > `ESC_MAX)
						motor_4_rate 		= motor_4_rate_last;
					else begin
						motor_4_rate 		= motor_4_mapped;
						motor_4_rate_last 	= motor_4_mapped;
						end
					*/ 
					$display("yaw      %d", n_m1_yaw_rate);
					$display("roll     %d", n_m1_roll_rate);
					$display("pitch    %d", n_m1_pitch_rate);
					$display("throttle %d", n_throttle_rate);		  	   
					$display("Motor Rate Output %d", motor_1_output);
					$display("Motor Rate Output (mapped) %d", motor_1_rate);
					state 		= `STATE_CLEAR_VALUES;
					end
				else
					state 		= `STATE_SEND_OUTPUT;
				end	
			`STATE_CLEAR_VALUES: begin
				motor_1_mapped	= `ZERO;
				motor_2_mapped	= `ZERO;
				motor_3_mapped	= `ZERO;
				motor_4_mapped	= `ZERO;
				state 			= `STATE_MAP_16_to_8;
				end
			default begin
				// Should Never Reach the State
				motor_1_rate	= `ZERO;
				/*motor_2_rate	= `ZERO;
				motor_3_rate	= `ZERO;
				motor_4_rate	= `ZERO;*/
				motor_1_mapped	= `ZERO;
				motor_2_mapped	= `ZERO;
				motor_3_mapped	= `ZERO;
				motor_4_mapped	= `ZERO;
				state 			= `STATE_MAP_16_to_8;
				end
		endcase
	end
endmodule