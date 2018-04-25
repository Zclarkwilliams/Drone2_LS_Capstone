/*
*
*
*
*
*
*
*
*
*/

/*
	Top level instantiation of module
	
	pid_mixer #(
				PID_RATE_BIT_WIDTH, 
				`MOTOR_RATE_BIT_WIDTH) 
	pid_mixer	(
				.motor_1_rate(motor_1_rate),
				.motor_2_rate(motor_2_rate),
				.motor_3_rate(motor_3_rate),
				.motor_4_rate(motor_4_rate),
				.throttle_rate(throttle_target_rate),
				.yaw_rate(yaw_rate),
				.roll_rate(roll_rate),
				.pitch_rate(pitch_rate),
				.sys_clk(sys_clk)
				);
*/


module pid_mixer #(
					parameter PID_RATE_BIT_WIDTH = 16, 
					parameter MOTOR_RATE_BIT_WIDTH = 8
				  )
				  (
					output	reg		motor_1_rate,			
					output	reg		motor_2_rate,
					output	reg		motor_3_rate,
					output	reg		motor_4_rate,
					input 	wire	throttle_target_rate,
					input 	wire	yaw_rate,
					input 	wire	roll_rate,
					input 	wire	pitch_rate,
					input 	wire	sys_clk,
					input 	wire	rst_n
				  );




endmodule