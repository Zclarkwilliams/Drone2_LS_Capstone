/**
 * module pwm_generator - take motor rates and map them to pwm
 *
 * Parameters
 * @N_RATE: Number of bits for the motor rate
 *
 * Outputs
 * @motor_1_pwm: signal to drive motor 1
 * @motor_2_pwm: signal to drive motor 2
 * @motor_3_pwm: signal to drive motor 3
 * @motor_4_pwm: signal to drive motor 4
 * 
 * Inputs
 * @motor_1_rate: rate to run motor 1 at (units?) 
 * @motor_2_rate: rate to run motor 2 at (units?)
 * @motor_3_rate: rate to run motor 3 at (units?)
 * @motor_4_rate: rate to run motor 4 at (units?)
 */
 module pwm_generator #(parameter N_RATE = 36)
					   (output reg motor_1_pwm,
					   	output reg motor_2_pwm,
					   	output reg motor_3_pwm,
					   	output reg motor_4_pwm,
					   	input [N_RATE-1:0] motor_1_rate,
					    input [N_RATE-1:0] motor_2_rate,
					    input [N_RATE-1:0] motor_3_rate,
					    input [N_RATE-1:0] motor_4_rate,
					    input sys_clk);

	always @(posedge sys_clk) begin
		if (motor_1_rate || motor_2_rate || motor_3_rate || motor_4_rate) begin
			motor_1_pwm <= ~motor_1_pwm;
			motor_2_pwm <= ~motor_2_pwm;
			motor_3_pwm <= ~motor_3_pwm;
			motor_4_pwm <= ~motor_4_pwm;
		end
	end

endmodule