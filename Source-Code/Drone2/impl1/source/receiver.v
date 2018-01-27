/**
 * module receiver - receive pwm input from the hardware receiver and output values.
 *					 The pwm inputs have a period of 20ms and a duty cycle rangeing
 *					 between ~5% to ~10% which equals ~1ms to ~2ms.
 *
 * Parameters
 * @N_VAL: Number of bits for throttle/yaw/roll/pitch val outputs
 * @N_AUX: Number of bits for aux1/aux2 val outputs
 * @N_MODE: Number of bits for the mode output
 *
 * Outputs
 * @aux1_val: 2's complement value for aux1 (only valid for -5 to 5)
 * @aux2_val: 2's complement value for aux2 (only valid for -5 to 5)
 * @mode_val: value for the mode (only valid for 0 thru 6)
 * @throttle_val: 2's complement for the throttle value (only valid for -9000 to 9000)
 * @yaw_val: 2's complement for the yaw value (only valid for -9000 to 9000)
 * @roll_val: 2's complement for the roll value (only valid for -9000 to 9000)
 * @pitch_val: 2's complementfor the pitch value (only valid for -9000 to 9000)
 *
 * Inputs
 * @aux1_pwm: from the hardware receiver (duty cycle between ~5% and 10%)
 * @aux2_pwm: from the hardware receiver (duty cycle between ~5% and 10%)
 * @mode_pwm: from the hardware receiver (duty cycle between ~5% and 10%)
 * @throttle_pwm: from the hardware receiver (duty cycle between ~5% and 10%)
 * @yaw_pwm: from the hardware receiver (duty cycle between ~5% and 10%)
 * @roll_pwm: from the hardware receiver (duty cycle between ~5% and 10%)
 * @pitch_pwm: from the hardware receiver (duty cycle between ~5% and 10%)
 * @sys_clk: system clock
 */
module receiver #(parameter N_VAL = 14,
				  parameter N_AUX = 4,
				  parameter N_MODE = 3)
				 (output reg [N_AUX-1:0] aux1_val,
				  output reg [N_AUX-1:0] aux2_val,
				  output reg [N_MODE-1:0] mode_val,
				  output reg [N_VAL-1:0] throttle_val,
				  output reg [N_VAL-1:0] yaw_val,
				  output reg [N_VAL-1:0] roll_val,
				  output reg [N_VAL-1:0] pitch_val,
				  input aux1_pwm,
				  input aux2_pwm,
				  input mode_pwm,
				  input throttle_pwm,
				  input yaw_pwm,
				  input roll_pwm,
				  input pitch_pwm,
				  input sys_clk);							
	
	always @(posedge sys_clk) begin
		if (aux1_pwm || aux2_pwm || mode_pwm || throttle_pwm || yaw_pwm || roll_pwm || pitch_pwm) begin
			aux1_val <= ~aux1_val;
			aux2_val <= ~aux2_val;
			mode_val <= ~mode_val;
			throttle_val <= ~throttle_val;
			yaw_val <= ~yaw_val;
			roll_val <= ~roll_val;
			pitch_val <= ~pitch_val;
		end
	end

endmodule