/**
 * module drone2 - top level module for the drone controller
 *
 * Parameters
 * @N_RATE: Number of bits for the motor rate
 *
 * Outputs
 * @motor_1_pwm: signal to drive the ESC connected to motor 1
 * @motor_2_pwm: signal to drive the ESC connected to motor 2
 * @motor_3_pwm: signal to drive the ESC connected to motor 3
 * @motor_4_pwm: signal to drive the ESC connected to motor 4
 *
 * Inputs
 * @aux1_pwm: signal from aux1 on the rc/receiver
 * @aux2_pwm: signal from aux2 on the rc/receiver
 * @mode_pwm: signal from mode switches on the rc/receiver
 * @throttle_pwm: signal from throttle on the rc/receiver
 * @yaw_pwm: signal from yaw on the rc/receiver
 * @roll_pwm: signal from roll on the rc/receiver
 * @pitch_pwm: signal from pitch on the rc/receiver
 * @sda: serial data line to the IMU
 * @scl: serial clock line to the IMU
 */
module drone2 (output wire motor_1_pwm,
			   output wire motor_2_pwm,
			   output wire motor_3_pwm,
			   output wire motor_4_pwm,
			   input aux1_pwm,
			   input aux2_pwm,
			   input mode_pwm,
			   input throttle_pwm,
			   input yaw_pwm,
			   input roll_pwm,
			   input pitch_pwm,
			   input sda,
			   input scl);

	localparam N_REC_VAL = 14;
	localparam N_AUX_VAL = 4;
	localparam N_MODE_VAL = 3;
	localparam N_PID_RATE = 36;
	localparam N_RATE = 36;
	localparam N_IMU_VAL = 36;
	localparam N_MOTOR_RATE = 36;

	wire [N_AUX_VAL-1:0] aux1_val;
	wire [N_AUX_VAL-1:0] aux2_val;
	wire [N_MODE_VAL-1:0] mode_val;

	wire [N_REC_VAL-1:0] throttle_val;
	wire [N_REC_VAL-1:0] yaw_val;
	wire [N_REC_VAL-1:0] roll_val;
	wire [N_REC_VAL-1:0] pitch_val;

	wire [N_PID_RATE-1:0] throttle_target_rate;
	wire [N_PID_RATE-1:0] yaw_target_rate;
	wire [N_PID_RATE-1:0] roll_target_rate;
	wire [N_PID_RATE-1:0] pitch_target_rate;

	wire [N_IMU_VAL-1:0] x_velocity;
	wire [N_IMU_VAL-1:0] y_velocity;
	wire [N_IMU_VAL-1:0] z_velocity;
	wire [N_IMU_VAL-1:0] x_rotation;
	wire [N_IMU_VAL-1:0] y_rotation;
	wire [N_IMU_VAL-1:0] z_rotation;
	wire [N_IMU_VAL-1:0] x_accel;
	wire [N_IMU_VAL-1:0] y_accel;
	wire [N_IMU_VAL-1:0] z_accel;

	wire [N_PID_RATE-1:0] yaw_rate;
	wire [N_PID_RATE-1:0] roll_rate;
	wire [N_PID_RATE-1:0] pitch_rate;

	wire [N_MOTOR_RATE-1:0] motor_1_rate;
	wire [N_MOTOR_RATE-1:0] motor_2_rate;
	wire [N_MOTOR_RATE-1:0] motor_3_rate;
	wire [N_MOTOR_RATE-1:0] motor_4_rate;

	defparam OSCH_inst.NOM_FREQ = "53.20";
	OSCH OSCH_inst (.STDBY(1'b0),
       			    .OSC(sys_clk),		// Connect the clock to the osc_clk wire.
       			    .SEDSTDBY());


	receiver #(N_REC_VAL, N_AUX_VAL, N_MODE_VAL) r(.aux1_val(aux1_val),
				  								   .aux2_val(aux2_val),
				  								   .mode_val(mode_val),
				  								   .throttle_val(throttle_val),
				  								   .yaw_val(yaw_val),
				  								   .roll_val(roll_val),
				  								   .pitch_val(pitch_val),
												   .aux1_pwm(aux1_pwm),
												   .aux2_pwm(aux2_pwm),
												   .mode_pwm(mode_pwm),
												   .throttle_pwm(throttle_pwm),
												   .yaw_pwm(yaw_pwm),
												   .roll_pwm(roll_pwm),
												   .pitch_pwm(pitch_pwm),
												   .sys_clk(sys_clk));

	value_to_rate #(N_RATE, N_REC_VAL) v(.throttle_rate(throttle_target_rate),
										 .yaw_rate(yaw_target_rate),
										 .roll_rate(roll_target_rate),
										 .pitch_rate(pitch_target_rate),
										 .throttle_val(throttle_val),
										 .yaw_val(yaw_val),
										 .roll_val(roll_val),
										 .pitch_val(pitch_val),
										 .sys_clk(sys_clk));

	imu #(N_IMU_VAL, N_IMU_VAL, N_IMU_VAL) i(.x_velocity(x_velocity),
											 .y_velocity(y_velocity),
											 .z_velocity(z_velocity),
											 .x_rotation(x_rotation),
											 .y_rotation(y_rotation),
											 .z_rotation(z_rotation),
											 .x_accel(x_accel),
											 .y_accel(y_accel),
											 .z_accel(z_accel),
											 .sda(sda),
											 .scl(scl),
											 .sys_clk(sys_clk));

	pid #(N_PID_RATE, N_IMU_VAL, N_IMU_VAL) p(.yaw_rate_out(yaw_rate),
											  .roll_rate_out(roll_rate),
											  .pitch_rate_out(pitch_rate),
											  .yaw_rate_in(yaw_target_rate),
											  .roll_rate_in(roll_target_rate),
											  .pitch_rate_in(pitch_target_rate),
											  .x_velocity(x_velocity),
											  .y_velocity(y_velocity),
											  .z_velocity(z_velocity),
											  .x_rotation(x_rotation),
											  .y_rotation(y_rotation),
											  .z_rotation(z_rotation),
											  .sys_clk(sys_clk));

	pid_mixer #(N_PID_RATE, N_MOTOR_RATE) pm(.motor_1_rate(motor_1_rate),
											 .motor_2_rate(motor_2_rate),
											 .motor_3_rate(motor_3_rate),
											 .motor_4_rate(motor_4_rate),
											 .throttle_rate(throttle_target_rate),
											 .yaw_rate(yaw_rate),
											 .roll_rate(roll_rate),
											 .pitch_rate(pitch_rate),
											 .sys_clk(sys_clk));

	pwm_generator #(N_MOTOR_RATE) pg(.motor_1_pwm(motor_1_pwm),
									 .motor_2_pwm(motor_2_pwm),
									 .motor_3_pwm(motor_3_pwm),
									 .motor_4_pwm(motor_4_pwm),
									 .motor_1_rate(motor_1_rate),
									 .motor_2_rate(motor_2_rate),
									 .motor_3_rate(motor_3_rate),
									 .motor_4_rate(motor_4_rate),
									 .sys_clk(sys_clk));

	endmodule
