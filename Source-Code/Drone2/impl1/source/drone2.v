/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, 
 * Brett Creeley, 
 * Daniel Christiansen, 
 * Kirk Hooper, 
 * Zachary Clark-Williams
 */

/**
 * drone2 - Top level module for the drone controller.
 *
 * Outputs:
 * @motor_1_pwm: signal to drive the ESC connected to motor 1
 * @motor_2_pwm: signal to drive the ESC connected to motor 2
 * @motor_3_pwm: signal to drive the ESC connected to motor 3
 * @motor_4_pwm: signal to drive the ESC connected to motor 4
 *
 * Inputs:
 * @throttle_pwm: signal from throttle on the rc/receiver
 * @yaw_pwm: signal from yaw on the rc/receiver
 * @roll_pwm: signal from roll on the rc/receiver
 * @pitch_pwm: signal from pitch on the rc/receiver
 * @resetn: top level reset signal
 * @led_data_out: connects to the on board LEDs for the MachX03
 *
 * Inouts:
 * @sda: serial data line to the IMU
 * @scl: serial clock line to the IMU
 */

`timescale 1ns / 1ns
`default_nettype none
`include "common_defines.v"

module drone2 (
	// Outputs
	output wire motor_1_pwm,
	output wire motor_2_pwm,
	output wire motor_3_pwm,
	output wire motor_4_pwm,
	output wire rstn_imu,
	output reg [7:0] led_data_out,
	// Inputs
	input wire throttle_pwm,
	input wire yaw_pwm,
	input wire roll_pwm,
	input wire pitch_pwm,
	input wire aux1_pwm,
	input wire aux2_pwm,
	input wire swa_swb_pwm,
	input wire machxo3_switch_reset_n,
	// DEBUG IO
	input wire DEBUG_LED_SWITCH_N,
	output reg [15:0] DEBUG_LEDs,
	output wire imu_data_valid_monitor,
	// Serial IO
	inout wire sda_1,
	inout wire sda_2,
	inout wire scl_1,
	inout wire scl_2);

	// values from receiver to angle_controller
	wire [`REC_VAL_BIT_WIDTH-1:0] throttle_val;
	wire [`REC_VAL_BIT_WIDTH-1:0] yaw_val;
	wire [`REC_VAL_BIT_WIDTH-1:0] roll_val;
	wire [`REC_VAL_BIT_WIDTH-1:0] pitch_val;
	wire [`REC_VAL_BIT_WIDTH-1:0] aux1_val;
	wire [`REC_VAL_BIT_WIDTH-1:0] aux2_val;
	wire [`REC_VAL_BIT_WIDTH-1:0] swa_swb_val;

	// values from angle_controller to body_frame_controller
	wire [`RATE_BIT_WIDTH-1:0] throttle_target_rate;
	wire [`RATE_BIT_WIDTH-1:0] yaw_target_rate;
	wire [`RATE_BIT_WIDTH-1:0] roll_target_rate;
	wire [`RATE_BIT_WIDTH-1:0] pitch_target_rate;
	wire [`RATE_BIT_WIDTH-1:0] roll_angle_error;
	wire [`RATE_BIT_WIDTH-1:0] pitch_angle_error;

	// values from the IMU
	wire [`IMU_VAL_BIT_WIDTH-1:0] x_linear_rate;
	wire [`IMU_VAL_BIT_WIDTH-1:0] y_linear_rate;
	wire [`IMU_VAL_BIT_WIDTH-1:0] z_linear_rate;
	wire [`IMU_VAL_BIT_WIDTH-1:0] x_rotation;
	wire [`IMU_VAL_BIT_WIDTH-1:0] y_rotation;
	wire [`IMU_VAL_BIT_WIDTH-1:0] z_rotation;
	wire [`IMU_VAL_BIT_WIDTH-1:0] x_rotation_rate;
	wire [`IMU_VAL_BIT_WIDTH-1:0] y_rotation_rate;
	wire [`IMU_VAL_BIT_WIDTH-1:0] z_rotation_rate;
	wire [`IMU_VAL_BIT_WIDTH-1:0] x_linear_accel;
	wire [`IMU_VAL_BIT_WIDTH-1:0] y_linear_accel;
	wire [`IMU_VAL_BIT_WIDTH-1:0] z_linear_accel;

	// values from the body_frame_controller to the motor_mixer
	wire [`PID_RATE_BIT_WIDTH-1:0] yaw_rate;
	wire [`PID_RATE_BIT_WIDTH-1:0] roll_rate;
	wire [`PID_RATE_BIT_WIDTH-1:0] pitch_rate;

	// values from motor_mixer to pwm_generator
	wire [`MOTOR_RATE_BIT_WIDTH-1:0] motor_1_rate;
	wire [`MOTOR_RATE_BIT_WIDTH-1:0] motor_2_rate;
	wire [`MOTOR_RATE_BIT_WIDTH-1:0] motor_3_rate;
	wire [`MOTOR_RATE_BIT_WIDTH-1:0] motor_4_rate;

	wire [7:0] imu_debug_out;
	wire imu_good;
	wire imu_data_valid;

	// status signals from angle_controller
	wire ac_valid_strobe;
	wire ac_active;

	// status signals from body_frame_controller
	wire [15:0] bfc_debug_wire;
	wire bf_valid_strobe;
	wire bf_active;

	// clock signals
	wire sys_clk;
	wire us_clk;

	// reset signals coupled together
	wire resetn;
	wire soft_reset_n;
	assign resetn = (machxo3_switch_reset_n | soft_reset_n);

	// TODO: Determine if we should stick with this clock rate (slower? faster?)
	defparam OSCH_inst.NOM_FREQ = "38.00";
	OSCH OSCH_inst (.STDBY(1'b0),
       			    .OSC(sys_clk),
       			    .SEDSTDBY());

	us_clk us_clk_divider (
		.us_clk(us_clk),
		.sys_clk(sys_clk),
		.resetn(resetn));

	bno055_driver imu (
		.scl_1(scl_1),                    //  I2C EFB SDA wires, Primary EFB
		.sda_1(sda_1),                    //  I2C EFB SDA wires, Primary EFB
		.scl_2(scl_2),                    //  I2C EFB SDA wires, Secondary EFB
		.sda_2(sda_2),                    //  I2C EFB SDA wires, Secondary EFB
		.rstn(resetn),                    //  async negative reset signal 0 = reset, 1 = not resete
		.led_data_out(imu_debug_out), 	  //  Module LED Status output
		.sys_clk(sys_clk),                //  master clock
		.rstn_imu(rstn_imu),              //  Low active reset signal to IMU hardware to trigger reset
		.imu_good(imu_good),              //  The IMU is either in an error or initial bootup states, measurements not yet active
		.valid_strobe(imu_data_valid), 	  //  Bit that indicates that the IMU data presented is valid, deasserted momentarily at 10ms polling interval and IMU burst read completion
		.gyro_rate_x(x_rotation_rate),    //  Rotation rate on X-Axis (Pitch rate)Precision: 1 Dps = 16 LSB
		.gyro_rate_y(y_rotation_rate),    //  Rotation rate on Y-Axis (Roll rate) Precision: 1 Dps = 16 LSB
		.gyro_rate_z(z_rotation_rate),    //  Rotation rate on Z-Axis (Yaw rate)  Precision: 1 Dps = 16 LSB
		.euler_angle_x(x_rotation),       //  Euler angle X-Axis       Pitch      Precision: 1 Deg = 16 LSB
		.euler_angle_y(y_rotation),       //  Euler angle Y-Axis       Roll       Precision: 1 Deg = 16 LSB
		.euler_angle_z(z_rotation),       //  Euler angle Z-Axis       Yaw        Precision: 1 Deg = 16 LSB
		.linear_accel_x(x_linear_accel),  //  Linear Acceleration X-Axis          Precision: 1 m/s^2 = 100 LSB
		.linear_accel_y(y_linear_accel),  //  Linear Acceleration Y-Axis          Precision: 1 m/s^2 = 100 LSB
		.linear_accel_z(z_linear_accel),  //  Linear Acceleration Z-Axis          Precision: 1 m/s^2 = 100 LSB
		.x_velocity(x_linear_rate),
		.y_velocity(y_linear_rate),
		.z_velocity(z_linear_rate),
		.ac_active(ac_active));

	receiver receiver (
		// Outputs
		.throttle_val(throttle_val),
		.yaw_val(yaw_val),
		.roll_val(roll_val),
		.pitch_val(pitch_val),
		.aux1_val(aux1_val),
		.aux2_val(aux2_val),
		.swa_swb_val(swa_swb_val),
		// Inputs
		.throttle_pwm(throttle_pwm),
		.yaw_pwm(yaw_pwm),
		.roll_pwm(roll_pwm),
		.pitch_pwm(pitch_pwm),
		.aux1_pwm(aux1_pwm),
		.aux2_pwm(aux2_pwm),
		.swa_swb_pwm(swa_swb_pwm),
		.us_clk(us_clk),
		.resetn(resetn));

	angle_controller ac (
		// Outputs
		.throttle_rate_out(throttle_target_rate),
		.yaw_rate_out(yaw_target_rate),
		.roll_rate_out(roll_target_rate),
		.pitch_rate_out(pitch_target_rate),
		.pitch_angle_error(pitch_angle_error),
		.roll_angle_error(roll_angle_error),
		.complete_signal(ac_valid_strobe),
		.active_signal(ac_active),
		// Inputs
		.throttle_target(throttle_val),
		.yaw_target(yaw_val),
		.roll_target(roll_val),
		.pitch_target(pitch_val),
		.yaw_actual(z_rotation),
		.roll_actual(y_rotation),
		.pitch_actual(x_rotation),
		.start_signal(1'b1),//imu_data_valid),
		.resetn(resetn),
		.us_clk(us_clk));

	body_frame_controller bfc (
		// Outpus
		.yaw_rate_out(yaw_rate),
		.roll_rate_out(roll_rate),
		.pitch_rate_out(pitch_rate),
		.complete_signal(bf_valid_strobe),
		// Debug LED wire
		.DEBUG_WIRE(bfc_debug_wire),
		// Inputs
		.yaw_target(yaw_target_rate),
		.roll_target(roll_target_rate),
		.pitch_target(pitch_target_rate),
		.roll_rotation(y_rotation_rate),
		.pitch_rotation(x_rotation_rate),
		.yaw_rotation(z_rotation_rate),
		.roll_angle_error(roll_angle_error),
		.pitch_angle_error(pitch_angle_error),
		.start_signal(ac_valid_strobe),
		.resetn(resetn),
		.us_clk(us_clk));

	motor_mixer motor_mixer (
		// Outputs
		.motor_1_rate(motor_1_rate),
		.motor_2_rate(motor_2_rate),
		.motor_3_rate(motor_3_rate),
		.motor_4_rate(motor_4_rate),
		// Inputs
		.throttle_rate(throttle_target_rate),
		.yaw_rate(yaw_rate),
		.roll_rate(roll_rate),
		.pitch_rate(pitch_rate),
		.sys_clk(sys_clk),
		.resetn(resetn));

	pwm_generator pwm_generator (
		// Outputs
		.motor_1_pwm(motor_1_pwm),
		.motor_2_pwm(motor_2_pwm),
		.motor_3_pwm(motor_3_pwm),
		.motor_4_pwm(motor_4_pwm),
		// Inputs
		.motor_1_rate(motor_1_rate),
		.motor_2_rate(motor_2_rate),
		.motor_3_rate(motor_3_rate),
		.motor_4_rate(motor_4_rate),
		.us_clk(us_clk),
		.resetn(resetn));

	assign imu_data_valid_monitor = imu_data_valid;

	// Update on board LEDs, all inputs are active low
	always @(posedge sys_clk) begin
		if (!resetn) begin
			led_data_out <= 8'hAA;
			DEBUG_LEDs	 <= 16'hAAAA;
			end
		else if (!DEBUG_LED_SWITCH_N) begin
			led_data_out <= ~throttle_val;
			DEBUG_LEDs	 <= throttle_target_rate;
			end
		else begin
			led_data_out <= ~roll_val;
			DEBUG_LEDs	 <= y_rotation;
			end
	end


endmodule

