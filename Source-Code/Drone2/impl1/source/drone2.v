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
 * @motor_1_pwm:  signal to drive the ESC connected to motor 1
 * @motor_2_pwm:  signal to drive the ESC connected to motor 2
 * @motor_3_pwm:  signal to drive the ESC connected to motor 3
 * @motor_4_pwm:  signal to drive the ESC connected to motor 4
 * @rstn_imu:	  signal to reset IMU from FPGA
 * @led_data_out: signal mapping data to FPGA board's 8 LEDs
 *
 * Inputs:
 * @yaw_pwm: 	  signal from yaw on the rc/receiver
 * @roll_pwm: 	  signal from roll on the rc/receiver
 * @pitch_pwm: 	  signal from pitch on the rc/receiver
 * @throttle_pwm: signal from throttle on the rc/receiver
 * @aux1_pwm: 	  signal from 1(aux1) on the rc/receiver
 * @aux2_pwm:	  signal from 2(aux2) on the rc/receiver
 * @swa_swb_pwm:  signal from 3(swa/swb) on the rc/receiver
 * @resetn: 	  top level reset signal
 * @led_data_out: connects to the on board LEDs for the MachX03
 *
 * Inouts:
 * @sda 1&2: 	  serial data line to the IMU
 * @scl 1&2: 	  serial clock line to the IMU
 */

`timescale 1ns / 1ns
//`default_nettype none
`include "common_defines.v"

module drone2 (
	// Outputs
	output wire motor_1_pwm,
	output wire motor_2_pwm,
	output wire motor_3_pwm,
	output wire motor_4_pwm,
	output wire rstn_imu,
	output reg  [7:0] led_data_out,
	// Inputs
	input wire throttle_pwm,
	input wire yaw_pwm,
	input wire roll_pwm,
	input wire pitch_pwm,
	input wire aux1_pwm,
	input wire aux2_pwm,
	input wire swa_swb_pwm,
	input wire machxo3_switch_reset_n,
	// Serial IO
	inout wire sda_1,
	inout wire sda_2,
	inout wire scl_1,
	inout wire scl_2,
	
	//UART IO
	input wire sin,
	output wire rxrdy_n,
	output wire sout,
	output wire txrdy_n
	);

	//--------------- Receiver Wires --------------//
	wire [`REC_VAL_BIT_WIDTH-1:0]
		throttle_val,
		yaw_val,
		roll_val,
		pitch_val,
		aux1_val,
		aux2_val,
		swa_swb_val;
		
	//-------- Yaw Angle Accumulator Wires --------//
	wire [`RATE_BIT_WIDTH-1:0]
		yaac_yaw_angle_error,
		yaac_debug,
		yaac_yaw_angle_target;
	wire yaac_active,
		yaw_stick_out_of_neutral_window,
		yaac_complete,
		yaac_enable_n;

	//---------------- IMU Wires ------------------//
	wire [`IMU_VAL_BIT_WIDTH-1:0]
		x_linear_rate,
		y_linear_rate,
		z_linear_rate,
		x_rotation,
		y_rotation,
		z_rotation,
		x_rotation_rate,
		y_rotation_rate,
		z_rotation_rate,
		x_linear_accel,
		y_linear_accel,
		z_linear_accel;
	wire ac_active;
	wire imu_data_valid_monitor;
	wire imu_good;
	wire imu_data_valid;
	wire [7:0] imu_debug;

	//--------- Throttle Controller Wires ---------//
	wire [`REC_VAL_BIT_WIDTH-1:0]
		tc_throttle_value;
	wire throttle_controller_complete,
		throttle_controller_active,
		tc_enable_n;
		
	//---------- Angle_Controller Wires -----------//
	wire [`RATE_BIT_WIDTH-1:0]
		throttle_target_rate,
		yaw_target_rate,
		roll_target_rate,
		pitch_target_rate,
		yaw_angle_error,
		roll_angle_error,
		pitch_angle_error;
	wire ac_valid_strobe;

	//-------- Body_Frame_Controller Wires --------//
	wire [`PID_RATE_BIT_WIDTH-1:0]
		yaw_rate,
		roll_rate,
		pitch_rate;
	wire bf_active;
	wire bf_valid_strobe;

	//------------- Motor_Mixer Wires -------------//
	wire [`MOTOR_RATE_BIT_WIDTH-1:0]
		motor_1_rate,
		motor_2_rate,
		motor_3_rate,
		motor_4_rate;

	//--------------- Clock Wires -----------------//
	wire sys_clk;
	wire us_clk;

	//---------------- Reset Wires ----------------//
	wire resetn;
	//wire soft_reset_n;
	//assign resetn = (machxo3_switch_reset_n & soft_reset_n);
	assign resetn = (machxo3_switch_reset_n);


	/**
	 * Generate System Clock
	 */
	defparam OSCH_inst.NOM_FREQ = "38.00";
	OSCH OSCH_inst (
		.STDBY(1'b0),
       	.OSC(sys_clk),
       	.SEDSTDBY());

	/**
	 * Then scale system clock down to 1 microsecond
	 *		file - us_clk.v
	 */
	us_clk us_clk_divider (
		.us_clk(us_clk),
		.sys_clk(sys_clk),
		.resetn(resetn));

	/**
	 * Gets inputs from the physical receiver and converts them to 0-255.
	 * 		file - receiver.v
	 */
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

	/**
	 * IMU Management and Control Module
	 *		file - bno055_driver.v
	 */
	bno055_driver IMU(
		// Outputs
		.imu_good(imu_good),
		.valid_strobe(imu_data_valid),
		.gyro_rate_x(x_rotation_rate),
		.gyro_rate_y(y_rotation_rate),
		.gyro_rate_z(z_rotation_rate),
		.euler_angle_x(x_rotation),
		.euler_angle_y(y_rotation),
		.euler_angle_z(z_rotation),
		.linear_accel_x(x_linear_accel),
		.linear_accel_y(y_linear_accel),
		.linear_accel_z(z_linear_accel),
		.x_velocity(x_linear_rate),
		.y_velocity(y_linear_rate),
		.z_velocity(z_linear_rate),
		// DEBUG WIRE
		.led_data_out(imu_debug),
		// InOuts
		.scl_1(scl_1),
		.sda_1(sda_1),
		.scl_2(scl_2),
		.sda_2(sda_2),
		// Inputs
		.rstn(resetn),
		.sys_clk(sys_clk),
		.rstn_imu(rstn_imu),
		.next_mod_active(throttle_controller_active));

 	throttle_controller TC(
		.throttle_pwm_value_out(tc_throttle_value),
		.complete_signal(throttle_controller_complete),
		.active_signal(throttle_controller_active),
		.throttle_pwm_value_in(throttle_val),
		.start_signal(imu_data_valid),
		.tc_enable_n(tc_enable_n),
		.imu_ready(imu_good),
		.resetn(resetn),
		.us_clk(us_clk));		
		
	/**
	 *	Takes accumulated yaw PWM value to calculate the current desired
	 *   yaw rotation. Uses the IMU provided yaw Euler angle to calculate an angle
	 *   error from this desired body angle
	 * 		file - yaw_angle_accumulator.v
	 */
	yaw_angle_accumulator  YAAc(
		.yaw_stick_out_of_neutral_window(yaw_stick_out_of_neutral_window),
		.body_yaw_angle_target(yaac_yaw_angle_target),
		.yaw_angle_error_out(yaac_yaw_angle_error),
		.active_signal(yaac_active),
		.complete_signal(yaac_complete),
		.throttle_pwm_value_input(tc_throttle_value),
		.yaw_pwm_value_input(yaw_val),
		.yaw_angle_imu(z_rotation),
		.yaac_enable_n(yaac_enable_n),
		.debug_out(yaac_debug),
		.imu_ready(imu_good),
		.start_signal(throttle_controller_complete),
		.resetn(resetn),
		.us_clk(us_clk)
		);

	/**
	 *	Take IMU provided orientation angle and user provided target angle and
	 *	subtract them to get the error angle rate to get to target angle
	 *	position.
	 * 		file - angle_controller.v
	 */
	angle_controller AC(
		// Outputs
		.throttle_rate_out(throttle_target_rate),
		.yaw_rate_out(yaw_target_rate),
		.roll_rate_out(roll_target_rate),
		.pitch_rate_out(pitch_target_rate),
		.yaw_angle_error(yaw_angle_error),
		.pitch_angle_error(pitch_angle_error),
		.roll_angle_error(roll_angle_error),
		.complete_signal(ac_valid_strobe),
		.active_signal(ac_active),
		// Inputs
		.throttle_target(tc_throttle_value),
		.yaac_enable_n(yaac_enable_n),
		.yaw_angle_target(yaac_yaw_angle_target),
		.yaw_angle_error_in(yaac_yaw_angle_error),
		.roll_target(roll_val),
		.pitch_target(pitch_val),
		.roll_actual(y_rotation),
		.pitch_actual(x_rotation),
		.start_signal(yaac_complete),
		.resetn(resetn),
		.us_clk(us_clk));

	/**
     * Take error rate angles from angle_controller and current rotational
	 * angle rates and feed them into a PID to get corrective control.
	 *		file - body_frame_controller.v
	 */
	body_frame_controller BFC(
		// Outputs
		.yaw_rate_out(yaw_rate),
		.roll_rate_out(roll_rate),
		.pitch_rate_out(pitch_rate),
		.complete_signal(bf_valid_strobe),
		// Inputs
		.yaw_target(yaw_target_rate),
		.roll_target(roll_target_rate),
		.pitch_target(pitch_target_rate),
		.roll_rotation(x_rotation_rate),
		.pitch_rotation(y_rotation_rate),
		//Bypass yaw rotation rate while troubleshooting YAAc
		.yaw_rotation(z_rotation_rate),
		//.yaw_rotation(16'h0000),
		.yaw_angle_error(yaw_angle_error),
		.roll_angle_error(roll_angle_error),
		.pitch_angle_error(pitch_angle_error),
		.start_signal(ac_valid_strobe),
		.resetn(resetn),
		.us_clk(us_clk));

	/**
	 * Get axis rates and calculate respective motor rates to achieve correct
	 * drone movements.
	 *		file - motor_mixer.v
	 */
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

	/**
	 * Take respective motor rate outputs from motor mixer and convert the
	 * 0-250 value to a PWM output to ESCs.
	 *		file - pwm_generator.v
	 */
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
		
		
	uart_top uart
	(
		.resetn(resetn),
		.clk(sys_clk),
		.start(imu_data_valid),
		.sin(sin),
		.rxrdy_n(rxrdy_n),
		.sout(sout),
		.txrdy_n(txrdy_n),

		.imu_x_rotation_angle(x_rotation),
		.imu_y_rotation_angle(y_rotation),
		.imu_z_rotation_angle(z_rotation),
		.imu_x_rotation_rate(x_rotation_rate),
		.imu_y_rotation_rate(y_rotation_rate),
		.imu_z_rotation_rate(z_rotation_rate),
		.imu_calibration_status(imu_debug),
		.rec_throttle_val(tc_throttle_value),
		.rec_yaw_val(yaw_val),
		.rec_roll_val(roll_val),
		.rec_pitch_val(pitch_val),
		.rec_aux1_val(aux1_val),
		.rec_aux2_val(aux2_val),
		.rec_swa_swb_val(swa_swb_val),
		.yaac_yaw_angle_error(yaac_yaw_angle_error),
		.yaac_yaw_angle_target(yaac_yaw_angle_target),
		//.yaac_debug(yaac_debug),
		.yaac_debug(yaw_rate),
		.yaw_stick_out_of_neutral_window(yaw_stick_out_of_neutral_window)

	);
	// Enable bits
	assign yaac_enable_n = `LOW_ACTIVE_ENABLE;  // Enable YAAC
	assign tc_enable_n   = `LOW_ACTIVE_ENABLE;  // Enable TC
	//assign tc_enable_n   = `LOW_ACTIVE_DISABLE; // Disable TC
	//assign soft_reset_n  = `LOW_ACTIVE_DISABLE; // Disable this reset for now, connect if soft reset is needed and remove this line
		
	/**
	 * The section below is for use with Debug LEDs
	 */

	// Update on board LEDs, all inputs are active low
	always @(posedge sys_clk) begin
		if (!resetn) begin
			led_data_out <= 8'hAA;
		end
		else begin
			led_data_out <= ~imu_debug;
		end
	end
endmodule

