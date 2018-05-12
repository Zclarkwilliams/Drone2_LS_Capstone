`timescale 1ns / 1ns

/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
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

 `include "common_defines.v"

module drone2 (
	output wire motor_1_pwm,
	output wire motor_2_pwm,
	output wire motor_3_pwm,
	output wire motor_4_pwm,
	input wire throttle_pwm,
	input wire yaw_pwm,
	input wire roll_pwm,
	input wire pitch_pwm,
	input wire debug_leds_receiver_throttle_n,
	input wire resetn,
	output wire rstn_imu,
	output reg [7:0]led_data_out,
	inout wire sda,
	inout wire scl);

	/* TODO: Figure out what these bit widths actually need to be
	 *		 and move them to the common_defines.v file.
	 */
	localparam REC_VAL_BIT_WIDTH = 8;   // values from receiver
	localparam PID_RATE_BIT_WIDTH = 16; // values from body frame controller
	localparam RATE_BIT_WIDTH = 16;     // values from angle controller
	localparam IMU_VAL_BIT_WIDTH = 16;  // values from IMU

	// values from receiver to angle_controller
	wire [REC_VAL_BIT_WIDTH-1:0] throttle_val;
	wire [REC_VAL_BIT_WIDTH-1:0] yaw_val;
	wire [REC_VAL_BIT_WIDTH-1:0] roll_val;
	wire [REC_VAL_BIT_WIDTH-1:0] pitch_val;

	// values from angle_controller to body_frame_controller
	wire [RATE_BIT_WIDTH-1:0] throttle_target_rate;
	wire [RATE_BIT_WIDTH-1:0] yaw_target_rate;
	wire [RATE_BIT_WIDTH-1:0] roll_target_rate;
	wire [RATE_BIT_WIDTH-1:0] pitch_target_rate;
	wire [RATE_BIT_WIDTH-1:0] roll_angle_error;
	wire [RATE_BIT_WIDTH-1:0] pitch_angle_error;

	// values from the IMU
	wire [IMU_VAL_BIT_WIDTH-1:0] x_velocity;
	wire [IMU_VAL_BIT_WIDTH-1:0] y_velocity;
	wire [IMU_VAL_BIT_WIDTH-1:0] z_velocity;
	wire [IMU_VAL_BIT_WIDTH-1:0] x_rotation;
	wire [IMU_VAL_BIT_WIDTH-1:0] y_rotation;
	wire [IMU_VAL_BIT_WIDTH-1:0] z_rotation;
	wire [IMU_VAL_BIT_WIDTH-1:0] x_accel;
	wire [IMU_VAL_BIT_WIDTH-1:0] y_accel;
	wire [IMU_VAL_BIT_WIDTH-1:0] z_accel;
	wire [IMU_VAL_BIT_WIDTH-1:0] x_linear_accel;
	wire [IMU_VAL_BIT_WIDTH-1:0] y_linear_accel;
	wire [IMU_VAL_BIT_WIDTH-1:0] z_linear_accel;

	// values from the body_frame_controller to the motor_mixer
	wire [PID_RATE_BIT_WIDTH-1:0] yaw_rate;
	wire [PID_RATE_BIT_WIDTH-1:0] roll_rate;
	wire [PID_RATE_BIT_WIDTH-1:0] pitch_rate;

	// values from motor_mixer to pwm_generator
	wire [`MOTOR_RATE_BIT_WIDTH-1:0] motor_1_rate;
	wire [`MOTOR_RATE_BIT_WIDTH-1:0] motor_2_rate;
	wire [`MOTOR_RATE_BIT_WIDTH-1:0] motor_3_rate;
	wire [`MOTOR_RATE_BIT_WIDTH-1:0] motor_4_rate;

	wire [7:0] imu_debug_out;
	wire imu_good;
	wire imu_valid_strobe;

	// status signals from angle_controller
	wire ac_valid_strobe;
	wire ac_active;

	// status signals from body_frame_controller
	wire bf_valid_strobe;
	wire bf_active;

	wire sys_clk;
	// TODO: Determine if we should stick with this clock rate (slower? faster?)
	defparam OSCH_inst.NOM_FREQ = "38.00";
	OSCH OSCH_inst (.STDBY(1'b0),
       			    .OSC(sys_clk),
       			    .SEDSTDBY());

	us_clk us_clk_divider (
		.us_clk(us_clk),
		.sys_clk(sys_clk),
		.resetn(resetn));

	/*
	bno055_driver	i(
		.scl_1(scl_1),                    //  I2C EFB SDA wires, Primary EFB
		.sda_1(sda_1),                    //  I2C EFB SDA wires, Primary EFB
		.scl_2(scl_2),                    //  I2C EFB SDA wires, Secondary EFB
		.sda_2(sda_2),                    //  I2C EFB SDA wires, Secondary EFB
		.rstn(resetn),                   //  async negative reset signal 0 = reset, 1 = not resete
		.led_data_out(imu_debug_out),     //  Module LED Status output
		.sys_clk(sys_clk),               //  master clock
		.rstn_imu(rstn_imu),             //  Low active reset signal to IMU hardware to trigger reset
		.imu_good(imu_good),             //  The IMU is either in an error or initial bootup states, measurements not yet active
		.valid_strobe(imu_valid_strobe),     //  Strobe signal that indicates the end of the data collection poll, subsequent modules key off this strobe.
		.accel_rate_x(x_accel),          //  Accelerometer X-Axis                Precision: 1 m/s^2 = 100 LSB
		.accel_rate_y(y_accel),          //  Accelerometer Y-Axis                Precision: 1 m/s^2 = 100 LSB
		.accel_rate_z(z_accel),          //  Accelerometer Z-Axis                Precision: 1 m/s^2 = 100 LSB
		.euler_angle_x(x_rotation),      //  Euler angle X-Axis                  Precision: Deg = 16 LSB
		.euler_angle_y(y_rotation),      //  Euler angle Y-Axis                  Precision: Deg = 16 LSB
		.euler_angle_z(z_rotation),      //  Euler angle Z-Axis                  Precision: Deg = 16 LSB
		.linear_accel_x(x_linear_accel), //  Linear Acceleration X-Axis          Precision: 1 m/s^2 = 100 LSB
		.linear_accel_y(y_linear_accel), //  Linear Acceleration Y-Axis          Precision: 1 m/s^2 = 100 LSB
		.linear_accel_z(z_linear_accel), //  Linear Acceleration Z-Axis          Precision: 1 m/s^2 = 100 LSB
		.x_velocity(x_velocity),
		.y_velocity(y_velocity),
		.z_velocity(z_velocity)
		);ode in the pull request reverted a lot of stuff related to the I2C EFB, was that intentional? The rate went back to 50kHz, only one EFB is used, the names were change
	ode in the pull request reverted a lot of stuff related to the I2C EFB, was that intentional? The rate went back to 50kHz, only one EFB is used, the names were change
	*/

	receiver receiver (
		.throttle_val(throttle_val),
		.yaw_val(yaw_val),
		.roll_val(roll_val),
		.pitch_val(pitch_val),
		.throttle_pwm(throttle_pwm),
		.yaw_pwm(yaw_pwm),
		.roll_pwm(roll_pwm),
		.pitch_pwm(pitch_pwm),
		.us_clk(us_clk),
		.resetn(resetn));

	angle_controller #(
		.RATE_BIT_WIDTH(RATE_BIT_WIDTH),
		.REC_VAL_BIT_WIDTH(REC_VAL_BIT_WIDTH),
		.IMU_VAL_BIT_WIDTH(IMU_VAL_BIT_WIDTH))
	angle_controller (
		.throttle_rate_out(throttle_target_rate),
		.yaw_rate_out(yaw_target_rate),
		.roll_rate_out(roll_target_rate),
		.pitch_rate_out(pitch_target_rate),
		.pitch_angle_error(pitch_angle_error),
		.roll_angle_error(roll_angle_error),
		.complete_signal(ac_valid_strobe),
		.active_signal(ac_active),
		.throttle_target(throttle_val),
		.yaw_target(yaw_val),
		.roll_target(roll_val),
		.pitch_target(pitch_val),
		.pitch_actual(16'h0000),  // changed for testing
		.roll_actual(16'h0000),  // changed for testing
		.resetn(resetn),
		.state(state),
		.start_signal(1'b1), // changed for testing
		.us_clk(us_clk));

/*
	body_frame_controller #(
		.PID_RATE_BIT_WIDTH(PID_RATE_BIT_WIDTH),
		.IMU_VAL_BIT_WIDTH(IMU_VAL_BIT_WIDTH),
		.PID_RATE_BIT_WIDTH(PID_RATE_BIT_WIDTH))
	body_frame_controller (
		.yaw_rate_out(yaw_rate),
		.roll_rate_out(roll_rate),
		.pitch_rate_out(pitch_rate),
		.complete_signal(bf_valid_strobe),
		.yaw_target(yaw_target_rate),
		.roll_target(roll_target_rate),
		.pitch_target(pitch_target_rate),
		.roll_rotation(x_rotation),
		.pitch_rotation(y_rotation),
		.yaw_rotation(z_rotation),
		.roll_angle_error(roll_angle_error),
		.pitch_angle_error(pitch_angle_error),
		.start_signal(ac_valid_strobe),
		.resetn(resetn),
		.us_clk(us_clk));
*/

	motor_mixer motor_mixer (
		.motor_1_rate(motor_1_rate),
		.motor_2_rate(motor_2_rate),
		.motor_3_rate(motor_3_rate),
		.motor_4_rate(motor_4_rate),
/*		// test connections
		.throttle_rate({4'h0, throttle_val, 4'h0}),
		.yaw_rate({4'h0, yaw_val, 4'h0}),
		.roll_rate({4'h0, roll_val, 4'h0}),
		.pitch_rate({4'h0, pitch_val, 4'h0}),
*/

		.throttle_rate(throttle_target_rate),
		.yaw_rate(yaw_target_rate),
		.roll_rate(roll_target_rate),
		.pitch_rate(pitch_target_rate),


		.sys_clk(sys_clk),
		.rst_n(resetn));

	pwm_generator pwm_generator (
		.motor_1_pwm(motor_1_pwm),
		.motor_2_pwm(motor_2_pwm),
		.motor_3_pwm(motor_3_pwm),
		.motor_4_pwm(motor_4_pwm),
		.state_out(state_out),

		/*
		.motor_1_rate(throttle_val),
		.motor_2_rate(roll_val),
		.motor_3_rate(pitch_val),
		.motor_4_rate(yaw_val),
		*/

		.motor_1_rate(motor_1_rate),
		.motor_2_rate(motor_2_rate),
		.motor_3_rate(motor_3_rate),
		.motor_4_rate(motor_4_rate),


		.us_clk(us_clk),
		.resetn(resetn));

	// Update on board LEDs, all inputs are active low
	always @(posedge sys_clk) begin
		if (!resetn)
			led_data_out <= 8'hAA;
		else if (!debug_leds_receiver_throttle_n)
			led_data_out <= ~throttle_val;
		else
			led_data_out <= imu_debug_out;
	end


endmodule

