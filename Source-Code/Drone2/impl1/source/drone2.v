`timescale 1ns / 1ns

/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

/**
 * drone2 - Top level module for the drone controller.
 *
 * Outputs
 * @motor_1_pwm: signal to drive the ESC connected to motor 1
 * @motor_2_pwm: signal to drive the ESC connected to motor 2
 * @motor_3_pwm: signal to drive the ESC connected to motor 3
 * @motor_4_pwm: signal to drive the ESC connected to motor 4
 *
 * Inputs
 * @throttle_pwm: signal from throttle on the rc/receiver
 * @yaw_pwm: signal from yaw on the rc/receiver
 * @roll_pwm: signal from roll on the rc/receiver
 * @pitch_pwm: signal from pitch on the rc/receiver
 * @resetn: top level reset signal
 * @sda: serial data line to the IMU
 * @scl: serial clock line to the IMU
 */

 `include "common_defines.v"

module drone2 (output wire motor_1_pwm,
			   output wire motor_2_pwm,
			   output wire motor_3_pwm,
			   output wire motor_4_pwm,
			   input wire throttle_pwm,
			   input wire yaw_pwm,
			   input wire roll_pwm,
			   input wire pitch_pwm,
			   input wire resetn,
			   output wire rstn_imu,
			   output wire [7:0]led_data_out,
			   inout wire sda,
			   inout wire scl);

	/* TODO: Figure out what these bit widths actually need to be
	 *		 and move them to the common_defines.v file.
	 */
	localparam REC_VAL_BIT_WIDTH = 8;
	localparam PID_RATE_BIT_WIDTH = 36;
	localparam RATE_BIT_WIDTH = 36;
	localparam IMU_VAL_BIT_WIDTH = 16;

	wire [REC_VAL_BIT_WIDTH-1:0] throttle_val;
	wire [REC_VAL_BIT_WIDTH-1:0] yaw_val;
	wire [REC_VAL_BIT_WIDTH-1:0] roll_val;
	wire [REC_VAL_BIT_WIDTH-1:0] pitch_val;

	wire [PID_RATE_BIT_WIDTH-1:0] throttle_target_rate;
	wire [PID_RATE_BIT_WIDTH-1:0] yaw_target_rate;
	wire [PID_RATE_BIT_WIDTH-1:0] roll_target_rate;
	wire [PID_RATE_BIT_WIDTH-1:0] pitch_target_rate;

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

	wire [PID_RATE_BIT_WIDTH-1:0] yaw_rate;
	wire [PID_RATE_BIT_WIDTH-1:0] roll_rate;
	wire [PID_RATE_BIT_WIDTH-1:0] pitch_rate;

	wire [`MOTOR_RATE_BIT_WIDTH-1:0] motor_1_rate;
	wire [`MOTOR_RATE_BIT_WIDTH-1:0] motor_2_rate;
	wire [`MOTOR_RATE_BIT_WIDTH-1:0] motor_3_rate;
	wire [`MOTOR_RATE_BIT_WIDTH-1:0] motor_4_rate;
	
	wire imu_good;
	wire valid_strobe;

	// TODO: Replace modules using this with the top level reset_n later on
	wire temp_reset_n;
	assign temp_reset_n = `HIGH;

	wire sys_clk;
	// TODO: Determine if we should stick with this clock rate (slower? faster?)
	defparam OSCH_inst.NOM_FREQ = "38.00";
	OSCH OSCH_inst (.STDBY(1'b0),
       			    .OSC(sys_clk),
       			    .SEDSTDBY());

	us_clk us_clk_divider (.us_clk(us_clk),
						   .sys_clk(sys_clk),
						   .resetn(temp_reset_n));  // TODO: Change this to the top level reset signal (.resetn(resetn))

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
		.resetn(temp_reset_n)); // TODO: Change this to the top level reset signal (.resetn(resetn))

	pwm_generator pwm_generator (
		.motor_1_pwm(motor_1_pwm),
		.motor_2_pwm(motor_2_pwm),
		.motor_3_pwm(motor_3_pwm),
		.motor_4_pwm(motor_4_pwm),
		.motor_1_rate(motor_1_rate),
		.motor_2_rate(motor_2_rate),
		.motor_3_rate(motor_3_rate),
		.motor_4_rate(motor_4_rate),
		.us_clk(us_clk),
		.resetn(temp_reset_n)); // TODO: Change this to the top level reset signal (.resetn(resetn))

	// TODO: Rename this block appropriately (I think we mentioned this is the initial p controller block)
	value_to_rate #(RATE_BIT_WIDTH, REC_VAL_BIT_WIDTH) value_to_rate (
		.throttle_rate(throttle_target_rate),
		.yaw_rate(yaw_target_rate),
		.roll_rate(roll_target_rate),
		.pitch_rate(pitch_target_rate),
		.throttle_val(throttle_val),
		.yaw_val(yaw_val),
		.roll_val(roll_val),
		.pitch_val(pitch_val),
		.sys_clk(sys_clk));

	bno055_driver	i(
		.scl1(scl),                      //  I2C EFB SDA wires
		.sda1(sda),                      //  I2C EFB SDA wires
		.rstn(resetn),                   //  async negative reset signal 0 = reset, 1 = not resete
		.led_data_out(led_data_out),     //  Module LED Status output
		.sys_clk(sys_clk),               //  master clock
		.rstn_imu(rstn_imu),             //  Low active reset signal to IMU hardware to trigger reset
		.imu_good(imu_good),             //  The IMU is either in an error or initial bootup states, measurements not yet active
		.valid_strobe(valid_strobe),     //  Strobe signal that indicates the end of the data collection poll, subsequent modules key off this strobe.
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
		);


	// TODO: Name this block more appropriately (more descriptive)
	pid #(PID_RATE_BIT_WIDTH, IMU_VAL_BIT_WIDTH, IMU_VAL_BIT_WIDTH) pid (
		.yaw_rate_out(yaw_rate),
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

	// TODO: Figure out if this is the correct name for this block still
	pid_mixer #(PID_RATE_BIT_WIDTH, `MOTOR_RATE_BIT_WIDTH) pid_mixer (
		.motor_1_rate(motor_1_rate),
		.motor_2_rate(motor_2_rate),
		.motor_3_rate(motor_3_rate),
		.motor_4_rate(motor_4_rate),
		.throttle_rate(throttle_target_rate),
		.yaw_rate(yaw_rate),
		.roll_rate(roll_rate),
		.pitch_rate(pitch_rate),
		.sys_clk(sys_clk));

endmodule

