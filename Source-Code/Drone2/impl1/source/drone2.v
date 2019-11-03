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
 * @resetn_imu:     signal to reset IMU from FPGA
 * @led_data_out: signal mapping data to FPGA board's 8 LEDs
 *
 * Inputs:
 * @yaw_pwm:      signal from yaw on the rc/receiver
 * @roll_pwm:     signal from roll on the rc/receiver
 * @pitch_pwm:    signal from pitch on the rc/receiver
 * @throttle_pwm: signal from throttle on the rc/receiver
 * @aux1_pwm:     signal from 1(aux1) on the rc/receiver
 * @aux2_pwm:     signal from 2(aux2) on the rc/receiver
 * @swa_swb_pwm:  signal from 3(swa/swb) on the rc/receiver
 * @resetn:       top level reset signal
 * @led_data_out: connects to the on board LEDs for the MachX03
 *
 * Inouts:
 * @sda 1&2:      serial data line to the IMU
 * @scl 1&2:      serial clock line to the IMU
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
    output wire resetn_imu,
    //output wire resetn_lidar,
    output wire resetn_lidar,
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
    input  wire sin,
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
        next_x_rotation_angle,
        next_y_rotation_angle,
        next_z_rotation_angle,
        next_x_rotation_rate,
        next_y_rotation_rate,
        next_z_rotation_rate,
        next_x_linear_accel,
        next_y_linear_accel,
        next_z_linear_accel,
        next_gravity_accel_x,
        next_gravity_accel_y,
        next_gravity_accel_z,
        next_quaternion_data_w,
        next_quaternion_data_x,
        next_quaternion_data_y,
        next_quaternion_data_z,
        next_accel_rate_x,
        next_accel_rate_y,
        next_accel_rate_z,
        next_magneto_rate_x,
        next_magneto_rate_y,
        next_magneto_rate_z,
        next_VL53L1X_chip_id,
        next_VL53L1X_range_mm;
    wire [`REC_VAL_BIT_WIDTH-1:0]      
        next_temperature,
        next_calib_status,
        next_VL53L1X_firm_rdy,
        next_VL53L1X_data_rdy,
        next_i2c_driver_debug,
        next_i2c_top_debug;
        
    reg [`IMU_VAL_BIT_WIDTH-1:0]
        x_rotation_angle,
        y_rotation_angle,
        z_rotation_angle,
        x_rotation_rate,
        y_rotation_rate,
        z_rotation_rate,
        x_linear_accel,
        y_linear_accel,
        z_linear_accel,
        gravity_accel_x,
        gravity_accel_y,
        gravity_accel_z,
        quaternion_data_w,
        quaternion_data_x,
        quaternion_data_y,
        quaternion_data_z,
        accel_rate_x,
        accel_rate_y,
        accel_rate_z,
        magneto_rate_x,
        magneto_rate_y,
        magneto_rate_z,
        VL53L1X_chip_id,
        VL53L1X_range_mm;
        
    reg [`REC_VAL_BIT_WIDTH-1:0]      
        temperature,
        calib_status,
        VL53L1X_firm_rdy,
        VL53L1X_data_rdy,
        i2c_driver_debug,
        i2c_top_debug;
    wire ac_active;
    reg  imu_good;
    wire next_imu_good;
    reg  imu_data_valid;
    wire next_imu_data_valid;
    
    
    wire trash;
    
    //--------- Auto Mode Controller Wires --------//
    wire  [`REC_VAL_BIT_WIDTH-1:0] amc_throttle_val;
    wire  amc_active_signal;
    wire  amc_complete_signal;
    wire  [15:0] amc_debug;

    //--------- Throttle Controller Wires ---------//
    wire [`REC_VAL_BIT_WIDTH-1:0]
        tc_throttle_val;
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
    //wire resetn;
    reg  resetn;

    
    

    //---------------- Mode Selector Wires ----------------//
    wire [2:0] switch_a;
    wire [1:0] switch_b;
    
    wire [15:0] z_linear_velocity;

    assign z_linear_velocity = 0;

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
     *        file - us_clk.v
     */
    us_clk us_clk_divider (
        .us_clk(us_clk),
        .sys_clk(sys_clk),
        .resetn(resetn));

    /**
     * Gets inputs from the physical receiver and converts them to 0-255.
     *         file - receiver.v
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
        
    /*
    *  Determine running flight mode from mode selector switches a and b
    */    
    flight_mode MODE(
        .swa_swb_val(swa_swb_val),
        .switch_a(switch_a),
        .switch_b(switch_b),
        .resetn(resetn),
        .us_clk(us_clk)
    );


    /**
     * IMU Management and Control Module
     *        file - bno055_driver.v
     */
    i2c_device_driver #(
                        //.INIT_INTERVAL(16'd10_000),
                        .INIT_INTERVAL(16'd1_000),
                        .POLL_INTERVAL(16'd20))
        I2C_Devices(
        // Outputs
        .imu_good(next_imu_good),
        .accel_rate_x(next_accel_rate_x),
        .accel_rate_y(next_accel_rate_y),
        .accel_rate_z(next_accel_rate_z),
        .magneto_rate_x(next_magneto_rate_x),
        .magneto_rate_y(next_magneto_rate_y),
        .magneto_rate_z(next_magneto_rate_z),
        .gyro_rate_x(next_x_rotation_rate),
        .gyro_rate_y(next_y_rotation_rate),
        .gyro_rate_z(next_z_rotation_rate),
        .euler_angle_x(next_x_rotation_angle),
        .euler_angle_y(next_y_rotation_angle),
        .euler_angle_z(next_z_rotation_angle),
        .quaternion_data_w(next_quaternion_data_w),
        .quaternion_data_x(next_quaternion_data_x),
        .quaternion_data_y(next_quaternion_data_y),
        .quaternion_data_z(next_quaternion_data_z),
        .linear_accel_x(next_x_linear_accel),
        .linear_accel_y(next_y_linear_accel),
        .linear_accel_z(next_z_linear_accel),
        .gravity_accel_x(next_gravity_accel_x),
        .gravity_accel_y(next_gravity_accel_y),
        .gravity_accel_z(next_gravity_accel_z),
        .temperature(next_temperature),
        .calib_status(next_calib_status),
        .valid_strobe(next_imu_data_valid),
        .VL53L1X_chip_id(next_VL53L1X_chip_id),
        .VL53L1X_range_mm(next_VL53L1X_range_mm),
        .VL53L1X_firm_rdy(next_VL53L1X_firm_rdy),
        .VL53L1X_data_rdy(next_VL53L1X_data_rdy),
        // DEBUG WIRE
        .led_data_out(next_i2c_driver_debug),
        .i2c_top_debug(next_i2c_top_debug),
        // InOuts
        .scl_1(scl_1),
        .sda_1(sda_1),
        .scl_2(scl_2),
        .sda_2(sda_2),
        // Inputs
        .resetn(resetn),
        .sys_clk(sys_clk),
        .resetn_imu(resetn_imu),
        .resetn_lidar(trash),
        //.resetn_lidar(resetn_lidar),
        .next_mod_active(throttle_controller_active)
    );

        
    auto_mode_controller AMC (
        .debug(amc_debug),
        .throttle_pwm_val_out(amc_throttle_val),
        .active_signal(amc_active_signal),
        .complete_signal(amc_complete_signal),
        .z_linear_velocity(z_linear_velocity),
        .imu_good(imu_good),
        .throttle_pwm_val_in(throttle_val),
        .switch_a(switch_a),
        .switch_b(switch_b),
        .start_signal(imu_data_valid),
        .resetn(resetn),
        .us_clk(us_clk)
    );

     throttle_controller TC(
        .throttle_pwm_value_out(tc_throttle_val),
        .complete_signal(throttle_controller_complete),
        .active_signal(throttle_controller_active),
        .throttle_pwm_value_in(amc_throttle_val),
        //.throttle_pwm_value_in(throttle_val),
        .start_signal(amc_complete_signal),
        //.start_signal(imu_data_valid),
        .tc_enable_n(tc_enable_n),
        .switch_a(switch_a),
        .imu_good(imu_good),
        .resetn(resetn),
        .us_clk(us_clk));        
        
    /**
     *   Takes accumulated yaw PWM value to calculate the current desired
     *   yaw rotation. Uses the IMU provided yaw Euler angle to calculate an angle
     *   error from this desired body angle
     *   file - yaw_angle_accumulator.v
     */
    yaw_angle_accumulator  YAAc(
        .yaw_stick_out_of_neutral_window(yaw_stick_out_of_neutral_window),
        .body_yaw_angle_target(yaac_yaw_angle_target),
        .yaw_angle_error_out(yaac_yaw_angle_error),
        .active_signal(yaac_active),
        .complete_signal(yaac_complete),
        .throttle_pwm_value_input(tc_throttle_val),
        .yaw_pwm_value_input(yaw_val),
        .yaw_angle_imu(z_rotation_angle),
        .yaac_enable_n(yaac_enable_n),
        .switch_a(switch_a),
        .debug_out(yaac_debug),
        .imu_good(imu_good),
        .start_signal(throttle_controller_complete),
        .resetn(resetn),
        .us_clk(us_clk)
        );

    /**
     *   Take IMU provided orientation angle and user provided target angle and
     *   subtract them to get the error angle rate to get to target angle
     *   position.
     *   file - angle_controller.v
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
        .throttle_target(tc_throttle_val),
        .yaac_enable_n(yaac_enable_n),
        .yaw_angle_target(yaac_yaw_angle_target),
        .yaw_angle_error_in(yaac_yaw_angle_error),
        .roll_target(roll_val),
        .pitch_target(pitch_val),
        .roll_actual(y_rotation_angle),
        .pitch_actual(x_rotation_angle),
        .switch_a(switch_a),
        .start_signal(yaac_complete),
        .resetn(resetn),
        .us_clk(us_clk));

    /**
     *   Take error rate angles from angle_controller and current rotational
     *   angle rates and feed them into a PID to get corrective control.
     *   file - body_frame_controller.v
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
        .yaw_rotation(z_rotation_rate),
        .yaw_angle_error(yaw_angle_error),
        .roll_angle_error(roll_angle_error),
        .pitch_angle_error(pitch_angle_error),
        .start_signal(ac_valid_strobe),
        .resetn(resetn),
        .us_clk(us_clk));

    /**
     *   Get axis rates and calculate respective motor rates to achieve correct
     *   drone movements.
     *   file - motor_mixer.v
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
     *   Take respective motor rate outputs from motor mixer and convert the
     *   0-250 value to a PWM output to ESCs.
     *   file - pwm_generator.v
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
    
/*
    // 2 word debug instead of the whole bunch    
    uart_top #(.NUM_DEBUG_ELEMENTS(8'd2), .FIXED_INTERVAL(760)) uart
    (
        .resetn(resetn),
        .clk(sys_clk),
        .trigger_start(imu_data_valid),  // Only used if FIXED_INTERVAL is 0
        .sin(sin),
        .rxrdy_n(rxrdy_n),
        .sout(sout),
        .txrdy_n(txrdy_n),

        .debug_1_in_16_bits({8'd0, i2c_driver_debug}),
        .debug_2_in_16_bits({8'd0, i2c_top_debug})

    );
*/        
//*
//   Disabled to use 2 word debug instead        
    uart_top #(.NUM_DEBUG_ELEMENTS(8'd18), .FIXED_INTERVAL(0)) uart
    (
        .resetn(resetn),
        .clk(sys_clk),
        .trigger_start(imu_data_valid),   // Only used if FIXED_INTERVAL is 0
        .sin(sin),
        .rxrdy_n(rxrdy_n),
        .sout(sout),
        .txrdy_n(txrdy_n),

        .debug_1_in_16_bits(x_rotation_angle),
        .debug_2_in_16_bits(y_rotation_angle),
        .debug_3_in_16_bits(z_rotation_angle),
        .debug_4_in_16_bits(x_rotation_rate),
        .debug_5_in_16_bits(y_rotation_rate),
        .debug_6_in_16_bits(z_rotation_rate),
        .debug_7_in_16_bits({8'd0, throttle_val}),
        .debug_8_in_16_bits({8'd0, yaw_val}),
        .debug_9_in_16_bits({8'd0, roll_val}),
        .debug_10_in_16_bits({8'd0, pitch_val}),
        .debug_11_in_16_bits({8'd0, aux1_val}),
        .debug_12_in_16_bits({8'd0, aux2_val}),
        .debug_13_in_16_bits({8'd0, swa_swb_val}),
        .debug_14_in_16_bits({11'd0, switch_b, switch_a}),
        .debug_15_in_16_bits(yaac_yaw_angle_target),
        .debug_16_in_16_bits(VL53L1X_chip_id),
        //.debug_16_in_16_bits({8'd0, VL53L1X_firm_rdy}),
        //.debug_16_in_16_bits({8'd0, VL53L1X_data_rdy}),
        .debug_17_in_16_bits({8'd0, i2c_driver_debug}),
        .debug_18_in_16_bits({8'd0, VL53L1X_firm_rdy}),
        //.debug_18_in_16_bits({8'd0, VL53L1X_data_rdy}),
        //.debug_19_in_16_bits(VL53L1X_range_mm)
        .debug_19_in_16_bits({8'd0, i2c_top_debug})

    );
//*/


    // Synchronously latc reset signal - Make this input not a clock resource
    always@(posedge sys_clk) begin
        resetn <= machxo3_switch_reset_n;
    end
    
    // Synchronously latch IMU values, prevent timing issue with large asynchronous paths
    always@(posedge sys_clk, negedge resetn) begin
        if(~resetn) begin
            x_rotation_angle  <= 'd0;
            y_rotation_angle  <= 'd0;
            z_rotation_angle  <= 'd0;
            x_rotation_rate   <= 'd0;
            y_rotation_rate   <= 'd0;
            z_rotation_rate   <= 'd0;
            x_linear_accel    <= 'd0;
            y_linear_accel    <= 'd0;
            z_linear_accel    <= 'd0;
            gravity_accel_x   <= 'd0;
            gravity_accel_y   <= 'd0;
            gravity_accel_z   <= 'd0;
            quaternion_data_w <= 'd0;
            quaternion_data_x <= 'd0;
            quaternion_data_y <= 'd0;
            quaternion_data_z <= 'd0;
            accel_rate_x      <= 'd0;
            accel_rate_y      <= 'd0;
            accel_rate_z      <= 'd0;
            magneto_rate_x    <= 'd0;
            magneto_rate_y    <= 'd0;
            magneto_rate_z    <= 'd0;
            temperature       <= 'd0;
            calib_status      <= 'd0;
            VL53L1X_chip_id   <= 'd0;
            VL53L1X_range_mm  <= 'd0;  
            VL53L1X_firm_rdy  <= 'd0;
            VL53L1X_data_rdy  <= 'd0;
            imu_data_valid    <= `FALSE;
            imu_good          <= `FALSE;
            i2c_driver_debug  <= 'd0;
            i2c_top_debug     <= 'd0;
        end
        else begin
            x_rotation_angle  <= next_x_rotation_angle;
            y_rotation_angle  <= next_y_rotation_angle;
            z_rotation_angle  <= next_z_rotation_angle;
            x_rotation_rate   <= next_x_rotation_rate;
            y_rotation_rate   <= next_y_rotation_rate;
            z_rotation_rate   <= next_z_rotation_rate;
            x_linear_accel    <= next_x_linear_accel;
            y_linear_accel    <= next_y_linear_accel;
            z_linear_accel    <= next_z_linear_accel;
            gravity_accel_x   <= next_gravity_accel_x;
            gravity_accel_y   <= next_gravity_accel_y;
            gravity_accel_z   <= next_gravity_accel_z;
            quaternion_data_w <= next_quaternion_data_w;
            quaternion_data_x <= next_quaternion_data_x;
            quaternion_data_y <= next_quaternion_data_y;
            quaternion_data_z <= next_quaternion_data_z;
            accel_rate_x      <= next_accel_rate_x;
            accel_rate_y      <= next_accel_rate_y;
            accel_rate_z      <= next_accel_rate_z;
            magneto_rate_x    <= next_magneto_rate_x;
            magneto_rate_y    <= next_magneto_rate_y;
            magneto_rate_z    <= next_magneto_rate_z;   
            temperature       <= next_temperature;
            calib_status      <= next_calib_status;
            VL53L1X_chip_id   <= next_VL53L1X_chip_id;
            VL53L1X_range_mm  <= next_VL53L1X_range_mm;   
            VL53L1X_firm_rdy  <= next_VL53L1X_firm_rdy;
            VL53L1X_data_rdy  <= next_VL53L1X_data_rdy;
            imu_data_valid    <= next_imu_data_valid;
            imu_good          <= next_imu_good;
            i2c_driver_debug  <= next_i2c_driver_debug;
            i2c_top_debug     <= next_i2c_top_debug;
        end
    end
    

    // Enable bits
    assign yaac_enable_n = `LOW_ACTIVE_ENABLE;  // Enable YAAC
    assign tc_enable_n   = `LOW_ACTIVE_ENABLE;  // Enable TC
    //assign tc_enable_n   = `LOW_ACTIVE_DISABLE; // Disable TC
    //assign soft_reset_n  = `LOW_ACTIVE_DISABLE; // Disable this reset for now, connect if soft reset is needed and remove this line
        
    
    assign resetn_lidar = 1;
    /**
     * The section below is for use with Debug LEDs
     */

    // Update on board LEDs, all inputs are active low
    always @(posedge sys_clk, negedge resetn) begin
        if (!resetn) begin
            //led_data_out <= 8'hAA;
            led_data_out <= 8'hFF;
        end
        else begin
            led_data_out <= ~(i2c_driver_debug<<1); // Shifted one bit left because D2 is burned out on my board
        end
    end
endmodule

