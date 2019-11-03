`timescale 1ns / 1ns
`default_nettype none
`include "common_defines.v"

/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */
 
//`define DETECT_INVALID_TRANSITIONS

module i2c_device_driver_tb();
    reg purn;
    wire done;
    reg go;
    reg read_write_in = 1;
    reg next_mod_active_cmd;
    
    
    parameter SLAVE_ADDR_VL53l1X = 7'h29;
    parameter SLAVE_ADDR_BNO055  = 7'h28;

    
    wire scl_1;
    wire sda_1;
    wire scl_2;
    wire sda_2;
    reg  resetn;
    wire [7:0]led_data_out;
    wire [7:0]i2c_top_debug;
    wire sys_clk;
    reg  next_mod_active;
    wire resetn_imu;
    wire resetn_lidar;
    wire imu_good;
    wire valid_strobe;
    wire [15:0]accel_rate_x;
    wire [15:0]accel_rate_y;
    wire [15:0]accel_rate_z;
    wire [15:0]magneto_rate_x;
    wire [15:0]magneto_rate_y;
    wire [15:0]magneto_rate_z;
    wire [15:0]gyro_rate_x;
    wire [15:0]gyro_rate_y;
    wire [15:0]gyro_rate_z;
    wire [15:0]euler_angle_x;
    wire [15:0]euler_angle_y;
    wire [15:0]euler_angle_z;
    wire [15:0]quaternion_data_w;
    wire [15:0]quaternion_data_x;
    wire [15:0]quaternion_data_y;
    wire [15:0]quaternion_data_z;
    wire [15:0]linear_accel_x;
    wire [15:0]linear_accel_y;
    wire [15:0]linear_accel_z;
    wire [15:0]gravity_accel_x;
    wire [15:0]gravity_accel_y;
    wire [15:0]gravity_accel_z;
    wire [7:0]temperature;
    wire [7:0]calib_status;
    wire [15:0]VL53L1X_chip_id; 
    wire [15:0]VL53L1X_range_mm;
    wire [7:0]VL53L1X_firm_rdy;
    wire [7:0]VL53L1X_data_rdy;

    integer i = 0;
    integer j = 0;
    
    GSR GSR_INST (.GSR (resetn));
    PUR PUR_INST (.PUR (purn));
    defparam OSCH_inst.NOM_FREQ = "38.00";
    OSCH OSCH_inst (.STDBY(1'b0),
                    .OSC(sys_clk),
                    .SEDSTDBY());


    i2c_device_driver #(10) DUT(
        .scl_1(scl_1),
        .sda_1(sda_1),
        .scl_2(scl_2),
        .sda_2(sda_2),
        .resetn(resetn),
        .led_data_out(led_data_out),
        .i2c_top_debug(i2c_top_debug),
        .sys_clk(sys_clk),
        .next_mod_active(next_mod_active),
        .resetn_imu(resetn_imu),
        .resetn_lidar(resetn_lidar),
        .imu_good(imu_good),
        .valid_strobe(valid_strobe),
        .accel_rate_x(accel_rate_x),
        .accel_rate_y(accel_rate_y),
        .accel_rate_z(accel_rate_z),
        .magneto_rate_x(magneto_rate_x),
        .magneto_rate_y(magneto_rate_y),
        .magneto_rate_z(magneto_rate_z),
        .gyro_rate_x(gyro_rate_x),
        .gyro_rate_y(gyro_rate_y),
        .gyro_rate_z(gyro_rate_z),
        .euler_angle_x(euler_angle_x),
        .euler_angle_y(euler_angle_y),
        .euler_angle_z(euler_angle_z),
        .quaternion_data_w(quaternion_data_w),
        .quaternion_data_x(quaternion_data_x),
        .quaternion_data_y(quaternion_data_y),
        .quaternion_data_z(quaternion_data_z),
        .linear_accel_x(linear_accel_x),
        .linear_accel_y(linear_accel_y),
        .linear_accel_z(linear_accel_z),
        .gravity_accel_x(gravity_accel_x),
        .gravity_accel_y(gravity_accel_y),
        .gravity_accel_z(gravity_accel_z),
        .temperature(temperature),
        .calib_status(calib_status),
        .VL53L1X_chip_id(VL53L1X_chip_id),
        .VL53L1X_range_mm(VL53L1X_range_mm),
        .VL53L1X_firm_rdy(VL53L1X_firm_rdy),
        .VL53L1X_data_rdy(VL53L1X_data_rdy)
        );
        
        
        
    always@(VL53L1X_chip_id)
        $display("%t: Chip ID is %h", $time, VL53L1X_chip_id);

	// Connect i2c slaves
	//i2c_slave_model_2B_reg #(SLAVE_ADDR_VL53l1X) VL53l1X (
	i2c_slave_model #(SLAVE_ADDR_VL53l1X) VL53l1X (
        .resetn(resetn),
		.scl(scl_1),
		.sda(sda_1)
	);

	i2c_slave_model #(SLAVE_ADDR_BNO055) BNO055 (
        .resetn(resetn),
		.scl(scl_1),
		.sda(sda_1)
	);

	pullup p1(scl_1); // pull-up scl line
	pullup p2(sda_1); // pull-up sda line
 
   
    
    // Respond to active cmd on posedge of next_mod_active_cmd
    // next_mod_active is low for #1000, (make sure previous module pauses until this goes high)
    //           high for #100,  (Stay high for a while)
    //           then low again
    always@(posedge next_mod_active_cmd, negedge resetn) begin
        if (~resetn)
            next_mod_active <= `LOW;
        else begin
            next_mod_active <= `LOW;
            #1000;
            next_mod_active <= `HIGH;
            #100;
            next_mod_active <= `LOW;
            #1;
        end
    end

    // trigger next_mod_active_cmd for only one clk cycle - posedge detection of valid_strobe
    always@(posedge sys_clk, negedge resetn) begin
        if (~resetn)
            next_mod_active_cmd <= `LOW;
        else if(valid_strobe && (~next_mod_active_cmd))
            next_mod_active_cmd <= `HIGH;
        else if(valid_strobe && (next_mod_active_cmd))
            next_mod_active_cmd <= `LOW;
        else
            next_mod_active_cmd <= `LOW;


    end
 
    // Test cases
    initial begin
        resetn = 1;
        #10 resetn = 0;
        #10 resetn = 1;
        read_write_in = 0;
        //for(j = 0; j < 2; j = j + 1) begin
        //    for(i = 0; i < 10; i = i + 1) begin
        //        $display("%t: efb_registers %1d EFB#%1d = %h", $time, i[4:0], (j[4:0]+1), bno055.i2c.efb_registers[i][j]);
        //    end
        //end
        #500_000_000;
        $stop;
        end
endmodule

