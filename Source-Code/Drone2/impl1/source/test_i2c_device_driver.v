`timescale 1ns / 1ns
`default_nettype none
`include "common_defines.v"

/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

module i2c_device_driver_tb();
    reg purn;
    wire done;
    reg go;
    reg read_write_in = 1;
    reg [3:0] i2c_count;
    reg i2c_ack;
    reg [7:0]sda_byte = 0;
    reg next_mod_active_cmd;
    
    reg prev_scl  = 0;
    reg prev_sda  = 0;
    reg i2c_start = 0;
    reg i2c_stop  = 0;
    reg prev_i2c_start = 0;
    reg prev_i2c_stop  = 0;
    
    
    wire scl_1;
    wire sda_1;
    wire scl_2;
    wire sda_2;
    reg  resetn;
    wire [7:0]led_data_out;
    wire sys_clk;
    reg  next_mod_active;
    wire resetn_imu;
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
    wire [15:0]vl53l1x_chip_id; 

    integer i = 0;
    integer j = 0;


    GSR GSR_INST (.GSR (resetn));
    PUR PUR_INST (.PUR (purn));

    defparam OSCH_inst.NOM_FREQ = "38.00";
    OSCH OSCH_inst (.STDBY(1'b0),
                    .OSC(sys_clk),
                    .SEDSTDBY());


    i2c_device_driver #(2) DUT(
        .scl_1(scl_1),
        .sda_1(sda_1),
        .scl_2(scl_2),
        .sda_2(sda_2),
        .resetn(resetn),
        .led_data_out(led_data_out),
        .sys_clk(sys_clk),
        .next_mod_active(next_mod_active),
        .resetn_imu(resetn_imu),
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
        .vl53l1x_chip_id(vl53l1x_chip_id)
        ); /* synthesis syn_hier=hard */;
        
        
    //always@(posedge sys_clk, negedge resetn) begin
    always@(posedge sys_clk, negedge resetn, negedge resetn_imu) begin
        if(~resetn || ~resetn_imu || (DUT.i2c_state <= 4)) begin
            prev_scl  <= 0;
            prev_sda  <= 0;
            i2c_start <= 0;
            i2c_stop  <= 0;
        end
        else begin
            if((~i2c_ack)&& (scl_1 == 1'b1) && ((prev_sda == 1'b1) && (sda_1 == 1'b0)) ) begin
                i2c_start <= 1;
                i2c_stop  <= 0;
            end
            else if ((~i2c_ack) && i2c_start && (scl_1 == 1'b1) && ((prev_sda == 1'b0) && (sda_1 == 1'b1)) ) begin
                i2c_start <= 0;
                i2c_stop  <= 1;
            end
            prev_scl       <= scl_1;
            prev_sda       <= sda_1;
            prev_i2c_start <= i2c_start;
            prev_i2c_stop  <= i2c_stop;
        end
    end

    always@(posedge i2c_start)
        $display("%t: I2C start asserted", $time);
        
    always@(posedge i2c_stop)
        $display("%t: I2C stop asserted", $time);
        
    always@(i2c_count)
        $display("%t: I2C i2c_count=%d", $time, i2c_count);

    always@(i2c_ack)
        $display("%t: I2C i2c_ack=%b", $time, i2c_ack);

// Generate a slave ACK every 9 i2c SCL posedges, regardless of what data is on the bus
    always@(posedge scl_1, posedge i2c_stop, posedge i2c_start, negedge resetn) begin
        if((~resetn) || (i2c_stop && (prev_i2c_stop != i2c_stop))|| (i2c_start&& (prev_i2c_start != i2c_start))) begin
            i2c_count <= 1'b0;
            i2c_ack   <= 1'b0;
            sda_byte  <= 8'b0;
        end
        else begin
            if(i2c_count >= 4'd8) begin
                i2c_ack   <= 1'b1;
                #(2_500*10); // hold low for one I2C clock tick, 2.5 us
                i2c_ack   <= 1'b0;
                i2c_count <= 1'b0;
                sda_byte  <= 8'b0;
            end
            else begin
                i2c_count <= i2c_count + 1'b1;
                i2c_ack   <= 1'b0;
                sda_byte  <= {sda_byte[6:0], sda_1};
            end
        end
    end


    // Respond to active cmd on posedge of next_mod_active_cmd
    // next_mode ative is loq for #1000, (make sure previous module plauses until this goes high)
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

    // I2C pullups
    assign ( pull1, strong0 ) scl_1 = 1'b1;
    assign ( pull1, strong0 ) sda_1 = i2c_ack ? 1'b0: 1'b1;
    
    // Test cases
    initial begin
        resetn = 1;
        #10 resetn = 0;
        #10 resetn = 1;
        read_write_in = 0;
        //for(j = 0; j < 2; j = j + 1) begin
        //    for(i = 0; i < 10; i = i + 1) begin
        //        $display("efb_registers %1d EFB#%1d = %h", i[4:0], (j[4:0]+1), bno055.i2c.efb_registers[i][j]);
        //    end
        //end
        #500_000_000;
        $stop;
        end
endmodule

