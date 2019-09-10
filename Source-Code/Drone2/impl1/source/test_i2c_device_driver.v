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
    reg [9:0] sys_clk_cnt;
    reg i2c_ack;
    reg [6:0]slave_addr = 0;
    reg rw_bit          = 0;
    reg [7:0]reg_addr   = 0;
    reg [7:0]reg_value  = 0;
    reg next_mod_active_cmd;
    
    reg prev_scl  = 0;
    reg prev_sda  = 0;
    reg i2c_start = 0;
    reg i2c_stop  = 0;
    
    reg [5:0]i2c_state = 0;
    reg prev_i2c_start = 0;
    reg prev_i2c_stop  = 0;
    
    
    wire scl_1;
    wire sda_1;
    wire scl_2;
    wire sda_2;
    reg  resetn;
    wire [7:0]led_data_out;
    wire sys_clk;
    reg  i2c_clk;
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
    
    //Common states
    localparam I2C_INIT         = 6'd0;      
    localparam I2C_STA       = 6'd1; 
    localparam WR_SLV_ADDR_B6   = 6'd2; 
    localparam WR_SLV_ADDR_B5   = 6'd3; 
    localparam WR_SLV_ADDR_B4   = 6'd4; 
    localparam WR_SLV_ADDR_B3   = 6'd5; 
    localparam WR_SLV_ADDR_B2   = 6'd6; 
    localparam WR_SLV_ADDR_B1   = 6'd7; 
    localparam WR_SLV_ADDR_B0   = 6'd8; 
    localparam WR_RW_BIT        = 6'd9; 
    localparam WR_SLV_ACK1      = 6'd10; 
    localparam WR_REG_ADDR_B7   = 6'd11; 
    localparam WR_REG_ADDR_B6   = 6'd12; 
    localparam WR_REG_ADDR_B5   = 6'd13; 
    localparam WR_REG_ADDR_B4   = 6'd14; 
    localparam WR_REG_ADDR_B3   = 6'd15; 
    localparam WR_REG_ADDR_B2   = 6'd16; 
    localparam WR_REG_ADDR_B1   = 6'd17; 
    localparam WR_REG_ADDR_B0   = 6'd18; 
    localparam WR_SLV_ACK2      = 6'd19; 
    
    //Only for write
    localparam WR_REG_VAL_B7    = 6'd20; 
    localparam WR_REG_VAL_B6    = 6'd21; 
    localparam WR_REG_VAL_B5    = 6'd22; 
    localparam WR_REG_VAL_B4    = 6'd23; 
    localparam WR_REG_VAL_B3    = 6'd24; 
    localparam WR_REG_VAL_B2    = 6'd25; 
    localparam WR_REG_VAL_B1    = 6'd26; 
    localparam WR_REG_VAL_B0    = 6'd27; 
    localparam WR_SLV_ACK3      = 6'd28; 
    
    //Only for read
    localparam RD_SLV_ADDR_B6   = 6'd29; 
    localparam RD_SLV_ADDR_B5   = 6'd30; 
    localparam RD_SLV_ADDR_B4   = 6'd31; 
    localparam RD_SLV_ADDR_B3   = 6'd32; 
    localparam RD_SLV_ADDR_B2   = 6'd33; 
    localparam RD_SLV_ADDR_B1   = 6'd34; 
    localparam RD_SLV_ADDR_B0   = 6'd35; 
    localparam RD_RW_BIT        = 6'd36; 
    localparam RD_SLV_ACK1      = 6'd37; 
    localparam RD_REG_VAL_B7    = 6'd38; 
    localparam RD_REG_VAL_B6    = 6'd39; 
    localparam RD_REG_VAL_B5    = 6'd40; 
    localparam RD_REG_VAL_B4    = 6'd41; 
    localparam RD_REG_VAL_B3    = 6'd42; 
    localparam RD_REG_VAL_B2    = 6'd43; 
    localparam RD_REG_VAL_B1    = 6'd44; 
    localparam RD_REG_VAL_B0    = 6'd45; 
    localparam RD_MAST_ACK_NACK = 6'd46;
    localparam RD_STO           = 6'd47;


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
        
        
     
    // Generate a 50kHz I2C clock for simulation (Even thugh 400kHz is configured?)   
    always@(negedge sys_clk, negedge resetn) begin
        if(~resetn ) begin
            i2c_clk     <= 1'b0;
            sys_clk_cnt <= 10'b0;
        end
        else begin
            if (sys_clk_cnt > 10'd760)
                sys_clk_cnt <= 10'b0;
            else
                sys_clk_cnt <= (sys_clk_cnt + 10'd1);
            if(sys_clk_cnt>10'd380)
                i2c_clk <= 1'b1;
            else
                i2c_clk <= 1'b0;
        end
    end

    
    
    always@(posedge i2c_clk, negedge resetn) begin
        if(~resetn) begin
            i2c_state  <= I2C_INIT;
            i2c_start  <= 1'b0;
            i2c_stop   <= 1'b0;
            rw_bit     <= 1'b0;
            slave_addr <= 7'd0;
            reg_addr   <= 8'd0;
            reg_value  <= 8'd0;
            i2c_ack    <= 1'b0;
        end
        else begin
            i2c_ack  <= 1'b0;
            prev_sda <= sda_1;
            case(i2c_state)
                I2C_INIT         : begin
                    i2c_state  <= I2C_STA;
                    i2c_start  <= 1'b0;
                    i2c_stop   <= 1'b0;
                    rw_bit     <= 1'b0;
                    slave_addr <= 7'd0;
                    reg_addr   <= 8'd0;
                    reg_value  <= 8'd0;
                    i2c_ack    <= 1'b0;
                end
                I2C_STA          : begin
                    if((scl_1 == 1'b1) && ((prev_sda == 1'b1) && (sda_1 == 1'b0))) begin
                        i2c_state  <= WR_SLV_ADDR_B6;
                        i2c_start  <= 1'b1;
                        i2c_stop   <= 1'b0;
                        rw_bit     <= 1'b0;
                        slave_addr <= 7'd0;
                        reg_addr   <= 8'd0;
                        reg_value  <= 8'd0;
                        i2c_ack    <= 1'b0;
                    end
                end
                WR_SLV_ADDR_B6   : begin
                    i2c_state     <= WR_SLV_ADDR_B5;
                    slave_addr[6] <= sda_1;
                end
                WR_SLV_ADDR_B5   : begin
                    i2c_state     <= WR_SLV_ADDR_B4;
                    slave_addr[5] <= sda_1;
                end
                WR_SLV_ADDR_B4   : begin
                    i2c_state     <= WR_SLV_ADDR_B3;
                    slave_addr[4] <= sda_1;
                end
                WR_SLV_ADDR_B3   : begin
                    i2c_state     <= WR_SLV_ADDR_B2;
                    slave_addr[3] <= sda_1;
                end
                WR_SLV_ADDR_B2   : begin
                    i2c_state     <= WR_SLV_ADDR_B1;
                    slave_addr[2] <= sda_1;
                end
                WR_SLV_ADDR_B1   : begin
                    i2c_state     <= WR_SLV_ADDR_B0;
                    slave_addr[1] <= sda_1;
                end
                WR_SLV_ADDR_B0   : begin
                    i2c_state     <= WR_RW_BIT;
                    $display("%t: Access slave address = %h", $time, {slave_addr[6:1], sda_1});
                    slave_addr[0] <= sda_1;
                end
                WR_RW_BIT        : begin
                    i2c_state   <= WR_SLV_ACK1;
                    rw_bit      <= sda_1;
                    if(sda_1 == 1'b0) begin
                        $display("%t: I2C access is a write", $time);
                        i2c_ack <= 1'b1;
                    end
                    else begin
                        $display("%t: I2C access is a read", $time);
                        $display("%t: ERROR, must start with write to write slave address to access", $time);
                        $stop;
                    end
                end
                WR_SLV_ACK1      : begin
                    i2c_ack       <= 1'b0;
                    if(sda_1 == 1'b0)
                        i2c_state <= WR_REG_ADDR_B7;
                    else begin
                        $display("%t: ERROR, slave didn't ack the slave address", $time);
                        $stop;
                    end
                end
                WR_REG_ADDR_B7   : begin
                    i2c_state   <= WR_REG_ADDR_B6;
                    reg_addr[7] <= sda_1;
                end
                WR_REG_ADDR_B6   : begin
                    i2c_state   <= WR_REG_ADDR_B5;
                    reg_addr[6] <= sda_1;
                end
                WR_REG_ADDR_B5   : begin
                    i2c_state   <= WR_REG_ADDR_B4;
                    reg_addr[5] <= sda_1;
                end
                WR_REG_ADDR_B4   : begin
                    i2c_state   <= WR_REG_ADDR_B3;
                    reg_addr[4] <= sda_1;
                end
                WR_REG_ADDR_B3   : begin
                    i2c_state   <= WR_REG_ADDR_B2;
                    reg_addr[3] <= sda_1;
                end
                WR_REG_ADDR_B2   : begin
                    i2c_state   <= WR_REG_ADDR_B1;
                    reg_addr[2] <= sda_1;
                end
                WR_REG_ADDR_B1   : begin
                    i2c_state   <= WR_REG_ADDR_B0;
                    reg_addr[1] <= sda_1;
                end
                WR_REG_ADDR_B0   : begin
                    i2c_state   <= WR_SLV_ACK2;
                    reg_addr[0] <= sda_1;
                    i2c_ack     <= 1'b1;
                    $display("%t: Register address = %h", $time, {reg_addr[7:1], sda_1});
                end
                WR_SLV_ACK2      : begin
                    i2c_ack       <= 1'b0;
                    if(sda_1 == 1'b0)
                        i2c_state <= WR_REG_VAL_B7;
                    else begin
                        $display("%t: ERROR, slave didn't ack the register address", $time);
                        $stop;
                    end
                end
                WR_REG_VAL_B7    : begin
                    if((scl_1 == 1'b1) && ((prev_sda == 1'b1) && (sda_1 == 1'b0))) begin // Restart - begin read cycle
                        $display("%t: Repeated start, begin I2C read", $time);
                        i2c_state    <= WR_REG_VAL_B6;
                    end
                    else if ((scl_1 == 1'b1) && ((prev_sda == 1'b0) && (sda_1 == 1'b1)) ) begin // Stop - no more to write
                        $display("%t: Stop Asserted! Terminate I2C write", $time);
                        i2c_state    <= I2C_STA;
                    end
                    else begin //Continue write
                        i2c_state    <= WR_REG_VAL_B6;
                        reg_value[7] <= sda_1;
                    end
                end
                WR_REG_VAL_B6    : begin
                    i2c_state    <= WR_REG_VAL_B5;
                    reg_value[6] <= sda_1;
                end
                WR_REG_VAL_B5    : begin
                    i2c_state    <= WR_REG_VAL_B4;
                    reg_value[5] <= sda_1;
                end
                WR_REG_VAL_B4    : begin
                    i2c_state    <= WR_REG_VAL_B3;
                    reg_value[4] <= sda_1;
                end
                WR_REG_VAL_B3    : begin
                    i2c_state    <= WR_REG_VAL_B2;
                    reg_value[3] <= sda_1;
                end
                WR_REG_VAL_B2    : begin
                    i2c_state    <= WR_REG_VAL_B1;
                    reg_value[2] <= sda_1;
                end
                WR_REG_VAL_B1    : begin
                    i2c_state    <= WR_REG_VAL_B0;
                    reg_value[1] <= sda_1;
                end
                WR_REG_VAL_B0    : begin
                    i2c_state    <= WR_REG_VAL_B6;
                    reg_value[0] <= sda_1;
                    i2c_ack      <= 1'b1;
                    $display("%t: Write register value = %h", $time, {reg_value[7:1], sda_1});
                end
                WR_SLV_ACK3      : begin
                    i2c_ack       <= 1'b0;
                    if(sda_1 == 1'b0)
                        i2c_state <= WR_REG_VAL_B7;
                    else begin
                        $display("%t: ERROR, slave didn't ack the register value", $time);
                        $stop;
                    end
                end
                RD_SLV_ADDR_B6   : begin
                    i2c_state     <= RD_SLV_ADDR_B5;
                    slave_addr[6] <= sda_1;
                end
                RD_SLV_ADDR_B5   : begin
                    i2c_state     <= RD_SLV_ADDR_B4;
                    slave_addr[5] <= sda_1;
                end
                RD_SLV_ADDR_B4   : begin
                    i2c_state     <= RD_SLV_ADDR_B3;
                    slave_addr[4] <= sda_1;
                end
                RD_SLV_ADDR_B3   : begin
                    i2c_state     <= RD_SLV_ADDR_B2;
                    slave_addr[3] <= sda_1;
                end
                RD_SLV_ADDR_B2   : begin
                    i2c_state     <= RD_SLV_ADDR_B1;
                    slave_addr[2] <= sda_1;
                end
                RD_SLV_ADDR_B1   : begin
                    i2c_state     <= RD_SLV_ADDR_B0;
                    slave_addr[1] <= sda_1;
                end
                RD_SLV_ADDR_B0   : begin
                    i2c_state     <= RD_RW_BIT;
                    slave_addr[0] <= sda_1;
                    $display("%t: Read from slave address = %h", $time, {slave_addr[6:1], sda_1});
                end
                RD_RW_BIT        : begin
                    rw_bit      <= sda_1;
                    if(sda_1 == 1'b1) begin
                        $display("%t: I2C access is a read", $time);
                        i2c_ack <= 1'b1;
                    end
                    else begin
                        $display("%t: I2C access is a write", $time);
                        $display("%t: ERROR, must start read cycle with rw bit set", $time);
                        $stop;
                    end
                end
                RD_SLV_ACK1      : begin
                    i2c_ack       <= 1'b0;
                    if(sda_1 == 1'b0)
                        i2c_state <= RD_REG_VAL_B7;
                    else begin
                        $display("%t: ERROR, slave didn't ack the slave address", $time);
                        $stop;
                    end
                end
                RD_REG_VAL_B7    : begin
                    if((scl_1 == 1'b1) && ((prev_sda == 1'b1) && (sda_1 == 1'b0))) begin // Restart - error
                        $display("%t: ERROR: Repeated start during read", $time);
                        $stop;
                    end
                    else if ((scl_1 == 1'b1) && ((prev_sda == 1'b0) && (sda_1 == 1'b1)) ) begin // Stop - no more to read
                        $display("%t: ERROR: Stop asserted during read with out NACK from master", $time);
                        $stop;
                    end
                    else begin //Continue read
                        i2c_state    <= RD_REG_VAL_B6;
                        reg_value[7] <= sda_1;
                    end
                end
                RD_REG_VAL_B6    : begin
                    i2c_state    <= RD_REG_VAL_B5;
                    reg_value[6] <= sda_1;
                end
                RD_REG_VAL_B5    : begin
                    i2c_state    <= RD_REG_VAL_B4;
                    reg_value[5] <= sda_1;
                end
                RD_REG_VAL_B4    : begin
                    i2c_state    <= RD_REG_VAL_B3;
                    reg_value[4] <= sda_1;
                end
                RD_REG_VAL_B3    : begin
                    i2c_state    <= RD_REG_VAL_B2;
                    reg_value[3] <= sda_1;
                end
                RD_REG_VAL_B2    : begin
                    i2c_state    <= RD_REG_VAL_B1;
                    reg_value[2] <= sda_1;
                end
                RD_REG_VAL_B1    : begin
                    i2c_state    <= RD_REG_VAL_B0;
                    reg_value[1] <= sda_1;
                end
                RD_REG_VAL_B0    : begin
                    i2c_state    <= RD_MAST_ACK_NACK;
                    reg_value[0] <= sda_1;
                    i2c_ack      <= 1'b0;
                    $display("%t: Read register value = %h", $time, {reg_value[6:1], sda_1});
                end
                RD_MAST_ACK_NACK : begin
                    i2c_ack       <= 1'b0;
                    if(sda_1 == 1'b0)
                        i2c_state <= RD_REG_VAL_B7;
                    else begin
                        i2c_state <= RD_STO;
                    end
                end
                RD_STO           : begin
                    if((scl_1 == 1'b1) && ((prev_sda == 1'b1) && (sda_1 == 1'b0))) begin // Restart - error
                        $display("%t: ERROR: Repeated start during read", $time);
                        $stop;
                    end
                    else if ((scl_1 == 1'b1) && ((prev_sda == 1'b0) && (sda_1 == 1'b1)) ) begin // Stop - no more to read
                        $display("%t: Stop asserted, terminating I2C read cycle", $time);
                        i2c_state <= I2C_STA;
                    end
                    else begin
                        $display("%t: ERROR: Master didn't STO following NACK from master", $time);
                        $stop;
                    end
                end
                default : begin
                    i2c_state  <= I2C_INIT;
                    i2c_start  <= 1'b0;
                    i2c_stop   <= 1'b0;
                    rw_bit     <= 1'b0;
                    slave_addr <= 7'd0;
                    reg_addr   <= 8'd0;
                    reg_value  <= 8'd0;
                end
            endcase
        end
    end
/*    
    always@(posedge i2c_clk, negedge resetn, negedge resetn_imu) begin
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
                @(negedge scl_1);
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
*/

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

