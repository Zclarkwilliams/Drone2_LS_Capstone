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
    reg [3:0] i2c_count;
    reg [9:0] sys_clk_cnt;
    reg [7:0] i2c_clk_8x_cnt;
    reg [7:0] i2c_clk_8x_async_cnt;
    reg cmd_i2c_ack;
    reg [6:0]slave_addr = 0;
    reg rw_bit          = 0;
    reg [7:0]reg_addr   = 0;
    reg [7:0]reg_value  = 0;
    reg next_mod_active_cmd;
    
    reg [6:0]sda_buff       = 7'd0;
    reg [2:0]sda_buff_cnt   = 3'd0;
    reg scl_buff_fast       = 1'b0; // Buffer scl value for one clock cycle at sys_clk rate
    reg [1:0]scl_buff_slow  = 1'b0; // Buffer scl value for one clock cycle at i2c_clk_8x_async rate

    reg rd_i2c_start   = 0;
    reg rd_i2c_stop    = 0;
    reg rd_i2c_ack     = 0;
    reg rd_i2c_nack    = 0;
    reg rd_i2c_bit     = 0;
    reg rd_i2c_bit_err = 0;
    
    reg [1:0]read_i2c_state = 0;
    reg [5:0]i2c_tbstate    = 0;
    
    
    wire scl_1;
    wire sda_1;
    wire scl_2;
    wire sda_2;
    reg  resetn;
    wire [7:0]led_data_out;
    wire sys_clk;
    reg  i2c_clk;
    reg  i2c_clk_8x;
    reg  i2c_clk_8x_async;
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
    
    // States for I2C single bit read logic
    localparam I2C_BIT_INIT     = 2'd0;
    localparam I2C_BIT_WAIT     = 2'd1;
    localparam I2C_BIT_READ1    = 2'd2;
    localparam I2C_BIT_READn    = 2'd3;
    
    
    // States for I2C bit logic
    // Common states
    localparam I2C_INIT         = 6'd0;      
    localparam I2C_STA          = 6'd1; 
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
    
    // Only for write
    localparam WR_REG_VAL_B7    = 6'd20; 
    localparam WR_REG_VAL_B6    = 6'd21; 
    localparam WR_REG_VAL_B5    = 6'd22; 
    localparam WR_REG_VAL_B4    = 6'd23; 
    localparam WR_REG_VAL_B3    = 6'd24; 
    localparam WR_REG_VAL_B2    = 6'd25; 
    localparam WR_REG_VAL_B1    = 6'd26; 
    localparam WR_REG_VAL_B0    = 6'd27; 
    localparam WR_SLV_ACK3      = 6'd28; 
    
    // Only for read
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
 
    
    // Generate a clock at 8x the I2C clock rate, asynchronous (NOT synced to posedge of SCL)   
    
    
    // This block is pending deletion as of Sept-26-2019 - Was previously used in scl_buff_slow capture block //
    
    always@(posedge sys_clk, negedge resetn) begin
        if(~resetn ) begin
            i2c_clk_8x_async             <= 1'b1;
            i2c_clk_8x_async_cnt         <= 10'd0;
        end
        else begin
            if (i2c_clk_8x_async_cnt > 10'd48)
                i2c_clk_8x_async_cnt <= 10'b0;
            else
                i2c_clk_8x_async_cnt <= (i2c_clk_8x_cnt + 10'd1);
            if(i2c_clk_8x_async_cnt > 10'd24)
                i2c_clk_8x_async     <= 1'b0;
            else
                i2c_clk_8x_async     <= 1'b1;
        end
    end

    // Latch previous SCL value at sys_clk rate, done at negedge since i2c_clk_8x generator uses this value at posedge

    always@(negedge sys_clk, negedge resetn) begin
        if(~resetn ) begin
            scl_buff_fast <= 0;
        end
        else begin
            scl_buff_fast <= scl_1;
        end
    end 


    // Generate a clock at 8x the I2C clock rate, synced to posedge of SCL   
    always@(posedge sys_clk, negedge resetn) begin
        if(~resetn ) begin
            i2c_clk_8x             <= 1'b1;
            i2c_clk_8x_cnt         <= 10'd0;
        end
        else begin
            if ((scl_buff_fast == 1'b0) && (scl_1 == 1'b1)) begin //positive clock edge
                    i2c_clk_8x     <= 1'b1;
                    i2c_clk_8x_cnt <= 10'b0;
            end
            else begin
                if (i2c_clk_8x_cnt > 10'd48)
                    i2c_clk_8x_cnt <= 10'b0;
                else
                    i2c_clk_8x_cnt <= (i2c_clk_8x_cnt + 10'd1);
                if(i2c_clk_8x_cnt > 10'd24)
                    i2c_clk_8x     <= 1'b0;
                else
                    i2c_clk_8x     <= 1'b1;
            end
        end
    end
    
    // Latch previous SCL value at i2c_clk_8x rate
    always@(negedge i2c_clk_8x, negedge resetn) begin
        if(~resetn ) begin
            scl_buff_slow[0] <= 0;
            scl_buff_slow[1] <= 0;
        end
        else begin
            scl_buff_slow[0] <= scl_1;
            scl_buff_slow[1] <= scl_buff_slow[0];
        end
    end 


    
    // I2C bit detector logic - Next states - At negedge since FSM that uses it is at posedge
    always@(negedge i2c_clk_8x, negedge resetn, negedge resetn_imu) begin
        if(~resetn)
            read_i2c_state <= I2C_BIT_INIT;
        else begin
            case(read_i2c_state)
                I2C_BIT_INIT : begin
                    read_i2c_state <= I2C_BIT_READ1;
                end
                I2C_BIT_WAIT : begin
                    if ((scl_buff_slow[0] == 1'b0) && (scl_1 == 1'b1)) // Positive edge of scl
                        read_i2c_state <= I2C_BIT_READ1;
                end
                I2C_BIT_READ1 : begin
                    read_i2c_state <= I2C_BIT_READn;
                end
                I2C_BIT_READn : begin
                    if((scl_buff_slow[0] == 1'b1) && (scl_1 == 1'b0)) // Negative edge of scl
                        read_i2c_state <= I2C_BIT_WAIT;
                end
            endcase
        end
    end
    
    // I2C bit detector logic    
    always@(posedge i2c_clk_8x, negedge resetn, negedge resetn_imu) begin
        if(~resetn || ~resetn_imu) begin
            sda_buff       <= 7'd0;
            rd_i2c_start   <= 1'b0;
            rd_i2c_stop    <= 1'b0;
            rd_i2c_ack     <= 1'b0;
            rd_i2c_nack    <= 1'b0;
            rd_i2c_bit     <= 1'b0;
            rd_i2c_bit_err <= 1'b0;
            sda_buff_cnt   <= 3'd0;
        end
        else begin
            case(read_i2c_state)
                I2C_BIT_INIT : begin
                    sda_buff       <= 7'd0;
                    rd_i2c_start   <= 1'b0;
                    rd_i2c_stop    <= 1'b0;
                    rd_i2c_ack     <= 1'b0;
                    rd_i2c_nack    <= 1'b0;
                    rd_i2c_bit     <= 1'b0;
                    rd_i2c_bit_err <= 1'b0;
                    sda_buff_cnt   <= 3'd0;
                end
                I2C_BIT_WAIT : begin
                    sda_buff       <= 8'd0;
                    sda_buff_cnt   <= 3'd0;
                end
                I2C_BIT_READ1 : begin
                    sda_buff_cnt   <= 3'd1;
                    sda_buff       <= {sda_buff[5:0], sda_1};
                    //rd_i2c_start   <= 1'b0;
                    //rd_i2c_stop    <= 1'b0;
                    //rd_i2c_ack     <= 1'b0;
                    //rd_i2c_nack    <= 1'b0;
                    //rd_i2c_bit     <= 1'b0;
                    //rd_i2c_bit_err <= 1'b0;
                end
                I2C_BIT_READn : begin
                    sda_buff_cnt   <= sda_buff_cnt + 3'd1;
                    sda_buff       <= {sda_buff[5:0], sda_1};
                    if( (sda_buff_cnt >= 7) || ((scl_buff_slow[0] == 1'b1)&&(scl_1 == 1'b0)) ) begin //Negative edge of scl
                        rd_i2c_start   <= 1'b0;
                        rd_i2c_stop    <= 1'b0;
                        rd_i2c_ack     <= 1'b0;
                        rd_i2c_nack    <= 1'b0;
                        rd_i2c_bit     <= 1'b0;
                        rd_i2c_bit_err <= 1'b0;
                        case({sda_buff, sda_1})
                            7'b1111110,
                            7'b1111100,
                            7'b1111000,
                            7'b1110000,
                            7'b1100000,
                            7'b1000000: begin // Start detected
                                rd_i2c_start <= 1'b1;
                                rd_i2c_bit   <= 1'b0;
                            end
                            7'b0000001,
                            7'b0000011,
                            7'b0000111,
                            7'b0001111,
                            7'b0011111,
                            7'b0111111: begin // Stop detected
                                rd_i2c_stop  <= 1'b1;
                                rd_i2c_bit   <= 1'b0;
                            end
                            7'b0000000: begin // ACK detected or a logic 0
                                rd_i2c_ack     <= 1'b1;
                                rd_i2c_bit     <= 1'b0;
                            end               
                            7'b1111111: begin // NACK detected or a logic 1
                                rd_i2c_nack    <= 1'b1;
                                rd_i2c_bit     <= 1'b1;
                            end                     
                            default: begin
                                rd_i2c_start   <= 1'b0;
                                rd_i2c_stop    <= 1'b0;
                                rd_i2c_ack     <= 1'b0;
                                rd_i2c_nack    <= 1'b0;
                                rd_i2c_bit     <= 1'b0;
`ifdef DETECT_INVALID_TRANSITIONS
                                rd_i2c_bit_err <= 1'b1;
`endif                        
                            end
                        endcase
                    end
                end
            endcase
        end
    end


     always@(posedge scl_buff_slow[1], negedge resetn) begin
        if(~resetn)
            i2c_tbstate   <= I2C_INIT;
        else begin
            case(i2c_tbstate)
                I2C_INIT         : begin
                    i2c_tbstate <= I2C_STA;
                end
                I2C_STA          : begin
                    if(rd_i2c_start)
                        i2c_tbstate <= WR_SLV_ADDR_B6;
                end
                WR_SLV_ADDR_B6   : begin
                    i2c_tbstate   <= WR_SLV_ADDR_B5;
                end
                WR_SLV_ADDR_B5   : begin
                    i2c_tbstate   <= WR_SLV_ADDR_B4;
                end
                WR_SLV_ADDR_B4   : begin
                    i2c_tbstate   <= WR_SLV_ADDR_B3;
                end
                WR_SLV_ADDR_B3   : begin
                    i2c_tbstate   <= WR_SLV_ADDR_B2;
                end
                WR_SLV_ADDR_B2   : begin
                    i2c_tbstate   <= WR_SLV_ADDR_B1;
                end
                WR_SLV_ADDR_B1   : begin
                    i2c_tbstate   <= WR_SLV_ADDR_B0;
                end
                WR_SLV_ADDR_B0   : begin
                    i2c_tbstate   <= WR_RW_BIT;
                end
                WR_RW_BIT        : begin
                    i2c_tbstate <= WR_SLV_ACK1;
                end
                WR_SLV_ACK1      : begin
                    if(rd_i2c_ack)
                        i2c_tbstate  <= WR_REG_ADDR_B7;
                end
                WR_REG_ADDR_B7   : begin
                    i2c_tbstate <= WR_REG_ADDR_B6;
                end
                WR_REG_ADDR_B6   : begin
                    i2c_tbstate <= WR_REG_ADDR_B5;
                end
                WR_REG_ADDR_B5   : begin
                    i2c_tbstate <= WR_REG_ADDR_B4;
                end
                WR_REG_ADDR_B4   : begin
                    i2c_tbstate <= WR_REG_ADDR_B3;
                end
                WR_REG_ADDR_B3   : begin
                    i2c_tbstate <= WR_REG_ADDR_B2;
                end
                WR_REG_ADDR_B2   : begin
                    i2c_tbstate <= WR_REG_ADDR_B1;
                end
                WR_REG_ADDR_B1   : begin
                    i2c_tbstate <= WR_REG_ADDR_B0;
                end
                WR_REG_ADDR_B0   : begin
                    i2c_tbstate <= WR_SLV_ACK2;
                end
                WR_SLV_ACK2      : begin
                    if(rd_i2c_ack)
                        i2c_tbstate  <= WR_REG_VAL_B7;
                end
                WR_REG_VAL_B7    : begin
                    if(rd_i2c_start) // Restart - begin read cycle
                        i2c_tbstate  <= RD_SLV_ADDR_B6;
                    else if (rd_i2c_stop) // Stop - no more to write
                        i2c_tbstate  <= I2C_STA;
                    else //Continue write
                        i2c_tbstate  <= WR_REG_VAL_B6;
                end
                WR_REG_VAL_B6    : begin
                    i2c_tbstate  <= WR_REG_VAL_B5;
                end
                WR_REG_VAL_B5    : begin
                    i2c_tbstate  <= WR_REG_VAL_B4;
                end
                WR_REG_VAL_B4    : begin
                    i2c_tbstate  <= WR_REG_VAL_B3;
                end
                WR_REG_VAL_B3    : begin
                    i2c_tbstate  <= WR_REG_VAL_B2;
                end
                WR_REG_VAL_B2    : begin
                    i2c_tbstate  <= WR_REG_VAL_B1;
                end
                WR_REG_VAL_B1    : begin
                    i2c_tbstate  <= WR_REG_VAL_B0;
                end
                WR_REG_VAL_B0    : begin
                    i2c_tbstate  <= WR_SLV_ACK3;
                end
                WR_SLV_ACK3      : begin
                    if(rd_i2c_ack)
                        i2c_tbstate  <= WR_REG_VAL_B7;
                end
                RD_SLV_ADDR_B6   : begin
                    i2c_tbstate   <= RD_SLV_ADDR_B5;
                end
                RD_SLV_ADDR_B5   : begin
                    i2c_tbstate   <= RD_SLV_ADDR_B4;
                end
                RD_SLV_ADDR_B4   : begin
                    i2c_tbstate   <= RD_SLV_ADDR_B3;
                end
                RD_SLV_ADDR_B3   : begin
                    i2c_tbstate   <= RD_SLV_ADDR_B2;
                end
                RD_SLV_ADDR_B2   : begin
                    i2c_tbstate   <= RD_SLV_ADDR_B1;
                end
                RD_SLV_ADDR_B1   : begin
                    i2c_tbstate   <= RD_SLV_ADDR_B0;
                end
                RD_SLV_ADDR_B0   : begin
                    i2c_tbstate   <= RD_RW_BIT;
                end
                RD_RW_BIT        : begin
                    i2c_tbstate   <= RD_SLV_ACK1;
                end
                RD_SLV_ACK1      : begin
                    if(rd_i2c_ack)
                        i2c_tbstate  <= RD_REG_VAL_B7;
                end
                RD_REG_VAL_B7    : begin
                    if( (rd_i2c_start) && (rd_i2c_stop) )
                        i2c_tbstate  <= RD_REG_VAL_B6;
                end
                RD_REG_VAL_B6    : begin
                    i2c_tbstate  <= RD_REG_VAL_B5;
                end
                RD_REG_VAL_B5    : begin
                    i2c_tbstate  <= RD_REG_VAL_B4;
                end
                RD_REG_VAL_B4    : begin
                    i2c_tbstate  <= RD_REG_VAL_B3;
                end
                RD_REG_VAL_B3    : begin
                    i2c_tbstate  <= RD_REG_VAL_B2;
                end
                RD_REG_VAL_B2    : begin
                    i2c_tbstate  <= RD_REG_VAL_B1;
                end
                RD_REG_VAL_B1    : begin
                    i2c_tbstate  <= RD_REG_VAL_B0;
                end
                RD_REG_VAL_B0    : begin
                    i2c_tbstate  <= RD_MAST_ACK_NACK;
                end
                RD_MAST_ACK_NACK : begin
                    if(rd_i2c_nack)
                        i2c_tbstate  <= RD_REG_VAL_B7;
                    else
                        i2c_tbstate  <= RD_STO;
                end
                RD_STO           : begin
                    if (rd_i2c_stop)
                        i2c_tbstate  <= I2C_STA;
                end
                default : begin
                    i2c_tbstate   <= I2C_INIT;
                end
            endcase
        end
    end        
    
    always@(posedge i2c_clk_8x, negedge resetn) begin
        if(~resetn) begin
            rd_i2c_start  <= 1'b0;
            rd_i2c_stop   <= 1'b0;
            rw_bit        <= 1'b0;
            slave_addr    <= 7'd0;
            reg_addr      <= 8'd0;
            reg_value     <= 8'd0;
            cmd_i2c_ack   <= 1'b0;
        end
        else begin
            cmd_i2c_ack  <= 1'b0;
            case(i2c_tbstate)
                I2C_INIT         : begin
                    rw_bit      <= 1'b0;
                    slave_addr  <= 7'd0;
                    reg_addr    <= 8'd0;
                    reg_value   <= 8'd0;
                    cmd_i2c_ack <= 1'b0;
                end
                I2C_STA          : begin
                    if(rd_i2c_start) begin
                        rw_bit      <= 1'b0;
                        slave_addr  <= 7'd0;
                        reg_addr    <= 8'd0;
                        reg_value   <= 8'd0;
                        cmd_i2c_ack <= 1'b0;
                    end
                end
                WR_SLV_ADDR_B6   : begin
                    slave_addr[6] <= rd_i2c_bit;
                end
                WR_SLV_ADDR_B5   : begin
                    slave_addr[5] <= rd_i2c_bit;
                end
                WR_SLV_ADDR_B4   : begin
                    slave_addr[4] <= rd_i2c_bit;
                end
                WR_SLV_ADDR_B3   : begin
                    slave_addr[3] <= rd_i2c_bit;
                end
                WR_SLV_ADDR_B2   : begin
                    slave_addr[2] <= rd_i2c_bit;
                end
                WR_SLV_ADDR_B1   : begin
                    slave_addr[1] <= rd_i2c_bit;
                end
                WR_SLV_ADDR_B0   : begin
                    $display("%t: Access slave address = %h", $time, {slave_addr[6:1], rd_i2c_bit});
                    slave_addr[0] <= rd_i2c_bit;
                end
                WR_RW_BIT        : begin
                    rw_bit      <= rd_i2c_bit;
                    if(rd_i2c_bit == 1'b0) begin
                        $display("%t: I2C access is a write", $time);
                        cmd_i2c_ack <= 1'b1;
                    end
                    else begin
                        $display("%t: I2C access is a read", $time);
                        $display("%t: ERROR, must start with write to write slave address to access", $time);
                        $stop;
                    end
                end
                WR_SLV_ACK1      : begin
                    cmd_i2c_ack   <= 1'b0;
                    if(~rd_i2c_ack) begin
                        $display("%t: ERROR, slave didn't ack the slave address", $time);
                        $stop;
                    end
                end
                WR_REG_ADDR_B7   : begin
                    reg_addr[7] <= rd_i2c_bit;
                end
                WR_REG_ADDR_B6   : begin
                    reg_addr[6] <= rd_i2c_bit;
                end
                WR_REG_ADDR_B5   : begin
                    reg_addr[5] <= rd_i2c_bit;
                end
                WR_REG_ADDR_B4   : begin
                    reg_addr[4] <= rd_i2c_bit;
                end
                WR_REG_ADDR_B3   : begin
                    reg_addr[3] <= rd_i2c_bit;
                end
                WR_REG_ADDR_B2   : begin
                    reg_addr[2] <= rd_i2c_bit;
                end
                WR_REG_ADDR_B1   : begin
                    reg_addr[1] <= rd_i2c_bit;
                end
                WR_REG_ADDR_B0   : begin
                    reg_addr[0] <= rd_i2c_bit;
                    cmd_i2c_ack <= 1'b1;
                    $display("%t: Register address = %h", $time, {reg_addr[7:1], rd_i2c_bit});
                end
                WR_SLV_ACK2      : begin
                    cmd_i2c_ack   <= 1'b0;
                    if(~rd_i2c_ack) begin
                        $display("%t: ERROR, slave didn't ack the register address", $time);
                        $stop;
                    end
                end
                WR_REG_VAL_B7    : begin
                    if( (~rd_i2c_start) && (~rd_i2c_stop) )// Continue - read this bit
                        reg_value[7] <= rd_i2c_bit;
                end
                WR_REG_VAL_B6    : begin
                    reg_value[6] <= rd_i2c_bit;
                end
                WR_REG_VAL_B5    : begin
                    reg_value[5] <= rd_i2c_bit;
                end
                WR_REG_VAL_B4    : begin
                    reg_value[4] <= rd_i2c_bit;
                end
                WR_REG_VAL_B3    : begin
                    reg_value[3] <= rd_i2c_bit;
                end
                WR_REG_VAL_B2    : begin
                    reg_value[2] <= rd_i2c_bit;
                end
                WR_REG_VAL_B1    : begin
                    reg_value[1] <= rd_i2c_bit;
                end
                WR_REG_VAL_B0    : begin
                    reg_value[0] <= rd_i2c_bit;
                    cmd_i2c_ack  <= 1'b1;
                    $display("%t: Write register value = %h", $time, {reg_value[7:1], rd_i2c_bit});
                end
                WR_SLV_ACK3      : begin
                    cmd_i2c_ack   <= 1'b0;
                    if(~rd_i2c_ack) begin
                        $display("%t: ERROR, slave didn't ack the register value", $time);
                        $stop;
                    end
                end
                RD_SLV_ADDR_B6   : begin
                    slave_addr[6] <= rd_i2c_bit;
                end
                RD_SLV_ADDR_B5   : begin
                    slave_addr[5] <= rd_i2c_bit;
                end
                RD_SLV_ADDR_B4   : begin
                    slave_addr[4] <= rd_i2c_bit;
                end
                RD_SLV_ADDR_B3   : begin
                    slave_addr[3] <= rd_i2c_bit;
                end
                RD_SLV_ADDR_B2   : begin
                    slave_addr[2] <= rd_i2c_bit;
                end
                RD_SLV_ADDR_B1   : begin
                    slave_addr[1] <= rd_i2c_bit;
                end
                RD_SLV_ADDR_B0   : begin
                    slave_addr[0] <= rd_i2c_bit;
                    $display("%t: Read from slave address = %h", $time, {slave_addr[6:1], rd_i2c_bit});
                end
                RD_RW_BIT        : begin
                    rw_bit      <= rd_i2c_bit;
                    if(rd_i2c_bit == 1'b1) begin
                        $display("%t: I2C access is a read", $time);
                        cmd_i2c_ack <= 1'b1;
                    end
                    else begin
                        $display("%t: I2C access is a write", $time);
                        $display("%t: ERROR, must start read cycle with rw bit set", $time);
                        $stop;
                    end
                end
                RD_SLV_ACK1      : begin
                    cmd_i2c_ack   <= 1'b0;
                    if(~rd_i2c_ack) begin
                        $display("%t: ERROR, slave didn't ack the slave address", $time);
                        $stop;
                    end
                end
                RD_REG_VAL_B7    : begin
                    if(rd_i2c_start) begin // Restart - error
                        $display("%t: ERROR: Repeated start during read", $time);
                        $stop;
                    end
                    else if (rd_i2c_stop) begin // Stop - no more to read
                        $display("%t: ERROR: Stop asserted during read with out NACK from master", $time);
                        $stop;
                    end
                    else begin //Continue read
                        reg_value[7] <= rd_i2c_bit;
                    end
                end
                RD_REG_VAL_B6    : begin
                    reg_value[6] <= rd_i2c_bit;
                end
                RD_REG_VAL_B5    : begin
                    reg_value[5] <= rd_i2c_bit;
                end
                RD_REG_VAL_B4    : begin
                    reg_value[4] <= rd_i2c_bit;
                end
                RD_REG_VAL_B3    : begin
                    reg_value[3] <= rd_i2c_bit;
                end
                RD_REG_VAL_B2    : begin
                    reg_value[2] <= rd_i2c_bit;
                end
                RD_REG_VAL_B1    : begin
                    reg_value[1] <= rd_i2c_bit;
                end
                RD_REG_VAL_B0    : begin
                    reg_value[0] <= rd_i2c_bit;
                    cmd_i2c_ack  <= 1'b0;
                    $display("%t: Read register value = %h", $time, {reg_value[6:1], rd_i2c_bit});
                end
                RD_MAST_ACK_NACK : begin
                    cmd_i2c_ack   <= 1'b0;
                end
                RD_STO           : begin
                    if(rd_i2c_start) begin // Restart - error
                        $display("%t: ERROR: Repeated start during read", $time);
                        $stop;
                    end
                    else if (rd_i2c_stop) begin // Stop - no more to read
                        $display("%t: Stop asserted, terminating I2C read cycle", $time);
                    end
                    else begin
                        $display("%t: ERROR: Master didn't STO following NACK from master", $time);
                        $stop;
                    end
                end
                default : begin
                    rd_i2c_start  <= 1'b0;
                    rd_i2c_stop   <= 1'b0;
                    rw_bit        <= 1'b0;
                    slave_addr    <= 7'd0;
                    reg_addr      <= 8'd0;
                    reg_value     <= 8'd0;
                end
            endcase
        end
    end

    always@(posedge rd_i2c_start)
        if (DUT.delay_timer_done)$display("%t: I2C start asserted", $time);
        
    always@(posedge rd_i2c_stop)
        if (DUT.delay_timer_done)$display("%t: I2C stop asserted", $time);
        
    always@(i2c_count)
        if (DUT.delay_timer_done)$display("%t: I2C byte count=%d", $time, i2c_count);

    always@(cmd_i2c_ack)
        if (DUT.delay_timer_done)$display("%t: I2C cmd_i2c_ack=%b", $time, cmd_i2c_ack);

`ifdef DETECT_INVALID_TRANSITIONS
    always@(posedge rd_i2c_bit_err) begin
        if (DUT.delay_timer_done) begin
            $display("%t: ERROR invalid transition while SCL high", $time, rd_i2c_bit_err);
            $stop;
        end
    end
`endif

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
    assign ( pull1, strong0 ) sda_1 = cmd_i2c_ack ? 1'b0: 1'b1;
    
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

