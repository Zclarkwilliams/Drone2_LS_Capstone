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

 *  Module inouts:
 *          inout scl_1,          I2C Primary   EFB SDA wire - Module only uses Primary or secondary I2C, but can use either one
 *          inout scl_2,          I2C Secondary EFB SDA wire
 *          inout sda_1,          I2C Primary   EFB SDA wire
 *          inout sda_2,          I2C Secondary EFB SDA wire

 *  Module takes as inputs:
 *          sys_clk                 master clock
 *          next_mod_active         Handshake signal from next module acknowledging the data valid strobe
 *          resetn                    async negative reset signal 0 = reset, 1 = not reset

 * Module provides as output (all values are 16-bit, 2's complement):
 *          led_data_out,          Module calibration status output for LED indication of IMU operating state
 *          resetn_imu             Low active reset signal to IMU hardware to trigger reset
 *          imu_good               The IMU is either in an error or initial bootup states, measurements not yet active
 *          valid_strobe           Strobe signal that indicates the end of the data collection poll, subsequent modules key off this strobe.
 *          accel_rate_x           Accelerometer X-Axis                Precision: 1 m/s^2 = 100 LSB
 *          accel_rate_y           Accelerometer Y-Axis                Precision: 1 m/s^2 = 100 LSB
 *          accel_rate_z           Accelerometer Z-Axis                Precision: 1 m/s^2 = 100 LSB
 *          magneto_rate_x         Magnetometer X-Axis                 Precision: 1uT = 16 LSB
 *          magneto_rate_y         Magnetometer Y-Axis                 Precision: 1uT = 16 LSB
 *          magneto_rate_z         Magnetometer Z-Axis                 Precision: 1uT = 16 LSB
 *          gyro_rate_x            Gyroscope X-Axis                    Precision: Dps = 16 LSB
 *          gyro_rate_y            Gyroscope Y-Axis                    Precision: Dps = 16 LSB
 *          gyro_rate_z            Gyroscope Z-Axis                    Precision: Dps = 16 LSB
 *          euler_angle_x          Euler angle X-Axis                  Precision: Deg = 16 LSB
 *          euler_angle_y          Euler angle Y-Axis                  Precision: Deg = 16 LSB
 *          euler_angle_z          Euler angle Z-Axis                  Precision: Deg = 16 LSB
 *          quaternion_data_w      Quaternion X-Axis                   Precision: Unit = 2^14 LSB
 *          quaternion_data_x      Quaternion X-Axis                   Precision: Unit = 2^14 LSB
 *          quaternion_data_y      Quaternion Y-Axis                   Precision: Unit = 2^14 LSB
 *          quaternion_data_z      Quaternion Z-Axis                   Precision: Unit = 2^14 LSB
 *          linear_accel_x         Linear Acceleration X-Axis          Precision: 1 m/s^2 = 100 LSB
 *          linear_accel_y         Linear Acceleration Y-Axis          Precision: 1 m/s^2 = 100 LSB
 *          linear_accel_z         Linear Acceleration Z-Axis          Precision: 1 m/s^2 = 100 LSB
 *          gravity_accel_x        Gravitational Acceleration X-Axis   Precision: 1 m/s^2 = 100 LSB
 *          gravity_accel_y        Gravitational Acceleration Y-Axis   Precision: 1 m/s^2 = 100 LSB
 *          gravity_accel_z        Gravitational Acceleration Z-Axis   Precision: 1 m/s^2 = 100 LSB
 *          temperature            Temperature in degrees Celsius      Precision: 1 Deg C = 1 LSB
 *          calib_status           Calibration status register
 *
 * TODO: Add module description with description of inputs/outputs
 *         instead of with the variable. Make this file look like all of
 *         the other files.
 */

`timescale 1ns / 1ns
`include "common_defines.v"
`include "bno055_defines.v"
`include "vl53l1x_defines.v"
`include "i2c_driver_defines.v"

module i2c_device_driver #(
    parameter INIT_INTERVAL = 16'd10_000, // 10 seconds (in ms) start-up time
    parameter POLL_INTERVAL = 16'd20      // 20 ms between data polls
)
(
    inout  wire scl_1,
    inout  wire scl_2,
    inout  wire sda_1,
    inout  wire sda_2,
    input  wire resetn,
//    output reg  [7:0]led_data_out,
    output wire [7:0]led_data_out,
    output wire [7:0]i2c_top_debug,                         //  Debug signals for I2C top
    input  wire sys_clk,
    input  wire next_mod_active,
    output wire resetn_imu,
    output wire resetn_lidar,
    output reg  imu_good,
    output reg  valid_strobe,
    output reg [15:0]accel_rate_x,
    output reg [15:0]accel_rate_y,
    output reg [15:0]accel_rate_z,
    output reg [15:0]magneto_rate_x,
    output reg [15:0]magneto_rate_y,
    output reg [15:0]magneto_rate_z,
    output reg [15:0]gyro_rate_x,
    output reg [15:0]gyro_rate_y,
    output reg [15:0]gyro_rate_z,
    output reg [15:0]euler_angle_x,
    output reg [15:0]euler_angle_y,
    output reg [15:0]euler_angle_z,
    output reg [15:0]quaternion_data_w,
    output reg [15:0]quaternion_data_x,
    output reg [15:0]quaternion_data_y,
    output reg [15:0]quaternion_data_z,
    output reg [15:0]linear_accel_x,
    output reg [15:0]linear_accel_y,
    output reg [15:0]linear_accel_z,
    output reg [15:0]gravity_accel_x,
    output reg [15:0]gravity_accel_y,
    output reg [15:0]gravity_accel_z,
    output reg [7:0]temperature,
    output reg [7:0]calib_status,
    output reg [15:0]vl53l1x_chip_id
);

    reg  read_write_in, next_read_write_in;           //  Value and next value of signal to i2c module to indicate read or write transaction, 1 = read, 0 = write
    reg  go;                                          //  Flag to i2c module signaling start of i2c transaction. All inputs must be valid before asserting this bit
    reg  next_go_flag;                                //  Next value of the i2c module GO flag
    wire one_byte_ready;                              //  Flag from i2c module indicating that a byte has been received, data_rx is valid
    wire busy;                                        //  Flag from i2c module indicating that a transaction is in progress
    reg  [6:0]slave_address;                          //  Slave address of device being accessed
    reg  [6:0]next_slave_address;                     //  Next value of slave address of device being accessed
    reg  [15:0]data_reg;                              //  Register address of device being accessed
    reg  [15:0]next_data_reg;                         //  Next value of register address 
    reg  [7:0]data_tx;                                //  Data written to register of device being accessed
    reg  [7:0]next_data_tx;                           //  Next value of data written to register of device being accessed
    wire [7:0]data_rx;                                //  Receives an RX data byte from i2c module
    reg  [`I2C_DRV_STATE_BITS-1:0]i2c_state;          //  State for i2c command sequence FSM
    reg  [`I2C_DRV_STATE_BITS-1:0]next_i2c_state;     //  Next FSM state
    reg  [`I2C_DRV_STATE_BITS-1:0]return_state;       //  FSM return state from i2c sub state
    reg  [`I2C_DRV_STATE_BITS-1:0]next_return_state;  //  Next value for FSM return state
    reg  [15:0]count_ms;                              //  Count number of milliseconds for delay timer, used to generate N ms delay trigger
    reg  clear_waiting_ms;                            //  Reset waiting X ms timer.
    reg  count_ms_init_time;                          //  Set count_ms timer to init time when set (Defaults to 650 ms) or regular polling interval (20 ms) when clear
    reg  [5:0]target_read_count;                      //  The number of bytes to access for a read command (Writes are always for a single byte)
    reg  [5:0]next_target_read_count;                 //  Next value of target_read_count
    reg  [5:0]data_rx_reg_index;                      //  Index in data_rx_reg for current byte
    reg  [5:0]vl53l1x_data_rx_reg_index;              //  Index in data_rx_reg for current byte from vl53l1x
    reg  [5:0]led_view_index;                         //  Index in data_rx_reg that is being monitored with status LEDs
    reg  [5:0]next_led_view_index;                    //  Next value of LED View Index
    reg  [7:0]data_rx_reg[`BNO055_DATA_RX_BYTE_REG_CNT:0];            //  Store all measurement bytes from BNO055 read burst
    reg  [7:0]vl53l1x_data_rx_reg[`VL53L1X_DATA_RX_BYTE_REG_CNT-1:0]; //  Store all received data bytes from VL53L1X
    reg  resetn_buffer;                               //  Negedge clears received measurement buffer
    reg  rx_data_latch_strobe;                        //  Strobe data output register, latch onto current data in rx buffer, asynchronous latch
    reg  rx_data_latch_tmp;                           //  Synchronously latched value of the data latch strobe
    reg  next_imu_good;                               //  Next value of module imu_good bit
    reg  i2c_number;                                  //  The i2c module to call, 0 = i2c EFB #1, 1 = i2c EFB #2
    reg  rx_from_vl53l1x;                             //  Receiving from VL53L1X = 1, from BNO055 = 0
    reg  next_rx_from_vl53l1x;                        //  Receiving from VL53L1X = 1, from BNO055 = 0
    reg [5:0]cal_restore_index;                       //  Current calibration value that is being written
    reg [7:0]cal_reg_addr;                            //  Current IMU register address that this calibration data is destined for
    reg clear_cal_restore_index;                      //  Reset calibration restore index and register addresses back to starting value
    reg increment_cal_restore_index;                  //  Increment calibration restore index and register addresses by 1
    reg calibrated_once;                              //  Flag that specifies whether the calibration has been restored once yet or not, used to run calibration twice
    reg next_calibrated_once;                         //  The next value of the calibrated once flag
    reg valid_strobe_enable;                          //  Enables the valid_strobe for one or two clock cycles
    reg [20:0]master_trigger_count_ms;                //  Counter used to generate a periodic 20ms timer tick.
    reg [16:0]count_sys_clk_for_ms;                   //  Counts number of sys clock ticks and generate a pulse (1 sys_clk duration) every 38k (1ms)
    reg delay_timer_done;                             //  Count down timer has reached final value, 0 = no, 1 = yes
    reg delay_timer_started;                          //  Count down timer started, 0 = no, 1 = yes
    reg delay_timer_at_init;                          //  Count down timer at init value, 0 = no, 1 = yes
    reg delay_timer_at_poll;                          //  Count down timer at polling value, 0 = no, 1 = yes
    reg is_2_byte_reg;                                //  This I2C device uses 2 byte registers, boolean 1 = true, 0 = false - If yes, then MSB transmitted first, then LSB
    reg next_is_2_byte_reg;                           //  Next value of I2C 2 byte register bool


    //
    //  Module body
    //
    // Changed this from 81 to 41 since LED2 is burned out on the board I am testing with
    //assign led_data_out = (i2c_state    <= `I2C_DRV_STATE_BOOT_WAIT ) ? 8'h41 : data_rx_reg[led_view_index]; //  Output for calibration status LEDs OR indicates that the IMU is in reset
    assign led_data_out = return_state;
    //assign led_data_out = i2c_state;
    
    
    
    
    // resetn_lidar follows resetn_imu - The ST Microelectronics VL53L1X has a low active reset 0 = shutdown, 1 = run, same as Bosch BNO055
    assign resetn_lidar = resetn_imu;


    //  Instantiate i2c driver
    i2c_module i2c( .scl_1(scl_1),
                    .sda_1(sda_1),
                    .scl_2(scl_2),
                    .sda_2(sda_2),
                    .resetn(resetn),
                    .resetn_imu(resetn_imu),
                    .target_read_count(target_read_count),
                    .slave_address(slave_address),
                    .module_data_out(data_rx),
                    .module_data_in(data_tx),
                    .module_reg_in(data_reg),
                    .read_write_in(read_write_in),
                    .is_2_byte_reg(is_2_byte_reg),
                    .go(go),
                    .busy(busy),
                    .one_byte_ready(one_byte_ready),
                    .i2c_number(i2c_number),
                    .sys_clk(sys_clk),
                    .i2c_top_debug(i2c_top_debug)
    );
    
    
    //  Generates a multiple of 1ms length duration delay trigger - Defaulted to 650 ms for BNO055 reset and boot time
    //  When the count down counter wraps around the timer is triggered and stops counting
    always@(negedge sys_clk, negedge resetn) begin
        if(~resetn) begin
            //$display("nMs timer reset");
            count_ms             <= INIT_INTERVAL;
            count_sys_clk_for_ms <= `WAIT_MS_DIVIDER - 17'd1;
            delay_timer_done     <= `FALSE;
            delay_timer_started  <= `FALSE;
            delay_timer_at_init  <= `FALSE;
            delay_timer_at_poll  <= `FALSE;
        end
        else if( clear_waiting_ms == `CLEAR_MS_TIMER ) begin  //Timer clear asserted, set initial values
            //$display("nMs timer cleared");
            // Set to value minus 2 because we count down to 0 and wrap around, which takes 2 additional ticks
            count_sys_clk_for_ms <= `WAIT_MS_DIVIDER - 17'd2;
            delay_timer_started  <= `FALSE;
            delay_timer_done     <= `FALSE;
            delay_timer_at_init  <= `FALSE;
            if (count_ms_init_time) begin
                //$display("nMs timer set to init interval");
                count_ms            <= INIT_INTERVAL - 16'd2; // Set delay to startup time (650 ms for BNO055 to initialize)  
                delay_timer_at_init <= `TRUE;
                delay_timer_at_poll <= `FALSE;
            end
            else begin
                //$display("nMs timer set to poll interval");
                count_ms            <= POLL_INTERVAL - 16'd2; // Normal 20 ms interval between data polls
                delay_timer_at_init <= `FALSE;
                delay_timer_at_poll <= `TRUE;
            end
        end
        else if(count_sys_clk_for_ms[16]) begin    //sys_clk tick counter wrapped around, trigger 1 ms update
            delay_timer_started      <= `TRUE;
            delay_timer_at_init      <= `FALSE;
            delay_timer_at_poll      <= `FALSE;
            if( ~count_ms[15]) begin               // Only count down ms timer if not wrapped around yet
                //$display("nMs timer wrapped on sys_clk but not ms timer");
                count_sys_clk_for_ms <= `WAIT_MS_DIVIDER - 16'd1;
                count_ms             <= count_ms - 16'd1;     
                delay_timer_done     <= `FALSE;
            end
            else begin                             //Both counters wrapped around, we're done!
                //$display("nMs timer wrapped on sys_clk AND ms timer");
                count_sys_clk_for_ms <= count_sys_clk_for_ms;
                count_ms             <= count_ms;     
                delay_timer_done     <= `TRUE;
            end
        end
        else begin
            //$display("nMs timer decrement sys_clk counter");
            count_sys_clk_for_ms     <= count_sys_clk_for_ms - 16'd1;
            delay_timer_done         <= `FALSE;
            delay_timer_at_init      <= `FALSE;
            delay_timer_at_poll      <= `FALSE;
            delay_timer_started      <= `TRUE;
            count_ms                 <= count_ms;
        end
    end

    
    
    //  During a read cycle increment the data_rx_reg_index until it reaches the end of  data_rx_reg
    //  If a byte has been read (one_byte_ready is asserted) assign it to the data_rx_reg byte array at the location specified by data_rx_reg_index
    always@(posedge sys_clk, negedge resetn_buffer, negedge resetn) begin
        if(~resetn) begin
            // Initialize data rx register to all 0s on reset
            for(data_rx_reg_index = 0; data_rx_reg_index < `BNO055_DATA_RX_BYTE_REG_CNT; data_rx_reg_index = data_rx_reg_index+1'b1)
                data_rx_reg[data_rx_reg_index] <= 8'b0;
            data_rx_reg_index <= 0;
            for(vl53l1x_data_rx_reg_index = 0; vl53l1x_data_rx_reg_index < `VL53L1X_DATA_RX_BYTE_REG_CNT; vl53l1x_data_rx_reg_index = vl53l1x_data_rx_reg_index+1'b1)
                vl53l1x_data_rx_reg[vl53l1x_data_rx_reg_index] <= 8'b0;
            vl53l1x_data_rx_reg_index <= 0;
        end
        else if(~resetn_buffer ) begin
            data_rx_reg_index <= 0;
            vl53l1x_data_rx_reg_index <= 0;
        end
        else if (one_byte_ready) begin
            if (rx_from_vl53l1x) begin
                // If the index is pointing to the last index in the array, then rest pointer
                // and write this byte to the start of the array
                if(vl53l1x_data_rx_reg_index == (`VL53L1X_DATA_RX_BYTE_REG_CNT - 1'b1)) begin
                    vl53l1x_data_rx_reg_index                      <= 0;
                    vl53l1x_data_rx_reg[vl53l1x_data_rx_reg_index] <= data_rx;
                end
                //  Otherwise, just write the byte to the data_rx_reg_index index in the byte array
                else begin
                    vl53l1x_data_rx_reg[vl53l1x_data_rx_reg_index] <= data_rx;
                    vl53l1x_data_rx_reg_index                      <= vl53l1x_data_rx_reg_index + 1'b1;
                end
            end
            else begin //Receiving bytes from BNO055
                // If the index is pointing to the last index in the array, then rest pointer
                // and write this byte to the start of the array
                if(data_rx_reg_index == (`BNO055_DATA_RX_BYTE_REG_CNT - 1'b1)) begin
                    data_rx_reg_index              <= 0;
                    data_rx_reg[data_rx_reg_index] <= data_rx;
                end
                //  Otherwise, just write the byte to the data_rx_reg_index index in the byte array
                else begin
                    data_rx_reg[data_rx_reg_index] <= data_rx;
                    data_rx_reg_index              <= data_rx_reg_index + 1'b1;
                end
            end
        end
    end

    //  Generates a 20ms countdown timer that enables module output valid strobe when it reaches 0
    //  When timer wraps around the enable signal is set for clock tick, or delayed for 1 additional tick
    //  If rx_data_latch_strobe is not asserted then the enable signal is asserted
    //  Otherwise it will be asserted the next clock tick
    always@(posedge sys_clk, negedge resetn) begin
        if(~resetn) begin  // Reset, set starting values
            master_trigger_count_ms <= `WAIT_MS_DIVIDER * POLL_INTERVAL;
            valid_strobe_enable     <= `FALSE;
        end 
        //  Timer wrapped around and rx_data_latch_strobe not asserted, reset timer and assert enable
        else if( master_trigger_count_ms[20] == `TRUE && ~rx_data_latch_strobe) begin
            master_trigger_count_ms <= `WAIT_MS_DIVIDER * POLL_INTERVAL;
            valid_strobe_enable     <= `TRUE;
        end 
        //  Timer wrapped around and rx_data_latch_strobe is asserted, leave timer and do not assert enable
        else if( master_trigger_count_ms[20] == `TRUE && rx_data_latch_strobe) begin
            master_trigger_count_ms <= master_trigger_count_ms;
            valid_strobe_enable     <= `FALSE;
        end
        //  Timer has not wrapped around, just decrement by 1 every 1ms, otherwise don't decrement timer
        else begin
            master_trigger_count_ms <= (master_trigger_count_ms - 1'b1);
            valid_strobe_enable     <= `FALSE;
        end
    end

    //  Handle output valid_strobe enable and handshake with following modules
    //  The modules after this run at a slower clock rate and require handshaking of this signal
    //  This block will hold valid_strobe high until the next module's active signal goes high
    //  Which acknowledges that receipt of this valid_strobe
    always@(posedge sys_clk, negedge resetn) begin
        if(~resetn)
            valid_strobe     <= `LOW;
        else if (valid_strobe_enable == `TRUE) begin
            if(~valid_strobe)                             // Valid not yet asserted
                valid_strobe <= `HIGH;
            else if( valid_strobe && (~next_mod_active))  // Hold strobe until the next module active
                valid_strobe <= `HIGH;
            else                                          // De-assert valid strobe
                valid_strobe <= `LOW;
        end
        else begin
            if( valid_strobe && (~next_mod_active))       // Hold strobe until the next module is active
                valid_strobe <= `HIGH;
            else
                valid_strobe <= `LOW;
        end
    end

    //  Take data read byte array and assign the byte values to output data wires
    //  Most of the data outputs are 16 bit words
    always@(posedge sys_clk, negedge resetn) begin
        if(~resetn) begin
            accel_rate_x      <= 16'b0;
            accel_rate_y      <= 16'b0;
            accel_rate_z      <= 16'b0;
            magneto_rate_x    <= 16'b0;
            magneto_rate_y    <= 16'b0;
            magneto_rate_z    <= 16'b0;
            gyro_rate_x       <= 16'b0;
            gyro_rate_y       <= 16'b0;
            gyro_rate_z       <= 16'b0;
            euler_angle_x     <= 16'b0;
            euler_angle_y     <= 16'b0;
            euler_angle_z     <= 16'b0;
            quaternion_data_w <= 16'b0;
            quaternion_data_x <= 16'b0;
            quaternion_data_y <= 16'b0;
            quaternion_data_z <= 16'b0;
            linear_accel_x    <= 16'b0;
            linear_accel_y    <= 16'b0;
            linear_accel_z    <= 16'b0;
            gravity_accel_x   <= 16'b0;
            gravity_accel_y   <= 16'b0;
            gravity_accel_z   <= 16'b0;
            temperature       <= 8'b0;
            calib_status      <= 8'b0;
            vl53l1x_chip_id   <= 16'b0;
        end
        else if(rx_data_latch_strobe) begin
            accel_rate_x      <= {data_rx_reg[`ACC_DATA_X_MSB_INDEX],data_rx_reg[`ACC_DATA_X_LSB_INDEX]};
            accel_rate_y      <= {data_rx_reg[`ACC_DATA_Y_MSB_INDEX],data_rx_reg[`ACC_DATA_Y_LSB_INDEX]};
            accel_rate_z      <= {data_rx_reg[`ACC_DATA_Z_MSB_INDEX],data_rx_reg[`ACC_DATA_Z_LSB_INDEX]};
            magneto_rate_x    <= {data_rx_reg[`MAG_DATA_X_MSB_INDEX],data_rx_reg[`MAG_DATA_X_LSB_INDEX]};
            magneto_rate_y    <= {data_rx_reg[`MAG_DATA_Y_MSB_INDEX],data_rx_reg[`MAG_DATA_Y_LSB_INDEX]};
            magneto_rate_z    <= {data_rx_reg[`MAG_DATA_Z_MSB_INDEX],data_rx_reg[`MAG_DATA_Z_LSB_INDEX]};
            gyro_rate_x       <= {data_rx_reg[`GYR_DATA_X_MSB_INDEX],data_rx_reg[`GYR_DATA_X_LSB_INDEX]};
            gyro_rate_y       <= {data_rx_reg[`GYR_DATA_Y_MSB_INDEX],data_rx_reg[`GYR_DATA_Y_LSB_INDEX]};
            gyro_rate_z       <= {data_rx_reg[`GYR_DATA_Z_MSB_INDEX],data_rx_reg[`GYR_DATA_Z_LSB_INDEX]};
            euler_angle_x     <= {data_rx_reg[`EUL_DATA_X_MSB_INDEX],data_rx_reg[`EUL_DATA_X_LSB_INDEX]};
            euler_angle_y     <= {data_rx_reg[`EUL_DATA_Y_MSB_INDEX],data_rx_reg[`EUL_DATA_Y_LSB_INDEX]};
            euler_angle_z     <= {data_rx_reg[`EUL_DATA_Z_MSB_INDEX],data_rx_reg[`EUL_DATA_Z_LSB_INDEX]};
            quaternion_data_w <= {data_rx_reg[`QUA_DATA_W_MSB_INDEX],data_rx_reg[`QUA_DATA_W_LSB_INDEX]};
            quaternion_data_x <= {data_rx_reg[`QUA_DATA_X_MSB_INDEX],data_rx_reg[`QUA_DATA_X_LSB_INDEX]};
            quaternion_data_y <= {data_rx_reg[`QUA_DATA_Y_MSB_INDEX],data_rx_reg[`QUA_DATA_Y_LSB_INDEX]};
            quaternion_data_z <= {data_rx_reg[`QUA_DATA_Z_MSB_INDEX],data_rx_reg[`QUA_DATA_Z_LSB_INDEX]};
            linear_accel_x    <= {data_rx_reg[`LIN_DATA_X_MSB_INDEX],data_rx_reg[`LIN_DATA_X_LSB_INDEX]};
            linear_accel_y    <= {data_rx_reg[`LIN_DATA_Y_MSB_INDEX],data_rx_reg[`LIN_DATA_Y_LSB_INDEX]};
            linear_accel_z    <= {data_rx_reg[`LIN_DATA_Z_MSB_INDEX],data_rx_reg[`LIN_DATA_Z_LSB_INDEX]};
            gravity_accel_x   <= {data_rx_reg[`GRA_DATA_X_MSB_INDEX],data_rx_reg[`GRA_DATA_X_LSB_INDEX]};
            gravity_accel_y   <= {data_rx_reg[`GRA_DATA_Y_MSB_INDEX],data_rx_reg[`GRA_DATA_Y_LSB_INDEX]};
            gravity_accel_z   <= {data_rx_reg[`GRA_DATA_Z_MSB_INDEX],data_rx_reg[`GRA_DATA_Z_LSB_INDEX]};
            temperature       <= data_rx_reg[`TEMPERATURE_DATA_INDEX];
            calib_status      <= data_rx_reg[`CALIBRATION_DATA_INDEX];
            vl53l1x_chip_id   <= {vl53l1x_data_rx_reg[0],vl53l1x_data_rx_reg[1]};
        end
    end

    // Increment calibration restore index and calibration register address by 1 or clear back to initial values
    always@(posedge sys_clk) begin
        if( ~clear_cal_restore_index) begin
            cal_reg_addr      <= `BNO055_ACCEL_OFFSET_X_LSB_ADDR;
            cal_restore_index <= 0;
        end
        else if( increment_cal_restore_index) begin
            cal_reg_addr      <= (cal_reg_addr      + 1'b1);
            cal_restore_index <= (cal_restore_index + 1'b1);
        end
        else begin
            cal_reg_addr      <= cal_reg_addr;
            cal_restore_index <= cal_restore_index;
        end
    end


    // Advance state and registered data at each positive clock edge
    always@(posedge sys_clk, negedge resetn) begin
        if(~resetn) begin
            data_reg            <= `ALL_ZERO_2BYTE;
            data_tx             <= `BYTE_ALL_ZERO;
            read_write_in       <= `I2C_READ;
            go                  <= `NOT_GO;
            i2c_state           <= `I2C_DRV_STATE_RESET;
            return_state        <= `FALSE;
            target_read_count   <= `FALSE;
            led_view_index      <= `FALSE;
            slave_address       <= `BNO055_SLAVE_ADDRESS;
            imu_good            <= `FALSE;
            calibrated_once     <= `FALSE;
            rx_data_latch_tmp   <= `LOW;
            rx_from_vl53l1x     <= `FALSE;
            is_2_byte_reg       <= `FALSE;
        end
        else begin
            data_reg            <= next_data_reg;
            data_tx             <= next_data_tx;
            read_write_in       <= next_read_write_in;
            go                  <= next_go_flag;
            i2c_state           <= next_i2c_state;
            return_state        <= next_return_state;
            target_read_count   <= next_target_read_count;
            led_view_index      <= next_led_view_index;
            slave_address       <= next_slave_address;
            imu_good            <= next_imu_good;
            calibrated_once     <= next_calibrated_once;
            rx_data_latch_tmp   <= rx_data_latch_strobe;
            rx_from_vl53l1x     <= next_rx_from_vl53l1x;
            is_2_byte_reg       <= next_is_2_byte_reg;
        end
    end


    // IMU FSM, Determine next state of FSM and drive i2c module inputs
    always@(*) begin
        if( ~(resetn & resetn_imu) ) begin
            next_imu_good             = `FALSE;
            clear_waiting_ms          = `RUN_MS_TIMER;
            count_ms_init_time        = `FALSE;
            next_i2c_state            = `I2C_DRV_STATE_RESET;
            next_return_state         = `I2C_DRV_STATE_RESET;
            next_go_flag              = `NOT_GO;
            next_data_reg             = `ALL_ZERO_2BYTE;
            next_data_tx              = `BYTE_ALL_ZERO;
            next_read_write_in        = `I2C_READ;
            next_led_view_index       = `FALSE;
            next_rx_from_vl53l1x      = `FALSE;
            next_is_2_byte_reg        = `FALSE;
            resetn_buffer             = `LOW;
            next_target_read_count    = 1'b1;
            rx_data_latch_strobe      = `LOW;
            i2c_number                = 1'b0; // Default to i2c EFB #1
            next_slave_address        = `BNO055_SLAVE_ADDRESS;
            increment_cal_restore_index = 1'b0;
            clear_cal_restore_index     = 1'b0;
        end
        else begin
            // Default to preserve these values, can be altered in lower steps
            next_imu_good             = imu_good;
            clear_waiting_ms          = `RUN_MS_TIMER;
            count_ms_init_time        = `FALSE;
            next_go_flag              = `NOT_GO;
            next_i2c_state            = i2c_state;
            next_return_state         = return_state;
            next_data_reg             = data_reg;
            next_data_tx              = data_tx;
            next_read_write_in        = read_write_in;
            next_led_view_index       = led_view_index;
            next_rx_from_vl53l1x      = rx_from_vl53l1x;
            next_is_2_byte_reg        = is_2_byte_reg;
            resetn_buffer             = `HIGH;
            next_target_read_count    = target_read_count;
            rx_data_latch_strobe      = `LOW;
            i2c_number                = 1'b0; // Default to i2c EFB #1
            next_slave_address        = slave_address;
            next_calibrated_once      = calibrated_once;
            increment_cal_restore_index = 1'b0;
            clear_cal_restore_index     = 1'b1;
            case(i2c_state)
                `I2C_DRV_STATE_RESET: begin
                    next_imu_good      = `FALSE;
                    clear_waiting_ms   = `RUN_MS_TIMER;
                    count_ms_init_time = `TRUE;
                    if(~delay_timer_started) //Timer not yet set to starting value
                        next_i2c_state      = `I2C_DRV_STATE_RESET;
                    else
                        next_i2c_state      = `I2C_DRV_STATE_BOOT;
                    next_slave_address      = `BNO055_SLAVE_ADDRESS;
                    clear_cal_restore_index = 1'b0;
                    next_calibrated_once    = 1'b0;
                    next_rx_from_vl53l1x    = `FALSE;
                end
                `I2C_DRV_STATE_BOOT: begin
                    next_imu_good      = `FALSE;
                    clear_waiting_ms   = `CLEAR_MS_TIMER; // Clear and set to wait_ms value
                    count_ms_init_time = `TRUE;
                    // Wait for I2C to boot then start wait for IMU
                    // Also wait for timer to initialize to INIT_INTERVAL
                    if(~busy && delay_timer_at_init)
                        next_i2c_state = `I2C_DRV_STATE_BOOT_WAIT;
                    else 
                        next_i2c_state = `I2C_DRV_STATE_BOOT;
                    next_slave_address = `BNO055_SLAVE_ADDRESS;
                end
                `I2C_DRV_STATE_BOOT_WAIT: begin
                    next_imu_good      = `FALSE;
                    clear_waiting_ms   = `RUN_MS_TIMER;
                    count_ms_init_time = `FALSE;  
                    next_i2c_state     = `I2C_DRV_STATE_BOOT_WAIT;
                    next_slave_address = `BNO055_SLAVE_ADDRESS;
                    // Wait for I2C to be not busy and delay timer done
                    if(~busy && delay_timer_done)
                        next_i2c_state     = `I2C_VL53L1X_STATE_READ_CHIP_ID;
                        //next_i2c_state     = `I2C_BNO055_STATE_READ_CHIP_ID;
                    else
                        next_i2c_state     = `I2C_DRV_STATE_BOOT_WAIT;
                end
                `I2C_VL53L1X_STATE_READ_CHIP_ID: begin
                    next_imu_good          = `FALSE;
                    // Set the wrong slave address to make sure that the issue isn't something with the VL53L1X and not the code
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    //next_slave_address     = `BNO055_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_BNO055_STATE_READ_CHIP_ID;
                    next_data_reg          = `VL53L1X_IDENTIFICATION__MODEL_ID;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_read_write_in     = `I2C_READ;
                    next_rx_from_vl53l1x   = `TRUE;
                    next_is_2_byte_reg     = `TRUE;
                    next_target_read_count = 5'd2;
                    next_led_view_index    = 1'b0;
                end
                `I2C_BNO055_STATE_READ_CHIP_ID: begin // Page 0
                    next_imu_good          = `FALSE;
                    next_slave_address     = `BNO055_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_BNO055_STATE_SET_UNITS;
                    next_data_reg          = `BNO055_CHIP_ID_ADDR;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_read_write_in     = `I2C_READ;
                    next_rx_from_vl53l1x   = `FALSE;
                    next_is_2_byte_reg     = `FALSE;
                    next_target_read_count = 1'b1;
                    next_led_view_index    = 1'b0;
                end
                `I2C_BNO055_STATE_SET_UNITS: begin // Page 0
                    next_imu_good      = `FALSE;
                    next_slave_address = `BNO055_SLAVE_ADDRESS;
                    next_go_flag       = `NOT_GO;
                    next_i2c_state     = `I2C_DRV_SUB_STATE_START;
                    next_return_state  = `I2C_BNO055_STATE_SET_POWER_MODE;
                    next_data_reg      = `BNO055_UNIT_SEL_ADDR;
                    // This line Modified from Adafruit Bosch BNO055 Arduino driver code, downloaded from: https://github.com/adafruit/Adafruit_BNO055
                    next_data_tx       = ((1 << 7) |  // Orientation = Windows - Range (Windows format) -180° to +180° corresponds with turning clockwise and increases values
                                         ( 0 << 4) |  // Temperature = Celsius
                                         ( 0 << 2) |  // Euler = Degrees
                                         ( 0 << 1) |  // Gyro = Degrees/Sec
                                         ( 0 << 0));  // Accelerometer = m/s^2;
                    next_read_write_in = `I2C_WRITE;
                end
                `I2C_BNO055_STATE_SET_POWER_MODE: begin // Page 0
                    next_imu_good      = `FALSE;
                    next_slave_address = `BNO055_SLAVE_ADDRESS;
                    clear_waiting_ms   = `RUN_MS_TIMER;
                    next_go_flag       = `NOT_GO;
                    next_i2c_state     = `I2C_DRV_SUB_STATE_START;
`ifdef BNO055_CAL_RESTORE_ENABLE
                    next_return_state  = `I2C_BNO055_STATE_CAL_RESTORE_DATA;
`else
                    next_return_state  = `I2C_BNO055_STATE_SET_EXT_CRYSTAL;
`endif
                    next_data_reg      = `BNO055_PWR_MODE_ADDR;
                    next_data_tx       = `BNO055_POWER_MODE_NORMAL;
                    next_read_write_in = `I2C_WRITE;
                end
                `I2C_BNO055_STATE_CAL_RESTORE_DATA: begin
                    next_imu_good      = `FALSE;
                    next_go_flag       = `NOT_GO;
                    next_slave_address = `BNO055_SLAVE_ADDRESS;
                    next_data_reg      = cal_reg_addr;
                    next_read_write_in = `I2C_WRITE;
                    /*
                        Calibration values from a good calibration of the particular BNO055 IMU that we are using:
                
                        Accelerometer: -27 -46 38
                        Mag: 284 30 -93
                        Gyro: -2 -3 0
                        Accel Radius: 1000
                        Mag Radius: 851
                        Accel X MSB:255, X LSB:229, Y MSB:255, Y LSB:210, Z MSB0,    Z LSB:38
                        Mag   X MSB:1,   X LSB:28,  Y MSB:0,   Y LSB:30,  Z MSB:255, Z LSB:163
                        Gyro  X MSB:255, X LSB:254, Y MSB:255, Y LSB:253, Z MSB:0,   Z LSB:0
                        Accel radius MSB:3, LSB:232
                        Mag radius   MSB:3, LSB:83
                       */
                    case(cal_restore_index)
                        `ACCEL_OFFSET_X_LSB_INDEX: next_data_tx = 8'd229;
                        `ACCEL_OFFSET_X_MSB_INDEX: next_data_tx = 8'd255;
                        `ACCEL_OFFSET_Y_LSB_INDEX: next_data_tx = 8'd210;
                        `ACCEL_OFFSET_Y_MSB_INDEX: next_data_tx = 8'd255;
                        `ACCEL_OFFSET_Z_LSB_INDEX: next_data_tx = 8'd38;
                        `ACCEL_OFFSET_Z_MSB_INDEX: next_data_tx = 8'd0;
                        `MAG_OFFSET_X_LSB_INDEX  : next_data_tx = 8'd28;
                        `MAG_OFFSET_X_MSB_INDEX  : next_data_tx = 8'd1;
                        `MAG_OFFSET_Y_LSB_INDEX  : next_data_tx = 8'd30;
                        `MAG_OFFSET_Y_MSB_INDEX  : next_data_tx = 8'd0;
                        `MAG_OFFSET_Z_LSB_INDEX  : next_data_tx = 8'd163;
                        `MAG_OFFSET_Z_MSB_INDEX  : next_data_tx = 8'd255;
                        `GYRO_OFFSET_X_LSB_INDEX : next_data_tx = 8'd254;
                        `GYRO_OFFSET_X_MSB_INDEX : next_data_tx = 8'd255;
                        `GYRO_OFFSET_Y_LSB_INDEX : next_data_tx = 8'd253;
                        `GYRO_OFFSET_Y_MSB_INDEX : next_data_tx = 8'd255;
                        `GYRO_OFFSET_Z_LSB_INDEX : next_data_tx = 8'd0;
                        `GYRO_OFFSET_Z_MSB_INDEX : next_data_tx = 8'd0;
                        `ACCEL_RADIUS_LSB_INDEX  : next_data_tx = 8'd3;
                        `ACCEL_RADIUS_MSB_INDEX  : next_data_tx = 8'd232;
                        `MAG_RADIUS_LSB_INDEX    : next_data_tx = 8'd3;
                        `MAG_RADIUS_MSB_INDEX    : next_data_tx = 8'd83;
                        default                  : next_data_tx = 8'd0;
                    endcase
                    next_i2c_state     = `I2C_BNO055_STATE_CAL_RESTORE_START;
                end
                `I2C_BNO055_STATE_CAL_RESTORE_START: begin
                    next_imu_good      = `FALSE;
                    next_go_flag       = `GO;
                    next_slave_address = `BNO055_SLAVE_ADDRESS;
                    next_data_reg      = data_reg;
                    next_data_tx       = data_tx;
                    next_read_write_in = read_write_in;
                    if(busy)
                        next_i2c_state = `I2C_BNO055_STATE_CAL_RESTORE_WAIT;
                    else
                        next_i2c_state = `I2C_BNO055_STATE_CAL_RESTORE_START;
                end
                `I2C_BNO055_STATE_CAL_RESTORE_WAIT: begin // Wait until send completes
                    next_imu_good      = `FALSE;
                    next_go_flag       = `NOT_GO;
                    next_slave_address = `BNO055_SLAVE_ADDRESS;
                    next_data_reg      = data_reg;
                    next_data_tx       = data_tx;
                    next_read_write_in = read_write_in;
                    if(~busy) begin
                        increment_cal_restore_index = 1'b1;
                        next_i2c_state = `I2C_BNO055_STATE_CAL_RESTORE_STOP;
                    end
                    else
                        next_i2c_state = `I2C_BNO055_STATE_CAL_RESTORE_WAIT;
                end
                `I2C_BNO055_STATE_CAL_RESTORE_STOP: begin // See if this was the last, loop around if more, otherwise, exit loop
                    next_imu_good      = `FALSE;
                    next_go_flag       = `NOT_GO;
                    next_slave_address = `BNO055_SLAVE_ADDRESS;
                    next_data_reg      = data_reg;
                    next_data_tx       = data_tx;
                    next_read_write_in = read_write_in;
                    if(cal_restore_index >= (`BNO055_CAL_DATA_REG_CNT)) begin
                        clear_cal_restore_index = 1'b0;
                        next_i2c_state = `I2C_BNO055_STATE_CAL_RESTORE_AGAIN;
                    end
                    else begin
                        next_i2c_state = `I2C_BNO055_STATE_CAL_RESTORE_DATA;
                    end
                end
                `I2C_BNO055_STATE_CAL_RESTORE_AGAIN: begin // Restore calibration two times, to ensure that one calibration parameter doesn't need to be written before another.
                    next_imu_good        = `FALSE;
                    next_go_flag         = `NOT_GO;
                    next_slave_address   = `BNO055_SLAVE_ADDRESS;
                    next_calibrated_once = 1'b1;
                    if(calibrated_once)
                        next_i2c_state = `I2C_BNO055_STATE_SET_EXT_CRYSTAL;
                    else
                        next_i2c_state = `I2C_BNO055_STATE_CAL_RESTORE_DATA;
                end
                `I2C_BNO055_STATE_SET_EXT_CRYSTAL: begin // Has to be done after caliubration restore, for some odd reason not documented in IMU docs
                    next_imu_good      = `FALSE;
                    next_slave_address = `BNO055_SLAVE_ADDRESS;
                    next_go_flag       = `NOT_GO;
                    next_i2c_state     = `I2C_DRV_SUB_STATE_START;
                    next_return_state  = `I2C_BNO055_STATE_SET_RUN_MODE;
                    next_data_reg      = `BNO055_SYS_TRIGGER_ADDR;
                    next_data_tx       = 8'd1<<7; // Enable external crystal, set bit 7 to 1'b1
                    next_read_write_in = `I2C_WRITE;
                end
                `I2C_BNO055_STATE_SET_RUN_MODE: begin // Change to run mode, changing run mode takes 7 to 19 ms depending on modes
                    next_imu_good      = `FALSE;
                    next_slave_address = `BNO055_SLAVE_ADDRESS;
                    clear_waiting_ms   = `CLEAR_MS_TIMER; // Clear and set to wait_ms value
                    count_ms_init_time = `FALSE;          // Make sure this is a POLL_INTERVAL timer
                    next_go_flag       = `NOT_GO;
                    next_return_state  = `I2C_BNO055_STATE_WAIT_20MS;
                    next_data_reg      = `BNO055_OPR_MODE_ADDR;
                    next_data_tx       = `BNO055_OPERATION_MODE_NDOF;
                    next_read_write_in = `I2C_WRITE;
                    // Wait for I2C to be not busy and timer to be set to polling time interval (20 ms)
                    if(~busy && delay_timer_at_poll)
                        next_i2c_state = `I2C_DRV_SUB_STATE_START;
                    else
                        next_i2c_state = `I2C_BNO055_STATE_SET_RUN_MODE;
                end
                `I2C_BNO055_STATE_WAIT_20MS: begin // Wait 20ms to go from config to running mode
                    next_imu_good      = `FALSE;
                    next_slave_address = `BNO055_SLAVE_ADDRESS;
                    clear_waiting_ms   = `RUN_MS_TIMER;
                    next_data_reg      = `ALL_ZERO_2BYTE;
                    next_data_tx       = `BYTE_ALL_ZERO;
                    next_go_flag       = `NOT_GO;
                    resetn_buffer      = `LOW; // Clear RX data buffer index before starting next state's read burst
                    // Wait for I2C to be not busy and delay timer done
                    if(~busy && delay_timer_done)
                        next_i2c_state = `I2C_BNO055_STATE_READ_IMU_DATA_BURST;
                    else
                        next_i2c_state = `I2C_BNO055_STATE_WAIT_20MS;
                end
                `I2C_BNO055_STATE_READ_IMU_DATA_BURST: begin // Page 0 - Read from Acceleration Data X-Axis LSB to Calibration Status registers - 46 bytes
                    clear_waiting_ms       = `CLEAR_MS_TIMER; // Clear and set to wait_ms value
                    count_ms_init_time     = `FALSE;          // Make sure this is a POLL_INTERVAL timer
                    next_slave_address     = `BNO055_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_DRV_STATE_WAIT_IMU_POLL_TIME;
                    next_data_reg          = `BNO055_ACCEL_DATA_X_LSB_ADDR;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_read_write_in     = `I2C_READ;
                    next_target_read_count = `BNO055_DATA_RX_BYTE_REG_CNT;
                    next_led_view_index    = (`BNO055_DATA_RX_BYTE_REG_CNT-1); // Calibration status will be in the last byte buffer, index 45
                    // Wait for I2C to be not busy and timer to be set to polling time interval (20 ms)
                    if(~busy && delay_timer_at_poll)
                        next_i2c_state     = `I2C_DRV_SUB_STATE_START;
                    else
                        next_i2c_state     = `I2C_BNO055_STATE_READ_IMU_DATA_BURST;
                end
                `I2C_DRV_STATE_WAIT_IMU_POLL_TIME: begin     // Wait 20 ms between polls to maintain 50Hz polling rate
                                                            // wait time is i2c time + time spent here, for a total of 20ms,
                                                            // i2c time is variable and dependent on slave
                                                            // This timer starts at the beginning of the the previous state
                    next_imu_good          = `TRUE;
                    next_slave_address     = `BNO055_SLAVE_ADDRESS;
                    clear_waiting_ms       = `RUN_MS_TIMER;
                    next_data_reg          = `ALL_ZERO_2BYTE;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_go_flag           = `NOT_GO;
                    resetn_buffer          = `LOW; // Clear the RX data buffer index starting next state's read burst
                    // Wait for I2C to be not busy and delay timer done
                    if(~busy && delay_timer_done)
                        next_i2c_state     = `I2C_BNO055_STATE_READ_IMU_DATA_BURST;
                    else
                        next_i2c_state     = `I2C_DRV_STATE_WAIT_IMU_POLL_TIME;
                end

                // FSM Sub States - Repeated for each i2c transaction
                `I2C_DRV_SUB_STATE_START: begin // Begin i2c transaction, wait for busy to be asserted
                    next_go_flag           = `GO;
                    if(busy && resetn_imu) // Stay here until i2c is busy AND the IMU isn't in reset (Prevent glitch at WD event)
                        next_i2c_state     = `I2C_DRV_SUB_STATE_WAIT_I2C;
                    else
                        next_i2c_state     = `I2C_DRV_SUB_STATE_START;
                end
                `I2C_DRV_SUB_STATE_WAIT_I2C: begin // Wait for end of i2c transaction, wait for busy to be cleared
                    next_go_flag           = `NOT_GO;
                    if(~busy && resetn_imu) // Stay here until i2c is not busy AND the IMU isn't in reset (Prevent glitch at WD event)
                        next_i2c_state     = `I2C_DRV_SUB_STATE_STOP;
                    else
                        next_i2c_state     = `I2C_DRV_SUB_STATE_WAIT_I2C;
                end // Set output data latch strobe and return to major FSM state
                `I2C_DRV_SUB_STATE_STOP: begin
                    next_go_flag           = `NOT_GO;
                    next_data_reg          = `ALL_ZERO_2BYTE;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    if(read_write_in == `I2C_READ) // Only latch data if this was a read
                        rx_data_latch_strobe = `HIGH;
                    next_i2c_state         = return_state;
                end

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
                // Default case, shouldn't be triggered
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
                default: begin
                    next_imu_good          = `FALSE;
                    clear_waiting_ms       = `RUN_MS_TIMER;
                    count_ms_init_time     = `FALSE;
                    next_i2c_state         = `I2C_DRV_STATE_RESET;
                    next_return_state      = `I2C_DRV_STATE_RESET;
                    next_go_flag           = `NOT_GO;
                    next_data_reg          = `ALL_ZERO_2BYTE;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_read_write_in     = `I2C_READ;
                    next_led_view_index    = `FALSE;
                    next_rx_from_vl53l1x   = `FALSE;
                    next_is_2_byte_reg     = `FALSE;
                    resetn_buffer          = `LOW;
                    next_target_read_count = 1'b1;
                    rx_data_latch_strobe   = `LOW;
                    i2c_number             = 1'b0; // Default to i2c EFB #1
                    next_slave_address     = `BNO055_SLAVE_ADDRESS;
                    increment_cal_restore_index = 1'b0;
                    clear_cal_restore_index     = 1'b0;
                end
            endcase
        end
    end
endmodule
