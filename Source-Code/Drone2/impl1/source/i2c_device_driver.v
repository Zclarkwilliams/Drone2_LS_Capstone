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

 */

`timescale 1ns / 1ns
`include "common_defines.v"
`include "bno055_defines.v"
`include "VL53L1X_defines.v"
`include "i2c_driver_defines.v"

module i2c_device_driver #(
    parameter INIT_INTERVAL = 16'd10_000, // 10 seconds (in ms) start-up time
    //parameter POLL_INTERVAL = 16'd20      // 20 ms between data polls
    parameter POLL_INTERVAL = 16'd2      // 2 ms between I2C cycles, for debugging
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
    output reg [15:0]VL53L1X_chip_id,
    output reg [15:0]VL53L1X_range_mm,
    output reg [7:0]VL53L1X_firm_rdy,                 //  Store VL53L1X firmware ready byte
    output reg [7:0]VL53L1X_data_rdy                  //  Store VL53L1X data ready byte
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
    reg  [5:0]VL53L1X_data_rx_reg_index;              //  Index in VL53L1X_data_rx_reg for current byte from VL53L1X
    reg  [5:0]next_VL53L1X_data_rx_reg_index;         //  Manually set index in VL53L1X_data_rx_reg for next byte from VL53L1X
    reg  set_VL53L1X_data_rx_reg_index;               //  Flag to indicate manual set of next VL53L1X_data_rx_reg_index 
    reg  [5:0]led_view_index;                         //  Index in BNO055_data_rx_reg that is being monitored with status LEDs
    reg  [5:0]next_led_view_index;                    //  Next value of LED View Index
    reg  [7:0]VL53L1X_data_rx_reg[`VL53L1X_DATA_RX_BYTE_REG_CNT-1:0]; //  Store all received data bytes from VL53L1X
    reg  [15:0]VL53L1X_osc_cal_val;                   //  Store VL53L1X oscillator calibration value bytes
    reg  [31:0]VL53L1X_measurement_period;            //  Calculated measurement period for the VL53L1X lidar sensor
    reg  [31:0]next_VL53L1X_measurement_period;       //  Next value of calculated measurement period for the VL53L1X lidar sensor
    reg  [2:0]measurement_period_tx_index;            //  Pointer to byte of VL53L1X_measurement_period to transmit
    reg  [2:0]next_measurement_period_tx_index;       //  Next value of pointer to byte of VL53L1X_measurement_period to transmit
    reg  resetn_VL53L1X_buffer;                       //  Negedge clears received measurement buffer
    reg  next_resetn_VL53L1X_buffer;                  //  Negedge clears received measurement buffer
    reg  rx_data_latch_strobe;                        //  Strobe data output register, latch onto current data in rx buffer, asynchronous latch
    reg  next_imu_good;                               //  Next value of module imu_good bit
    reg  i2c_number;                                  //  The i2c module to call, 0 = i2c EFB #1, 1 = i2c EFB #2
    reg  rx_from_VL53L1X;                             //  Receiving from VL53L1X = 1, from BNO055 = 0
    reg  next_rx_from_VL53L1X;                        //  Receiving from VL53L1X = 1, from BNO055 = 0
    reg [7:0]cal_reg_addr;                            //  Current IMU register address that this calibration data is destined for
    reg [7:0]next_cal_reg_addr;                       //  Current IMU register address that this calibration data is destined for
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
    //assign led_data_out = (i2c_state    <= `I2C_DRV_STATE_BOOT_WAIT ) ? 8'h41 : BNO055_data_rx_reg[led_view_index]; //  Output for calibration status LEDs OR indicates that the IMU is in reset
    assign led_data_out = return_state;
    //assign led_data_out = VL53L1X_data_rx_reg_index;




    // resetn_lidar follows resetn_imu - The ST Microelectronics VL53L1X has a low active reset 0 = shutdown, 1 = run, same as Bosch BNO055
    assign resetn_lidar = resetn_imu;                   //Regular LiDAR reset operation
    //assign resetn_lidar = 1'd1 /*syn_force_pads = 1*/;  // Force LiDAR sensor to always active, ignore resetn input
    //assign resetn_lidar = 1'd0 /*syn_force_pads = 1*/;  // Force LiDAR sensor to always inactive, ignore resetn input


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

    always@(posedge sys_clk, posedge set_VL53L1X_data_rx_reg_index, negedge resetn_VL53L1X_buffer, negedge resetn) begin
        if(~resetn)
            VL53L1X_data_rx_reg_index <= 'd0;
        else if(~resetn_VL53L1X_buffer )
            VL53L1X_data_rx_reg_index <= 'd0;
        else if (set_VL53L1X_data_rx_reg_index)
            VL53L1X_data_rx_reg_index <= next_VL53L1X_data_rx_reg_index;
        else if (one_byte_ready) // A byte has been read by I2C
            VL53L1X_data_rx_reg_index <= VL53L1X_data_rx_reg_index + 6'd1;
    end


    //  During a read cycle increment the VL53L1X_data_rx_reg_index until it reaches the end of  VL53L1X_data_rx_reg
    //  If a byte has been read (one_byte_ready is asserted) assign it to the VL53L1X_data_rx_reg byte array at the location specified by VL53L1X_data_rx_reg_index
    always@(posedge sys_clk, negedge resetn_VL53L1X_buffer, negedge resetn) begin
        if(~resetn) begin
            VL53L1X_data_rx_reg[0] <= 'd0;
            VL53L1X_data_rx_reg[1] <= 'd0;
            VL53L1X_data_rx_reg[2] <= 'd0;
            VL53L1X_data_rx_reg[3] <= 'd0;
            VL53L1X_data_rx_reg[4] <= 'd0;
            VL53L1X_data_rx_reg[5] <= 'd0;
            VL53L1X_data_rx_reg[6] <= 'd0;
            VL53L1X_data_rx_reg[7] <= 'd0;
        end
        else if(~resetn_VL53L1X_buffer ) begin
            VL53L1X_data_rx_reg[`VL53L1X_DATA_RX_REG_FIRMWARE_SYSTEM_STATUS_INDEX] <= 'd0;
            VL53L1X_data_rx_reg[`VL53L1X_DATA_RX_REG_DATA_RDY_INDEX]               <= 'd0;
        end
        else if (one_byte_ready && rx_from_VL53L1X) begin  // A byte has been read by I2C and byte is from the VL53L1X
            VL53L1X_data_rx_reg[VL53L1X_data_rx_reg_index] <= data_rx;
        end
    end


    //  Generates a 20ms countdown timer that enables module output valid strobe when it counts beyond 0
    //  When timer wraps around the enable signal is set for clock tick, or delayed for 1 additional tick
    //
    //  If rx_data_latch_strobe is not asserted then the enable signal is asserted for one click tick
    //  Otherwise it will not be asserted now and will be asserted at the next clock tick
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
    //  This block is for VL53L1X registers
    always@(posedge sys_clk, negedge resetn) begin
        if(~resetn) begin
            VL53L1X_chip_id     <= 16'd0;
            VL53L1X_range_mm    <= 16'd0;
            VL53L1X_data_rdy    <= 8'd0;
            VL53L1X_firm_rdy    <= 8'd0;
            VL53L1X_osc_cal_val <= 16'd0;
        end
        else if(rx_data_latch_strobe) begin
            VL53L1X_chip_id     <= {VL53L1X_data_rx_reg[`VL53L1X_DATA_RX_REG_CHIP_ID_HI_INDEX],                VL53L1X_data_rx_reg[`VL53L1X_DATA_RX_REG_CHIP_ID_LO_INDEX]};
            VL53L1X_range_mm    <= {VL53L1X_data_rx_reg[`VL53L1X_DATA_RX_REG_RESULT_RANGE_MEASURE_MM_HI_INDEX],VL53L1X_data_rx_reg[`VL53L1X_DATA_RX_REG_RESULT_RANGE_MEASURE_MM_LO_INDEX]};
            VL53L1X_data_rdy    <= VL53L1X_data_rx_reg[`VL53L1X_DATA_RX_REG_DATA_RDY_INDEX];
            VL53L1X_firm_rdy    <= VL53L1X_data_rx_reg[`VL53L1X_DATA_RX_REG_FIRMWARE_SYSTEM_STATUS_INDEX];
            VL53L1X_osc_cal_val <= {VL53L1X_data_rx_reg[`VL53L1X_DATA_RX_REG_RESULT_OSC_CAL_VAL_HI_INDEX],VL53L1X_data_rx_reg[`VL53L1X_DATA_RX_REG_RESULT_OSC_CAL_VAL_LO_INDEX]};
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
            slave_address       <= `VL53L1X_SLAVE_ADDRESS;
            imu_good            <= `FALSE;
            calibrated_once     <= `FALSE;
            rx_from_VL53L1X     <= `FALSE;
            is_2_byte_reg       <= `FALSE;
            VL53L1X_measurement_period  <= 32'd0;
            measurement_period_tx_index <= 3'd4;
            resetn_VL53L1X_buffer       <= `TRUE;
            cal_reg_addr                <= `VL53L1X_PAD_I2C_HV_CONFIG_ADDR;
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
            rx_from_VL53L1X     <= next_rx_from_VL53L1X;
            is_2_byte_reg       <= next_is_2_byte_reg;
            VL53L1X_measurement_period  <= next_VL53L1X_measurement_period;
            measurement_period_tx_index <= next_measurement_period_tx_index;
            resetn_VL53L1X_buffer       <= next_resetn_VL53L1X_buffer;
            cal_reg_addr                <= next_cal_reg_addr;
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
            next_rx_from_VL53L1X      = `FALSE;
            next_is_2_byte_reg        = `FALSE;
            next_resetn_VL53L1X_buffer = `LOW;
            next_target_read_count    = 'd1;
            rx_data_latch_strobe      = `LOW;
            i2c_number                = 1'b0; // Default to i2c EFB #1
            next_slave_address        = `VL53L1X_SLAVE_ADDRESS;
            set_VL53L1X_data_rx_reg_index    = `FALSE;
            next_VL53L1X_data_rx_reg_index   = 8'd0;
            next_VL53L1X_measurement_period  = 32'd0;
            next_measurement_period_tx_index = 3'd3;
            next_cal_reg_addr                = `VL53L1X_PAD_I2C_HV_CONFIG_ADDR;
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
            next_rx_from_VL53L1X      = rx_from_VL53L1X;
            next_is_2_byte_reg        = is_2_byte_reg;
            next_resetn_VL53L1X_buffer = `HIGH;
            next_target_read_count    = target_read_count;
            rx_data_latch_strobe      = `LOW;
            i2c_number                = 1'b0; // Default to i2c EFB #1
            next_slave_address        = `VL53L1X_SLAVE_ADDRESS;
            next_calibrated_once      = calibrated_once;
            set_VL53L1X_data_rx_reg_index    = `FALSE;
            next_VL53L1X_data_rx_reg_index   = VL53L1X_data_rx_reg_index;
            next_VL53L1X_measurement_period  = VL53L1X_measurement_period;
            next_measurement_period_tx_index = measurement_period_tx_index;
            next_cal_reg_addr                = cal_reg_addr;
            case(i2c_state)
                `I2C_DRV_STATE_RESET: begin
                    next_imu_good      = `FALSE;
                    clear_waiting_ms   = `RUN_MS_TIMER;
                    count_ms_init_time = `TRUE;
                    if(~delay_timer_started) //Timer not yet set to starting value
                        next_i2c_state      = `I2C_DRV_STATE_RESET;
                    else
                        next_i2c_state        = `I2C_DRV_STATE_BOOT;
                    next_slave_address        = `VL53L1X_SLAVE_ADDRESS;
                    next_calibrated_once      = 1'b0;
                    next_rx_from_VL53L1X      = `FALSE;
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
                    next_slave_address = `VL53L1X_SLAVE_ADDRESS;
                    next_resetn_VL53L1X_buffer = `LOW; // Clear VL53L1X RX data buffer index and data ready
                end
                `I2C_DRV_STATE_BOOT_WAIT: begin
                    next_imu_good      = `FALSE;
                    clear_waiting_ms   = `RUN_MS_TIMER;
                    count_ms_init_time = `FALSE;
                    next_i2c_state     = `I2C_DRV_STATE_BOOT_WAIT;
                    next_slave_address = `VL53L1X_SLAVE_ADDRESS;
                    // Wait for I2C to be not busy and delay timer done
                    if(~busy && delay_timer_done)
                        next_i2c_state     = `I2C_VL53L1X_STATE_READ_CHIP_ID;
                    else
                        next_i2c_state     = `I2C_DRV_STATE_BOOT_WAIT;
                end
                `I2C_VL53L1X_STATE_READ_CHIP_ID: begin
                    next_imu_good          = `FALSE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_VL53L1X_STATE_READ_FIRMWARE_READY;
                    next_data_reg          = `VL53L1X_IDENTIFICATION_MODEL_ID_ADDR;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_read_write_in     = `I2C_READ;
                    next_rx_from_VL53L1X   = `TRUE;
                    next_is_2_byte_reg     = `TRUE;
                    next_target_read_count = 5'd2;
                    next_led_view_index    = 1'b0;
                end
                `I2C_VL53L1X_STATE_READ_FIRMWARE_READY: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    if(VL53L1X_firm_rdy[0] == 1'b1) // If firmware ready, go to next state
                        next_i2c_state     = `I2C_STATE_CAL_RESTORE_DATA;
                    else
                        next_i2c_state     = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_VL53L1X_STATE_READ_FIRMWARE_READY;
                    next_data_reg          = `VL53L1X_FIRMWARE_SYSTEM_STATUS_ADDR;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_read_write_in     = `I2C_READ;
                    next_rx_from_VL53L1X   = `TRUE;
                    next_is_2_byte_reg     = `TRUE;
                    next_target_read_count = 5'd1;
                    set_VL53L1X_data_rx_reg_index  = `TRUE;
                    next_VL53L1X_data_rx_reg_index = `VL53L1X_DATA_RX_REG_FIRMWARE_SYSTEM_STATUS_INDEX;
                    next_cal_reg_addr              = `VL53L1X_PAD_I2C_HV_CONFIG_ADDR;
                end
                `I2C_STATE_CAL_RESTORE_DATA: begin
                    next_imu_good      = `FALSE;
                    next_slave_address = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag       = `NOT_GO;
                    next_i2c_state     = `I2C_DRV_SUB_STATE_START;
                    next_return_state  = `I2C_STATE_CAL_RESTORE_NEXT;
                    next_data_reg      = cal_reg_addr;
                    next_read_write_in = `I2C_WRITE;
                    case(cal_reg_addr)
                        `VL53L1X_PAD_I2C_HV_CONFIG_ADDR                             : next_data_tx = `VL53L1X_INIT_VAL_PAD_I2C_HV_CONFIG;
                        `VL53L1X_PAD_I2C_HV_EXTSUP_CONFIG_ADDR                      : next_data_tx = `VL53L1X_INIT_VAL_PAD_I2C_HV_EXTSUP_CONFIG;
                        `VL53L1X_GPIO_HV_PAD_CTRL_ADDR                              : next_data_tx = `VL53L1X_INIT_VAL_GPIO_HV_PAD_CTRL;
                        `VL53L1X_GPIO_HV_MUX_CTRL_ADDR                              : next_data_tx = `VL53L1X_INIT_VAL_GPIO_HV_MUX_CTRL;
                        `VL53L1X_GPIO_TIO_HV_STATUS_ADDR                            : next_data_tx = `VL53L1X_INIT_VAL_GPIO_TIO_HV_STATUS;
                        `VL53L1X_GPIO_FIO_HV_STATUS_ADDR                            : next_data_tx = `VL53L1X_INIT_VAL_GPIO_FIO_HV_STATUS;
                        `VL53L1X_ANA_CONFIG_SPAD_SEL_PSWIDTH_ADDR                   : next_data_tx = `VL53L1X_INIT_VAL_ANA_CONFIG_SPAD_SEL_PSWIDTH;
                        `VL53L1X_ANA_CONFIG_VCSEL_PULSE_WIDTH_OFFSET_ADDR           : next_data_tx = `VL53L1X_INIT_VAL_ANA_CONFIG_VCSEL_PULSE_WIDTH_OFFSET;
                        `VL53L1X_ANA_CONFIG_FAST_OSC_CONFIG_CTRL_ADDR               : next_data_tx = `VL53L1X_INIT_VAL_ANA_CONFIG_FAST_OSC_CONFIG_CTRL;
                        `VL53L1X_SIGMA_ESTIMATOR_EFFECTIVE_PULSE_WIDTH_NS_ADDR      : next_data_tx = `VL53L1X_INIT_VAL_SIGMA_ESTIMATOR_EFFECTIVE_PULSE_WIDTH_NS;
                        `VL53L1X_SIGMA_ESTIMATOR_EFFECTIVE_AMBIENT_WIDTH_NS_ADDR    : next_data_tx = `VL53L1X_INIT_VAL_SIGMA_ESTIMATOR_EFFECTIVE_AMBIENT_WIDTH_NS;
                        `VL53L1X_SIGMA_ESTIMATOR_SIGMA_REF_MM_ADDR                  : next_data_tx = `VL53L1X_INIT_VAL_SIGMA_ESTIMATOR_SIGMA_REF_MM;
                        `VL53L1X_ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT_MM_ADDR   : next_data_tx = `VL53L1X_INIT_VAL_ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT_MM;
                        `VL53L1X_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_0_ADDR       : next_data_tx = `VL53L1X_INIT_VAL_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_0;
                        `VL53L1X_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_1_ADDR       : next_data_tx = `VL53L1X_INIT_VAL_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_1;
                        `VL53L1X_ALGO_RANGE_IGNORE_THRESHOLD_MCPS_HI_ADDR           : next_data_tx = `VL53L1X_INIT_VAL_ALGO_RANGE_IGNORE_THRESHOLD_MCPS_HI;
                        `VL53L1X_ALGO_RANGE_IGNORE_THRESHOLD_MCPS_LO_ADDR           : next_data_tx = `VL53L1X_INIT_VAL_ALGO_RANGE_IGNORE_THRESHOLD_MCPS_LO;
                        `VL53L1X_ALGO_RANGE_IGNORE_VALID_HEIGHT_MM_ADDR             : next_data_tx = `VL53L1X_INIT_VAL_ALGO_RANGE_IGNORE_VALID_HEIGHT_MM;
                        `VL53L1X_ALGO_RANGE_MIN_CLIP_ADDR                           : next_data_tx = `VL53L1X_INIT_VAL_ALGO_RANGE_MIN_CLIP;
                        `VL53L1X_ALGO_CONSISTENCY_CHECK_TOLERANCE_ADDR              : next_data_tx = `VL53L1X_INIT_VAL_ALGO_CONSISTENCY_CHECK_TOLERANCE;
                        `VL53L1X_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_2_ADDR       : next_data_tx = `VL53L1X_INIT_VAL_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_2;
                        `VL53L1X_SD_CONFIG_RESET_STAGES_MSB_ADDR                    : next_data_tx = `VL53L1X_INIT_VAL_SD_CONFIG_RESET_STAGES_MSB;
                        `VL53L1X_SD_CONFIG_RESET_STAGES_LSB_ADDR                    : next_data_tx = `VL53L1X_INIT_VAL_SD_CONFIG_RESET_STAGES_LSB;
                        `VL53L1X_GPH_CONFIG_STREAM_COUNT_UPDATE_VALUE_ADDR          : next_data_tx = `VL53L1X_INIT_VAL_GPH_CONFIG_STREAM_COUNT_UPDATE_VALUE;
                        `VL53L1X_GLOBAL_CONFIG_STREAM_DIVIDER_ADDR                  : next_data_tx = `VL53L1X_INIT_VAL_GLOBAL_CONFIG_STREAM_DIVIDER;
                        `VL53L1X_SYSTEM_INTERRUPT_CONFIG_GPIO_ADDR                  : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_INTERRUPT_CONFIG_GPIO;
                        `VL53L1X_CAL_CONFIG_VCSEL_START_ADDR                        : next_data_tx = `VL53L1X_INIT_VAL_CAL_CONFIG_VCSEL_START;
                        `VL53L1X_CAL_CONFIG_REPEAT_RATE_HI_ADDR                     : next_data_tx = `VL53L1X_INIT_VAL_CAL_CONFIG_REPEAT_RATE_HI;
                        `VL53L1X_CAL_CONFIG_REPEAT_RATE_LO_ADDR                     : next_data_tx = `VL53L1X_INIT_VAL_CAL_CONFIG_REPEAT_RATE_LO;
                        `VL53L1X_GLOBAL_CONFIG_VCSEL_WIDTH_ADDR                     : next_data_tx = `VL53L1X_INIT_VAL_GLOBAL_CONFIG_VCSEL_WIDTH;
                        `VL53L1X_PHASECAL_CONFIG_TIMEOUT_MACROP_ADDR                : next_data_tx = `VL53L1X_INIT_VAL_PHASECAL_CONFIG_TIMEOUT_MACROP;
                        `VL53L1X_PHASECAL_CONFIG_TARGET_ADDR                        : next_data_tx = `VL53L1X_INIT_VAL_PHASECAL_CONFIG_TARGET;
                        `VL53L1X_PHASECAL_CONFIG_OVERRIDE_ADDR                      : next_data_tx = `VL53L1X_INIT_VAL_PHASECAL_CONFIG_OVERRIDE;
                        `VL53L1X_UNNAMED_REG_0x004E_ADDR                            : next_data_tx = `VL53L1X_INIT_VAL_UNNAMED_REG_0x004E;
                        `VL53L1X_DSS_CONFIG_ROI_MODE_CONTROL_ADDR                   : next_data_tx = `VL53L1X_INIT_VAL_DSS_CONFIG_ROI_MODE_CONTROL;
                        `VL53L1X_SYSTEM_THRESH_RATE_HIGH_HI_ADDR                    : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_THRESH_RATE_HIGH_HI;
                        `VL53L1X_SYSTEM_THRESH_RATE_HIGH_LO_ADDR                    : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_THRESH_RATE_HIGH_LO;
                        `VL53L1X_SYSTEM_THRESH_RATE_LOW_HI_ADDR                     : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_THRESH_RATE_LOW_HI;
                        `VL53L1X_SYSTEM_THRESH_RATE_LOW_LO_ADDR                     : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_THRESH_RATE_LOW_LO;
                        `VL53L1X_DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT_HI_ADDR   : next_data_tx = `VL53L1X_INIT_VAL_DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT_HI;
                        `VL53L1X_DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT_LO_ADDR   : next_data_tx = `VL53L1X_INIT_VAL_DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT_LO;
                        `VL53L1X_DSS_CONFIG_MANUAL_BLOCK_SELECT_ADDR                : next_data_tx = `VL53L1X_INIT_VAL_DSS_CONFIG_MANUAL_BLOCK_SELECT;
                        `VL53L1X_DSS_CONFIG_APERTURE_ATTENUATION_ADDR               : next_data_tx = `VL53L1X_INIT_VAL_DSS_CONFIG_APERTURE_ATTENUATION;
                        `VL53L1X_DSS_CONFIG_MAX_SPADS_LIMIT_ADDR                    : next_data_tx = `VL53L1X_INIT_VAL_DSS_CONFIG_MAX_SPADS_LIMIT;
                        `VL53L1X_DSS_CONFIG_MIN_SPADS_LIMIT_ADDR                    : next_data_tx = `VL53L1X_INIT_VAL_DSS_CONFIG_MIN_SPADS_LIMIT;
                        `VL53L1X_MM_CONFIG_TIMEOUT_MACROP_A_HI_ADDR                 : next_data_tx = `VL53L1X_INIT_VAL_MM_CONFIG_TIMEOUT_MACROP_A_HI;
                        `VL53L1X_MM_CONFIG_TIMEOUT_MACROP_A_LO_ADDR                 : next_data_tx = `VL53L1X_INIT_VAL_MM_CONFIG_TIMEOUT_MACROP_A_LO;
                        `VL53L1X_MM_CONFIG_TIMEOUT_MACROP_B_HI_ADDR                 : next_data_tx = `VL53L1X_INIT_VAL_MM_CONFIG_TIMEOUT_MACROP_B_HI;
                        `VL53L1X_MM_CONFIG_TIMEOUT_MACROP_B_LO_ADDR                 : next_data_tx = `VL53L1X_INIT_VAL_MM_CONFIG_TIMEOUT_MACROP_B_LO;
                        `VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_A_HI_ADDR              : next_data_tx = `VL53L1X_INIT_VAL_RANGE_CONFIG_TIMEOUT_MACROP_A_HI;
                        `VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_A_LO_ADDR              : next_data_tx = `VL53L1X_INIT_VAL_RANGE_CONFIG_TIMEOUT_MACROP_A_LO;
                        `VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_A_ADDR                   : next_data_tx = `VL53L1X_INIT_VAL_RANGE_CONFIG_VCSEL_PERIOD_A;
                        `VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_B_HI_ADDR              : next_data_tx = `VL53L1X_INIT_VAL_RANGE_CONFIG_TIMEOUT_MACROP_B_HI;
                        `VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_B_LO_ADDR              : next_data_tx = `VL53L1X_INIT_VAL_RANGE_CONFIG_TIMEOUT_MACROP_B_LO;
                        `VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_B_ADDR                   : next_data_tx = `VL53L1X_INIT_VAL_RANGE_CONFIG_VCSEL_PERIOD_B;
                        `VL53L1X_RANGE_CONFIG_SIGMA_THRESH_HI_ADDR                  : next_data_tx = `VL53L1X_INIT_VAL_RANGE_CONFIG_SIGMA_THRESH_HI;
                        `VL53L1X_RANGE_CONFIG_SIGMA_THRESH_LO_ADDR                  : next_data_tx = `VL53L1X_INIT_VAL_RANGE_CONFIG_SIGMA_THRESH_LO;
                        `VL53L1X_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI_ADDR : next_data_tx = `VL53L1X_INIT_VAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI;
                        `VL53L1X_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO_ADDR : next_data_tx = `VL53L1X_INIT_VAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO;
                        `VL53L1X_RANGE_CONFIG_VALID_PHASE_LOW_ADDR                  : next_data_tx = `VL53L1X_INIT_VAL_RANGE_CONFIG_VALID_PHASE_LOW;
                        `VL53L1X_RANGE_CONFIG_VALID_PHASE_HIGH_ADDR                 : next_data_tx = `VL53L1X_INIT_VAL_RANGE_CONFIG_VALID_PHASE_HIGH;
                        `VL53L1X_UNNAMED_REG_0x006A_ADDR                            : next_data_tx = `VL53L1X_INIT_VAL_UNNAMED_REG_0x006A;
                        `VL53L1X_UNNAMED_REG_0x006B_ADDR                            : next_data_tx = `VL53L1X_INIT_VAL_UNNAMED_REG_0x006B;
                        `VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD_3_ADDR              : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_INTERMEASUREMENT_PERIOD_3;
                        `VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD_2_ADDR              : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_INTERMEASUREMENT_PERIOD_2;
                        `VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD_1_ADDR              : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_INTERMEASUREMENT_PERIOD_1;
                        `VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD_0_ADDR              : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_INTERMEASUREMENT_PERIOD_0;
                        `VL53L1X_SYSTEM_FRACTIONAL_ENABLE_ADDR                      : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_FRACTIONAL_ENABLE;
                        `VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD_0_ADDR               : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_GROUPED_PARAMETER_HOLD_0;
                        `VL53L1X_SYSTEM_THRESH_HIGH_HI_ADDR                         : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_THRESH_HIGH_HI;
                        `VL53L1X_SYSTEM_THRESH_HIGH_LO_ADDR                         : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_THRESH_HIGH_LO;
                        `VL53L1X_SYSTEM_THRESH_LOW_HI_ADDR                          : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_THRESH_LOW_HI;
                        `VL53L1X_SYSTEM_THRESH_LOW_LO_ADDR                          : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_THRESH_LOW_LO;
                        `VL53L1X_SYSTEM_ENABLE_XTALK_PER_QUADRANT_ADDR              : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_ENABLE_XTALK_PER_QUADRANT;
                        `VL53L1X_SYSTEM_SEED_CONFIG_ADDR                            : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_SEED_CONFIG;
                        `VL53L1X_SD_CONFIG_WOI_SD0_ADDR                             : next_data_tx = `VL53L1X_INIT_VAL_SD_CONFIG_WOI_SD0;
                        `VL53L1X_SD_CONFIG_WOI_SD1_ADDR                             : next_data_tx = `VL53L1X_INIT_VAL_SD_CONFIG_WOI_SD1;
                        `VL53L1X_SD_CONFIG_INITIAL_PHASE_SD0_ADDR                   : next_data_tx = `VL53L1X_INIT_VAL_SD_CONFIG_INITIAL_PHASE_SD0;
                        `VL53L1X_SD_CONFIG_INITIAL_PHASE_SD1_ADDR                   : next_data_tx = `VL53L1X_INIT_VAL_SD_CONFIG_INITIAL_PHASE_SD1;
                        `VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD_1_ADDR               : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_GROUPED_PARAMETER_HOLD_1;
                        `VL53L1X_SD_CONFIG_FIRST_ORDER_SELECT_ADDR                  : next_data_tx = `VL53L1X_INIT_VAL_SD_CONFIG_FIRST_ORDER_SELECT;
                        `VL53L1X_SD_CONFIG_QUANTIFIER_ADDR                          : next_data_tx = `VL53L1X_INIT_VAL_SD_CONFIG_QUANTIFIER;
                        `VL53L1X_ROI_CONFIG_USER_ROI_CENTRE_SPAD_ADDR               : next_data_tx = `VL53L1X_INIT_VAL_ROI_CONFIG_USER_ROI_CENTRE_SPAD;
                        `VL53L1X_ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE_ADDR  : next_data_tx = `VL53L1X_INIT_VAL_ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE;
                        `VL53L1X_SYSTEM_SEQUENCE_CONFIG_ADDR                        : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_SEQUENCE_CONFIG;
                        `VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD_ADDR                 : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_GROUPED_PARAMETER_HOLD;
                        `VL53L1X_POWER_MANAGEMENT_GO1_POWER_FORCE_ADDR              : next_data_tx = `VL53L1X_INIT_VAL_POWER_MANAGEMENT_GO1_POWER_FORCE;
                        `VL53L1X_SYSTEM_STREAM_COUNT_CTRL_ADDR                      : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_STREAM_COUNT_CTRL;
                        `VL53L1X_FIRMWARE_ENABLE_ADDR                               : next_data_tx = `VL53L1X_INIT_VAL_FIRMWARE_ENABLE;
                        `VL53L1X_SYSTEM_INTERRUPT_CLEAR_ADDR                        : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_INTERRUPT_CLEAR;
                        `VL53L1X_SYSTEM_MODE_START_ADDR                             : next_data_tx = `VL53L1X_INIT_VAL_SYSTEM_MODE_START;
                        default                                                     : next_data_tx = 8'd0;
                    endcase
                end
                `I2C_STATE_CAL_RESTORE_NEXT: begin // See if this was the last, loop around if more, otherwise, exit loop
                    next_imu_good      = `FALSE;
                    next_go_flag       = `NOT_GO;
                    next_slave_address = slave_address;
                    next_data_reg      = data_reg;
                    next_data_tx       = data_tx;
                    next_read_write_in = read_write_in;
                    next_cal_reg_addr  = cal_reg_addr      + 8'd1;
                    if(cal_reg_addr >= (`VL53L1X_SYSTEM_MODE_START_ADDR)) begin
                        next_i2c_state = `I2C_VL53L1X_STATE_INIT_START_MEASURE;
                    end
                    else begin
                        next_i2c_state = `I2C_STATE_CAL_RESTORE_DATA;
                    end
                end
                `I2C_VL53L1X_STATE_INIT_START_MEASURE: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_VL53L1X_STATE_INIT_POLL_DATA_READY;
                    next_data_reg          = `VL53L1X_SYSTEM_MODE_START_ADDR;
                    next_data_tx           = 8'h40;
                    next_read_write_in     = `I2C_WRITE;
                    next_is_2_byte_reg     = `TRUE;
                    next_resetn_VL53L1X_buffer  = `LOW; // Clear VL53L1X RX data buffer index and data ready
                end
                `I2C_VL53L1X_STATE_INIT_POLL_DATA_READY: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    if(VL53L1X_data_rdy[0] == 1'b1) // If measurement ready, go to next state
                        next_i2c_state     = `I2C_VL53L1X_STATE_INIT_CLEAR_INTERRUPT;
                    else
                        next_i2c_state     = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_VL53L1X_STATE_INIT_POLL_DATA_READY;
                    next_data_reg          = `VL53L1X_GPIO_TIO_HV_STATUS_ADDR;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_read_write_in     = `I2C_READ;
                    next_rx_from_VL53L1X   = `TRUE;
                    next_is_2_byte_reg     = `TRUE;
                    next_target_read_count = 5'd1;
                    set_VL53L1X_data_rx_reg_index  = `TRUE;
                    next_VL53L1X_data_rx_reg_index = `VL53L1X_DATA_RX_REG_DATA_RDY_INDEX;
                end
                `I2C_VL53L1X_STATE_INIT_CLEAR_INTERRUPT: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_VL53L1X_STATE_INIT_STOP_MEASURE;
                    next_data_reg          = `VL53L1X_SYSTEM_INTERRUPT_CLEAR_ADDR;
                    next_data_tx           = 8'h01;
                    next_read_write_in     = `I2C_WRITE;
                    next_is_2_byte_reg     = `TRUE;
                end
                `I2C_VL53L1X_STATE_INIT_STOP_MEASURE: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_VL53L1X_STATE_SET_TEMPERATURE_MACRO_LOOP_BOUND;
                    next_data_reg          = `VL53L1X_SYSTEM_MODE_START_ADDR;
                    next_data_tx           = 8'h00;
                    next_read_write_in     = `I2C_WRITE;
                    next_is_2_byte_reg     = `TRUE;
                end
                `I2C_VL53L1X_STATE_SET_TEMPERATURE_MACRO_LOOP_BOUND: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_VL53L1X_STATE_SET_TEMPERATURE_VHV_CONFIG_INIT;
                    next_data_reg          = `VL53L1X_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND_ADDR;
                    next_data_tx           = 8'h01;
                    next_read_write_in     = `I2C_WRITE;
                    next_is_2_byte_reg     = `TRUE;
                end
                `I2C_VL53L1X_STATE_SET_TEMPERATURE_VHV_CONFIG_INIT: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_VL53L1X_STATE_SET_MEASUREMENT_PERIOD_RX_OSC_VAL;
                    next_data_reg          = `VL53L1X_VHV_CONFIG_INIT_ADDR;
                    next_data_tx           = 8'h01;
                    next_read_write_in     = `I2C_WRITE;
                    next_is_2_byte_reg     = `TRUE;
                end
                `I2C_VL53L1X_STATE_SET_MEASUREMENT_PERIOD_RX_OSC_VAL: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_VL53L1X_STATE_SET_MEASUREMENT_PERIOD_CALCULATE;
                    next_data_reg          = `VL53L1X_RESULT_OSC_CALIBRATE_VAL_ADDR;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_read_write_in     = `I2C_READ;
                    next_rx_from_VL53L1X   = `TRUE;
                    next_is_2_byte_reg     = `TRUE;
                    next_target_read_count = 5'd2;
                    set_VL53L1X_data_rx_reg_index  = `TRUE;
                    next_VL53L1X_data_rx_reg_index = `VL53L1X_DATA_RX_REG_RESULT_OSC_CAL_VAL_HI_INDEX;
                end
                `I2C_VL53L1X_STATE_SET_MEASUREMENT_PERIOD_CALCULATE: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_i2c_state         = `I2C_VL53L1X_STATE_SET_MEASUREMENT_PERIOD_TX_PERIOD;
                    // measurement period = result from previous * 100 * 1.075 = result from previous * 107.5 = (result from previous * 107)/2
                    next_VL53L1X_measurement_period = ((VL53L1X_osc_cal_val * 107)>>>2);
                end
                `I2C_VL53L1X_STATE_SET_MEASUREMENT_PERIOD_TX_PERIOD: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_measurement_period_tx_index = measurement_period_tx_index - 3'd1;
                    if(measurement_period_tx_index == 3'd0)
                        next_return_state  = `I2C_VL53L1X_STATE_START_MEASURE;
                    else
                        next_return_state  = `I2C_VL53L1X_STATE_SET_MEASUREMENT_PERIOD_TX_PERIOD;
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_data_reg          = `VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD_3_ADDR;
                    case(measurement_period_tx_index)
                        3'd0    : next_data_tx = VL53L1X_measurement_period[7 :0 ]; 
                        3'd1    : next_data_tx = VL53L1X_measurement_period[15:8 ];  
                        3'd2    : next_data_tx = VL53L1X_measurement_period[23:16]; 
                        3'd3    : next_data_tx = VL53L1X_measurement_period[31:24];
                        default : next_data_tx = 8'd0;
                    endcase
                    next_read_write_in     = `I2C_WRITE;
                    next_is_2_byte_reg     = `TRUE;
                end
                `I2C_VL53L1X_STATE_START_MEASURE: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    clear_waiting_ms       = `CLEAR_MS_TIMER; // Clear and set to wait_ms value
                    count_ms_init_time     = `FALSE;          // Make sure this is a POLL_INTERVAL timer
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_DRV_STATE_WAIT_IMU_POLL_TIME;
                    next_data_reg          = `VL53L1X_SYSTEM_MODE_START_ADDR;
                    next_data_tx           = 8'h40;
                    next_read_write_in     = `I2C_WRITE;
                    next_is_2_byte_reg     = `TRUE;
                    next_resetn_VL53L1X_buffer  = `LOW; // Clear VL53L1X RX data buffer index and data ready
                end
                `I2C_DRV_STATE_WAIT_IMU_POLL_TIME: begin     // Wait 20 ms between polls to maintain 50Hz polling rate
                                                            // wait time is i2c time + time spent here, for a total of 20ms,
                                                            // i2c time is variable and dependent on slave
                                                            // This timer starts at the beginning of the the previous state
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    clear_waiting_ms       = `RUN_MS_TIMER;
                    next_data_reg          = `ALL_ZERO_2BYTE;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_go_flag           = `NOT_GO;
                    next_resetn_VL53L1X_buffer  = `LOW; // Clear VL53L1X RX data buffer index and data ready
                    if(delay_timer_done)
                        next_i2c_state     = `I2C_VL53L1X_STATE_POLL_READY;
                    else
                        next_i2c_state     = `I2C_DRV_STATE_WAIT_IMU_POLL_TIME;
                end
                `I2C_VL53L1X_STATE_POLL_READY: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    if(VL53L1X_data_rdy[0] == 1'b1) // If measurement ready, go to next state
                        next_i2c_state     = `I2C_VL53L1X_STATE_GET_MEASUREMENT;
                    else
                        next_i2c_state     = `I2C_DRV_SUB_STATE_START;
                      
                    next_return_state      = `I2C_VL53L1X_STATE_POLL_READY;
                    next_data_reg          = `VL53L1X_GPIO_TIO_HV_STATUS_ADDR;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_read_write_in     = `I2C_READ;
                    next_rx_from_VL53L1X   = `TRUE;
                    next_is_2_byte_reg     = `TRUE;
                    next_target_read_count = 5'd1;
                    set_VL53L1X_data_rx_reg_index  = `TRUE;
                    next_VL53L1X_data_rx_reg_index = `VL53L1X_DATA_RX_REG_DATA_RDY_INDEX;
                end
                `I2C_VL53L1X_STATE_GET_MEASUREMENT: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_VL53L1X_STATE_CLEAR_INTERRUPT;
                    next_data_reg          = `VL53L1X_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_ADDR;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_read_write_in     = `I2C_READ;
                    next_rx_from_VL53L1X   = `TRUE;
                    next_is_2_byte_reg     = `TRUE;
                    next_target_read_count = 5'd2;
                    set_VL53L1X_data_rx_reg_index  = `TRUE;
                    next_VL53L1X_data_rx_reg_index = `VL53L1X_DATA_RX_REG_RESULT_RANGE_MEASURE_MM_HI_INDEX;
                end
                `I2C_VL53L1X_STATE_CLEAR_INTERRUPT: begin
                    next_imu_good          = `TRUE;
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                    next_go_flag           = `NOT_GO;
                    clear_waiting_ms       = `CLEAR_MS_TIMER; // Clear and set to wait_ms value
                    count_ms_init_time     = `FALSE;          // Make sure this is a POLL_INTERVAL timer
                    next_i2c_state         = `I2C_DRV_SUB_STATE_START;
                    next_return_state      = `I2C_DRV_STATE_WAIT_IMU_POLL_TIME;
                    next_data_reg          = `VL53L1X_SYSTEM_INTERRUPT_CLEAR_ADDR;
                    next_data_tx           = 8'h01;
                    next_read_write_in     = `I2C_WRITE;
                    next_is_2_byte_reg     = `TRUE;
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
                    clear_waiting_ms       = `CLEAR_MS_TIMER; // Clear and set to wait_ms value
                    count_ms_init_time     = `FALSE;          // Make sure this is a POLL_INTERVAL timer
                    if(~busy && resetn_imu) // Stay here until i2c is not busy AND the IMU isn't in reset (Prevent glitch at WD event)
                        next_i2c_state     = `I2C_DRV_SUB_STATE_STOP;
                    else
                        next_i2c_state     = `I2C_DRV_SUB_STATE_WAIT_I2C;
                end // Set output data latch strobe and return to major FSM state
                `I2C_DRV_SUB_STATE_STOP: begin
                    next_go_flag           = `NOT_GO;
                    next_data_reg          = `ALL_ZERO_2BYTE;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    clear_waiting_ms       = `CLEAR_MS_TIMER; // Clear and set to wait_ms value
                    next_i2c_state         = `I2C_STATE_WAIT_1MS;
                    if(read_write_in == `I2C_READ) // Only latch data if this was a read
                        rx_data_latch_strobe = `HIGH;
                end
                `I2C_STATE_WAIT_1MS: begin
                    next_imu_good          = `FALSE;
                    clear_waiting_ms       = `RUN_MS_TIMER;
                    next_data_reg          = `ALL_ZERO_2BYTE;
                    next_data_tx           = `BYTE_ALL_ZERO;
                    next_go_flag           = `NOT_GO;
                    //next_resetn_BNO055_buffer   = `LOW; // Clear BNO055 RX data buffer index
                    //next_resetn_VL53L1X_buffer  = `LOW; // Clear VL53L1X RX data buffer index and data ready
                    // Wait for I2C to be not busy and delay timer done
                    if(delay_timer_done)
                        next_i2c_state     = return_state;
                    else
                        next_i2c_state     = `I2C_STATE_WAIT_1MS;
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
                    next_rx_from_VL53L1X   = `FALSE;
                    next_is_2_byte_reg     = `FALSE;
                    //next_resetn_VL53L1X_buffer  = `LOW; // Clear VL53L1X RX data buffer index and data ready
                    next_target_read_count = 1'b1;
                    rx_data_latch_strobe   = `LOW;
                    i2c_number             = 1'b0; // Default to i2c EFB #1
                    next_slave_address     = `VL53L1X_SLAVE_ADDRESS;
                end
            endcase
        end
    end
endmodule
