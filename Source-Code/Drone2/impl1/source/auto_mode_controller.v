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

 */
`timescale 1ns / 1ns

`include "common_defines.v"

module auto_mode_controller (
    output reg  [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_val_out,
    output reg  active_signal,
    output reg  complete_signal,
    output reg  [15:0] debug,
    input  wire imu_good,
    input  wire [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_val_in,
    input  wire [2:0] switch_a,
    input  wire [1:0] switch_b,
    input  wire [`RATE_BIT_WIDTH-1:0] z_linear_velocity,
    input  wire start_signal,
    input  wire resetn,
    input  wire us_clk
);

    // number of total FSM states, determines the number of required bits for states
    `define AMC_NUM_STATES 5
    // state names
    localparam
        STATE_INIT           = `AMC_NUM_STATES'b1<<0,
        STATE_WAIT           = `AMC_NUM_STATES'b1<<1,
        STATE_LATCH          = `AMC_NUM_STATES'b1<<2,
        STATE_AUTO_SUB_STATE = `AMC_NUM_STATES'b1<<3,
        STATE_COMPLETE       = `AMC_NUM_STATES'b1<<4;


    // number of total FSM auto states, determines the number of required bits for states
    `define AMC_NUM_AUTO_STATES 9
    // state names
    localparam
        STATE_AUTO_FAILSAFE          = `AMC_NUM_AUTO_STATES'b1<<0,
        STATE_AUTO_ON_GROUND         = `AMC_NUM_AUTO_STATES'b1<<1,
        STATE_AUTO_ON_GND_THROT_OFF  = `AMC_NUM_AUTO_STATES'b1<<2,
        STATE_AUTO_UP_START          = `AMC_NUM_AUTO_STATES'b1<<3,
        STATE_AUTO_DOWN_START        = `AMC_NUM_AUTO_STATES'b1<<4,
        STATE_AUTO_GO_UP             = `AMC_NUM_AUTO_STATES'b1<<5,
        STATE_AUTO_GO_DOWN           = `AMC_NUM_AUTO_STATES'b1<<6,
        STATE_AUTO_IN_AIR            = `AMC_NUM_AUTO_STATES'b1<<7,
        STATE_NOT_AUTO               = `AMC_NUM_AUTO_STATES'b1<<8;

    reg next_complete_signal;
    reg next_active_signal;
    reg [15:0] next_debug;
    reg [`RATE_BIT_WIDTH-1:0]    z_linear_velocity_latched;
    reg [`RATE_BIT_WIDTH-1:0]    next_z_linear_velocity_latched;
    reg [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_val_calc;
    reg [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_val_latched;
    reg [`REC_VAL_BIT_WIDTH-1:0] next_throttle_pwm_val_latched;
    reg [`REC_VAL_BIT_WIDTH-1:0] next_throttle_pwm_val_calc;
    reg [`REC_VAL_BIT_WIDTH-1:0] next_throttle_pwm_val_out;

    reg [27:0]count__auto_time_ms; //  Count from 0 to value determined by clock rate, used to generate N us delay trigger
                                   //  Each count value indicates one ms
    reg clear_auto_mode_timer;     //  Reset auto mode waiting X ms timer.

    // state variables
    reg [`AMC_NUM_STATES-1:0] state, next_state;
    // Auto-mode state variables
    reg [`AMC_NUM_AUTO_STATES-1:0] auto_state, next_auto_state;
    reg start_flag = `FALSE;
    reg in_air_flag;
    reg next_in_air_flag;
    reg on_ground_flag;
    reg next_on_ground_flag;
    //  Generates a multiple of 1us length duration delay trigger - Defaulted to 10 seconds total time
    //  When the count down counter wraps around the timer is triggered and stops counting
    always@(posedge us_clk, negedge clear_auto_mode_timer, negedge resetn) begin
        if(~resetn)
            count__auto_time_ms <= 28'hFFFFFFF;
        else if( clear_auto_mode_timer == 1'b0 )
            count__auto_time_ms <= (1_000*10); //Reset to 10 seconds with a 1ms clock
        else if( count__auto_time_ms != 28'hFFFFFFF )
            count__auto_time_ms <= (count__auto_time_ms - 1'b1);
        // Otherwise leave count_us unchanged.
    end

    // latch start signal
    always @(posedge us_clk or negedge resetn) begin
        if(!resetn)
            start_flag     <= `FALSE;
        else if(start_signal && !start_flag)
            start_flag     <= `TRUE;
        else if(!start_signal && start_flag) begin
            if(state == STATE_LATCH)
                start_flag <= `FALSE;
        end
        else
            start_flag     <= start_flag;
    end


    // update state
    always @(posedge us_clk or negedge resetn) begin
        if(!resetn) begin
            state                       <= STATE_INIT;
            auto_state                  <= STATE_AUTO_FAILSAFE;
            complete_signal             <= `FALSE;
            active_signal               <= `FALSE;
            throttle_pwm_val_calc       <= 8'b0;
            throttle_pwm_val_latched    <= 8'b0;
            throttle_pwm_val_out        <= 8'b0;
            in_air_flag                 <= `FALSE;
            on_ground_flag              <= `TRUE;
            debug                       <= 16'b0;
        end
        else begin
            state                       <= next_state;
            auto_state                  <= next_auto_state;
            complete_signal             <= next_complete_signal;
            active_signal               <= next_active_signal;
            throttle_pwm_val_calc       <= next_throttle_pwm_val_calc;
            throttle_pwm_val_latched    <= next_throttle_pwm_val_latched;
            throttle_pwm_val_out        <= next_throttle_pwm_val_out;
            in_air_flag                 <= next_in_air_flag;
            on_ground_flag              <= next_on_ground_flag;
            debug                       <= next_debug;
        end
    end

    // Determine next state
    always@* begin
        if(!resetn) begin
            next_state      = STATE_INIT;
            next_auto_state = STATE_AUTO_FAILSAFE;
        end
        else begin
            // Major FSM next state
            case(state)
                STATE_INIT           : next_state = ~imu_good  ? STATE_INIT  : STATE_WAIT;
                STATE_WAIT           : next_state = start_flag ? STATE_LATCH : STATE_WAIT;
                STATE_LATCH          : next_state = STATE_AUTO_SUB_STATE;
                STATE_AUTO_SUB_STATE : next_state = STATE_COMPLETE;
                STATE_COMPLETE       : next_state = STATE_WAIT;
            endcase
            
            // Auto mode (Minor) FSM next state
            //Only updated auto_state when current state is the major state just prior to the auto sub-state
            if (state != (STATE_AUTO_SUB_STATE - 1)) 
                next_auto_state = auto_state;
            else begin
                case(auto_state)
                    STATE_AUTO_FAILSAFE   : begin
                                                if (throttle_pwm_val_latched < 'd10) //Throttle in off position or radio off
                                                    next_auto_state = STATE_AUTO_FAILSAFE;
                                                else                                 // Throttle on
                                                    next_auto_state = STATE_AUTO_ON_GROUND;
                                            end
                    STATE_AUTO_ON_GROUND  : begin
                                                if (throttle_pwm_val_latched < 'd10) //Throttle in off position or radio off
                                                    next_auto_state = STATE_AUTO_FAILSAFE;
                                                else if (switch_a[`SWITCH_A_AUTO] && switch_b[`SWITCH_B_UP]) //Auto mode and up direction selected
                                                    next_auto_state = STATE_AUTO_UP_START;
                                                else if (~switch_a[`SWITCH_A_AUTO])               //Not Auto mode
                                                    next_auto_state = STATE_NOT_AUTO;
                                                // Z linear velocity is zero, we're on the ground 
                                                else if ( (z_linear_velocity_latched  < 'sd10) && (z_linear_velocity_latched  > (-'sd10)) ) 
                                                    next_auto_state = STATE_AUTO_ON_GND_THROT_OFF;
                                                else                                 //Auto mode selected and down direction
                                                    next_auto_state = STATE_AUTO_ON_GROUND;
                                            end
                    STATE_AUTO_ON_GND_THROT_OFF :
                                            begin
                                                if (throttle_pwm_val_latched < 'd10)  //Throttle in off position or radio off
                                                    next_auto_state = STATE_AUTO_FAILSAFE;
                                                else if (~switch_a[`SWITCH_A_AUTO]) //Not Auto mode, exit auto and go to not auto
                                                    next_auto_state = STATE_NOT_AUTO;
                                                else
                                                    next_auto_state = STATE_AUTO_ON_GND_THROT_OFF;
                                            end
                    STATE_AUTO_UP_START   : next_auto_state = STATE_AUTO_GO_UP;
                    STATE_AUTO_DOWN_START : next_auto_state = STATE_AUTO_GO_DOWN;
                    STATE_AUTO_GO_UP      : begin
                                                if (throttle_pwm_val_latched < 'd10)   //Throttle in off position or radio off
                                                    next_auto_state = STATE_AUTO_FAILSAFE;
                                                else if (~switch_a[`SWITCH_A_AUTO]) //Not Auto mode, exit auto and go to not auto
                                                    next_auto_state = STATE_NOT_AUTO;
                                                else if (~count__auto_time_ms[27]) 
                                                    next_auto_state = STATE_AUTO_GO_UP;
                                                else // In auto mode, timer expired, now we are in the air
                                                    next_auto_state = STATE_AUTO_IN_AIR;
                                            end
                    STATE_AUTO_GO_DOWN    : begin
                                                if (throttle_pwm_val_latched < 'd10) //Throttle in off position or radio off
                                                    next_auto_state = STATE_AUTO_FAILSAFE;
                                                else if (~switch_a[`SWITCH_A_AUTO]) //Not Auto mode, exit auto and go to not auto
                                                    next_auto_state = STATE_NOT_AUTO;
                                                else if (~count__auto_time_ms[27])
                                                    next_auto_state = STATE_AUTO_GO_DOWN;
                                                else
                                                    next_auto_state = STATE_AUTO_ON_GROUND;
                                            end
                    STATE_AUTO_IN_AIR     : begin
                                                if (throttle_pwm_val_latched < 'd10) //Throttle in off position or radio off
                                                    next_auto_state = STATE_AUTO_FAILSAFE;
                                                else if (switch_a[`SWITCH_A_AUTO] && switch_b[`SWITCH_B_DOWN]) //Auto mode and down direction selected
                                                    next_auto_state = STATE_AUTO_DOWN_START;
                                                else if (~switch_a[`SWITCH_A_AUTO]) //Not Auto mode
                                                    next_auto_state = STATE_NOT_AUTO;
                                                else
                                                    next_auto_state = STATE_AUTO_IN_AIR;
                                            end
                    STATE_NOT_AUTO        : begin
                                                if (switch_a[`SWITCH_A_AUTO]) //Auto mode
                                                    next_auto_state = STATE_AUTO_IN_AIR;
                                                else
                                                    next_auto_state = STATE_NOT_AUTO;
                                            end
                endcase
            end
        end
            
    end


    // FSM values and output
    always@* begin
        if(!resetn) begin
            next_complete_signal             = `FALSE;
            next_active_signal               = `FALSE;
            clear_auto_mode_timer            = `FALSE;
            next_z_linear_velocity_latched   = 16'd0;
            next_throttle_pwm_val_latched    = 8'b0;
            next_throttle_pwm_val_calc       = 8'b0;
            next_throttle_pwm_val_out        = 8'b0;
            next_in_air_flag                 = `FALSE;
            next_on_ground_flag              = `TRUE;
            next_debug                       = 32'd0;
        end
        else begin
            clear_auto_mode_timer            = `FALSE;
            next_z_linear_velocity_latched   = z_linear_velocity_latched;
            next_throttle_pwm_val_latched    = throttle_pwm_val_latched;
            next_throttle_pwm_val_calc       = throttle_pwm_val_calc;
            next_throttle_pwm_val_out        = throttle_pwm_val_out;
            next_in_air_flag                 = in_air_flag;
            next_on_ground_flag              = on_ground_flag;
            next_debug                       = {7'd0, auto_state};
            case(state)
                STATE_INIT           : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `FALSE;
                    clear_auto_mode_timer            = `FALSE;
                    next_throttle_pwm_val_out        = 16'b0;
                end
                STATE_WAIT           : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `FALSE;
                    clear_auto_mode_timer            = `FALSE;
                end
                STATE_LATCH          : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `TRUE;
                    clear_auto_mode_timer            = `FALSE;
                    next_throttle_pwm_val_latched    = throttle_pwm_val_in;
                    next_z_linear_velocity_latched   = z_linear_velocity;
                end
                STATE_AUTO_SUB_STATE : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `TRUE;
                end
                STATE_COMPLETE       : begin
                    next_complete_signal             = `TRUE;
                    next_active_signal               = `FALSE;
                    clear_auto_mode_timer            = `FALSE;
                end
            endcase
            
            if(state != (STATE_AUTO_SUB_STATE - 1)) begin            
                // Do nothing
            end
            else begin
                case(auto_state)
                STATE_AUTO_FAILSAFE         : begin
                    next_in_air_flag           = `FALSE;
                    next_on_ground_flag        = `TRUE;
                    next_throttle_pwm_val_out  = 0;
                end
                STATE_AUTO_ON_GROUND        : begin
                    // Probably on the ground, but not for certain until velocity is 0, leave flags alone
                    next_in_air_flag           = in_air_flag;
                    next_on_ground_flag        = on_ground_flag;
                    //next_throttle_pwm_val_calc = throttle_pwm_val_calc;
                    next_throttle_pwm_val_calc = 15;
                end
                STATE_AUTO_ON_GND_THROT_OFF : begin
                    // By the time velocity is zero we're definitely not in the air anymore
                    next_in_air_flag           = `FALSE;
                    next_on_ground_flag        = `TRUE;
                    // Cut the throttle off
                    next_throttle_pwm_val_calc = 0;
                end
                STATE_AUTO_UP_START         : begin
                    next_in_air_flag           = `TRUE;
                    next_on_ground_flag        = `FALSE;
                    clear_auto_mode_timer      = `TRUE;
                    next_throttle_pwm_val_calc = throttle_pwm_val_calc;
                end
                STATE_AUTO_DOWN_START       : begin
                    next_in_air_flag           = `TRUE;
                    next_on_ground_flag        = `FALSE;
                    clear_auto_mode_timer      = `TRUE;
                    next_throttle_pwm_val_calc = throttle_pwm_val_calc;
                end
                STATE_AUTO_GO_UP            : begin
                    next_throttle_pwm_val_calc = 25;
                end
                STATE_AUTO_GO_DOWN          : begin
                    next_throttle_pwm_val_calc = 20;
                end
                STATE_AUTO_IN_AIR           : begin
                    next_in_air_flag           = `TRUE;
                    next_on_ground_flag        = `FALSE;
                    //next_throttle_pwm_val_calc = throttle_pwm_val_calc;
                    next_throttle_pwm_val_calc = 30;
                end
                STATE_NOT_AUTO              : begin
                    next_throttle_pwm_val_calc = throttle_pwm_val_latched;
                end
                endcase
            end
        end
    end

endmodule