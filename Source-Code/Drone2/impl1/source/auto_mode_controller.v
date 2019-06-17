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
    output wire [`RATE_BIT_WIDTH-1:0] z_linear_velocity,
    output reg  [31:0] debug,
    output reg  signed [31:0] z_linear_accel_zeroed,
    input  wire imu_good,
    input  wire [`IMU_VAL_BIT_WIDTH-1:0] z_linear_accel,
    input  wire [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_val_in,
    input  wire [2:0] switch_a,
    input  wire [1:0] switch_b,
    input  wire start_signal,
    input  wire resetn,
    input  wire us_clk
);

    // number of total FSM states, determines the number of required bits for states
    `define AMC_NUM_STATES 8
    // state names
    localparam
        STATE_INIT           = `AMC_NUM_STATES'b1<<0,
        STATE_WAIT           = `AMC_NUM_STATES'b1<<1,
        STATE_LATCH          = `AMC_NUM_STATES'b1<<2,
        STATE_ZERO_ACCEL     = `AMC_NUM_STATES'b1<<3,
        STATE_LAST_ACCEL     = `AMC_NUM_STATES'b1<<4,
        STATE_CALC_VELOCITY  = `AMC_NUM_STATES'b1<<5,
        STATE_AUTO_SUB_STATE = `AMC_NUM_STATES'b1<<6,
        STATE_COMPLETE       = `AMC_NUM_STATES'b1<<7;


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
        STATE_AUTO_NOT_AUTO          = `AMC_NUM_AUTO_STATES'b1<<8;

    reg next_complete_signal;
    reg next_active_signal;
    reg small_accel_val_now;
    reg small_accel_val_prev;
    reg next_small_accel_val_now;
    reg next_small_accel_val_prev;
	reg signed [`RATE_BIT_WIDTH-1:0] next_z_linear_velocity;
	reg signed [31:0] z_linear_velocity_internal;
	reg signed [31:0] next_z_linear_velocity_internal;
	reg signed [31:0] z_linear_accel_latched;
	reg signed [31:0] next_z_linear_accel_latched;
	reg signed [31:0] z_linear_accel_latched_zero;
	reg signed [31:0] next_z_linear_accel_latched_zero;
    reg signed [31:0] next_z_linear_accel_zeroed;
    reg signed [31:0] next_debug;
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
            small_accel_val_now         <= `FALSE;
            small_accel_val_prev        <= `FALSE;
            throttle_pwm_val_calc       <= 8'b0;
			z_linear_accel_latched      <= 32'b0;
            z_linear_accel_latched_zero <= 32'b0;
			throttle_pwm_val_latched    <= 8'b0;
			z_linear_velocity_internal  <= 32'b0;
			throttle_pwm_val_out        <= 8'b0;
			z_linear_accel_zeroed       <= 32'b0;
            in_air_flag                 <= `FALSE;
            on_ground_flag              <= `TRUE;
			debug                       <= 32'b0;
        end
        else begin
            state                       <= next_state;
            auto_state                  <= next_auto_state;
            complete_signal             <= next_complete_signal;
            active_signal               <= next_active_signal;
            small_accel_val_now         <= next_small_accel_val_now;
            small_accel_val_prev        <= next_small_accel_val_prev;
            throttle_pwm_val_calc       <= next_throttle_pwm_val_calc;
			z_linear_accel_latched      <= next_z_linear_accel_latched;
            z_linear_accel_latched_zero <= next_z_linear_accel_latched_zero;
			throttle_pwm_val_latched    <= next_throttle_pwm_val_latched;
			z_linear_velocity_internal  <= next_z_linear_velocity_internal;
			throttle_pwm_val_out        <= next_throttle_pwm_val_out;
			z_linear_accel_zeroed       <= next_z_linear_accel_zeroed;
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
                STATE_LATCH          : next_state = STATE_ZERO_ACCEL;
                STATE_ZERO_ACCEL     : next_state = STATE_LAST_ACCEL;
                STATE_LAST_ACCEL     : next_state = STATE_CALC_VELOCITY;
                STATE_CALC_VELOCITY  : next_state = STATE_AUTO_SUB_STATE;
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
                                                    next_auto_state = STATE_AUTO_NOT_AUTO;
                                                // Z linear velocity is zero, we're on the ground
                                                else if ( (z_linear_velocity_internal  < 'sd10) && (z_linear_velocity_internal  > (-'sd10)) ) 
                                                //else if (z_linear_velocity_internal  == 0)
                                                    next_auto_state = STATE_AUTO_ON_GND_THROT_OFF;
                                                else                                 //Auto mode selected and down direction
                                                    next_auto_state = STATE_AUTO_ON_GROUND;
                                            end
                    STATE_AUTO_ON_GND_THROT_OFF :
                                            begin
                                                if (throttle_pwm_val_latched < 'd10)  //Throttle in off position or radio off
                                                    next_auto_state = STATE_AUTO_FAILSAFE;
                                                else if (~switch_a[`SWITCH_A_AUTO]) //Not Auto mode, exit auto and go to not auto
                                                    next_auto_state = STATE_AUTO_NOT_AUTO;
                                                else
                                                    next_auto_state = STATE_AUTO_ON_GND_THROT_OFF;
                                            end
                    STATE_AUTO_UP_START   : next_auto_state = STATE_AUTO_GO_UP;
                    STATE_AUTO_DOWN_START : next_auto_state = STATE_AUTO_GO_DOWN;
                    STATE_AUTO_GO_UP      : begin
                                                if (throttle_pwm_val_latched < 'd10)   //Throttle in off position or radio off
                                                    next_auto_state = STATE_AUTO_FAILSAFE;
                                                else if (~switch_a[`SWITCH_A_AUTO]) //Not Auto mode, exit auto and go to not auto
                                                    next_auto_state = STATE_AUTO_NOT_AUTO;
                                                else if (~count__auto_time_ms[27]) 
                                                    next_auto_state = STATE_AUTO_GO_UP;
                                                else // In auto mode, timer expired, now we are in the air
                                                    next_auto_state = STATE_AUTO_IN_AIR;
                                            end
                    STATE_AUTO_GO_DOWN    : begin
                                                if (throttle_pwm_val_latched < 'd10) //Throttle in off position or radio off
                                                    next_auto_state = STATE_AUTO_FAILSAFE;
                                                else if (~switch_a[`SWITCH_A_AUTO]) //Not Auto mode, exit auto and go to not auto
                                                    next_auto_state = STATE_AUTO_NOT_AUTO;
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
                                                    next_auto_state = STATE_AUTO_NOT_AUTO;
                                                else
                                                    next_auto_state = STATE_AUTO_IN_AIR;
                                            end
                    STATE_AUTO_NOT_AUTO   : begin
                                                if (switch_a[`SWITCH_A_AUTO]) //Auto mode
                                                    next_auto_state = STATE_AUTO_IN_AIR;
                                                else
                                                    next_auto_state = STATE_AUTO_NOT_AUTO;
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
            next_small_accel_val_now         = `FALSE;
            next_small_accel_val_prev        = `FALSE;
			next_z_linear_accel_latched      = 32'b0;
            next_z_linear_accel_latched_zero = 32'b0;
            next_z_linear_accel_zeroed       = 32'b0;
			next_throttle_pwm_val_latched    = 8'b0;
            next_throttle_pwm_val_calc       = 8'b0;
            next_throttle_pwm_val_out        = 8'b0;
			next_z_linear_velocity_internal  = 32'b0;
            next_in_air_flag                 = `FALSE;
            next_on_ground_flag              = `TRUE;
		end
        else begin
			clear_auto_mode_timer            = `FALSE;
            next_small_accel_val_now         = small_accel_val_now;
            next_small_accel_val_prev        = small_accel_val_prev;
			next_z_linear_accel_latched      = z_linear_accel_latched;
            next_z_linear_accel_latched_zero = z_linear_accel_latched_zero;
            next_z_linear_accel_zeroed       = z_linear_accel_zeroed;
			next_throttle_pwm_val_latched    = throttle_pwm_val_latched;
			next_z_linear_velocity_internal  = z_linear_velocity_internal;
            next_throttle_pwm_val_calc       = throttle_pwm_val_calc;
			next_throttle_pwm_val_out        = throttle_pwm_val_out;
            next_in_air_flag                 = in_air_flag;
            next_on_ground_flag              = on_ground_flag;
            next_debug                       = debug;
            case(state)
                STATE_INIT          : begin
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
                STATE_LATCH         : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `TRUE;
                    clear_auto_mode_timer            = `FALSE;
                    next_throttle_pwm_val_latched    = throttle_pwm_val_in;
					next_z_linear_accel_latched      = $signed({z_linear_accel, 16'b0});
                    // Zero acceleration  throttle when throttle idle
                    if (throttle_pwm_val_in < 'd10)
                        next_z_linear_accel_latched_zero = $signed({z_linear_accel, 16'b0});
                    // Decoupled from throttle, allows debug of function without running throttle
                    // if (z_linear_accel_latched_zero == 32'd0)
                    //    next_z_linear_accel_latched_zero = z_linear_accel_latched;
                end
                STATE_ZERO_ACCEL         : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `TRUE;
                    clear_auto_mode_timer            = `FALSE;
                    next_z_linear_accel_zeroed       = $signed(z_linear_accel_latched - z_linear_accel_latched_zero);
                end
                STATE_LAST_ACCEL         : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `TRUE;
                    clear_auto_mode_timer            = `FALSE;
                    next_small_accel_val_now         = ((z_linear_accel_zeroed <= $signed((10)<<<16)) && (z_linear_accel_zeroed >= $signed((-10)<<<16)));
                    next_small_accel_val_prev        = small_accel_val_now;
                end
				STATE_CALC_VELOCITY : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `TRUE;
                    clear_auto_mode_timer            = `FALSE;
                    next_debug                       = auto_state;
                    /*
                    Don't accumulate on small accelerations, remove from accumulated value to remove error over time
                    if there is no accel, we're probably not moving. Zero acceleration is difficult to achieve in reality
                    Subtract from vel if its positive and add to it if its negative
                    remove amount is shifted 10 bits, so it's only removing 0.015625 m/s per iteration
                    
                    Only do this if previous and current accel values were both small, otherwise accumulate normally
                    This prevents issues with acceleration crossover (negative to positive and vice versa) zeroing velocity prematurely
                    */
                    if (small_accel_val_now && small_accel_val_prev) begin
                        if ((z_linear_velocity_internal < $signed((10)<<<10)) && (z_linear_velocity_internal > $signed((-10)<<<10))) begin
                            next_z_linear_velocity_internal  = 0;
                        end
                        else if ($signed(z_linear_velocity_internal>>>8) > 0) begin
				            next_z_linear_velocity_internal  = (z_linear_velocity_internal - $signed('sd10<<<10));
                        end
                        else if ($signed(z_linear_velocity_internal>>>8) < 0) begin
                            next_z_linear_velocity_internal  = (z_linear_velocity_internal + $signed('sd10<<<10));
                        end
                        else begin
                            next_z_linear_velocity_internal  = 0;
                        end
                    end
                    else begin
                        if ($signed(z_linear_velocity_internal>>>8) > 0) 
				            next_z_linear_velocity_internal  = calc_z_linear_velocity(z_linear_velocity_internal, z_linear_accel_zeroed) - $signed(1<<<0);
                        else if ($signed(z_linear_velocity_internal>>>8) < 0)
				            next_z_linear_velocity_internal  = calc_z_linear_velocity(z_linear_velocity_internal, z_linear_accel_zeroed) + $signed(1<<<0);
                        else
				            next_z_linear_velocity_internal  = calc_z_linear_velocity(z_linear_velocity_internal, z_linear_accel_zeroed);
                    end
				end
                STATE_AUTO_SUB_STATE : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `TRUE;
                end
                STATE_COMPLETE      : begin
                    next_complete_signal             = `TRUE;
                    next_active_signal               = `FALSE;
                    clear_auto_mode_timer            = `FALSE;
                    next_z_linear_velocity_internal  = throttle_pwm_val_calc;
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
                STATE_AUTO_NOT_AUTO         : begin
                    next_throttle_pwm_val_calc = throttle_pwm_val_latched;
                end
                endcase
            end
		end
    end

	// Drive linear velocity module output from high 16 bits of internal linear velocity value
	assign z_linear_velocity = z_linear_velocity_internal[24:9];

    // Calculate linear velocity from linear acceleration and previous linear velocity
    function automatic signed [31:0] calc_z_linear_velocity;
       input reg signed [31:0] z_linear_velocity_calc;
       input reg signed [31:0] z_linear_accel_calc;
       /*
       m/s^2 time per iteration                           = 0.040 (delta_t Runs every 40 ms)
       value of each linear acceleration bit MSB          = 1/100 m/s^2 = 0.01 m/s^2
       value of each linear acceleration bit MSB per 40ms = 0.01*0.040= 0.0004
       in 32 bits, binary value = 0.00000000000110100011011011100010
                 0.0  0  0  0  0  0  0  0  0  0  0  1  1  0  1  0  0  0  1  1  0  1  1  0  1  1  1  0  0  0  1  0
        SHAMT     01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32
        current_velocity = old_velocity + a * delta_t = old_velocity + a>>12 + a>>13 + a>>15....
       */
       calc_z_linear_velocity = (  z_linear_velocity_calc +
                                  (z_linear_accel_calc>>>12) +
                                  (z_linear_accel_calc>>>13) +
                                  (z_linear_accel_calc>>>15) +
                                  (z_linear_accel_calc>>>19) +
                                  (z_linear_accel_calc>>>20) +
                                  (z_linear_accel_calc>>>22) +
                                  (z_linear_accel_calc>>>23) +
                                  (z_linear_accel_calc>>>25) +
                                  (z_linear_accel_calc>>>26) +
                                  (z_linear_accel_calc>>>27) +
                                  (z_linear_accel_calc>>>31)  );
	endfunction

endmodule