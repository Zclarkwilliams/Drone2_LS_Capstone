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
        STATE_INIT          = `AMC_NUM_STATES'b1<<0,
        STATE_NOT_AUTO      = `AMC_NUM_STATES'b1<<1,
        STATE_WAIT_AUTO     = `AMC_NUM_STATES'b1<<2,
        STATE_LATCH         = `AMC_NUM_STATES'b1<<3,
        STATE_ZERO_ACCEL    = `AMC_NUM_STATES'b1<<4,
        STATE_CALC_VELOCITY = `AMC_NUM_STATES'b1<<5,
        STATE_UP_DOWN       = `AMC_NUM_STATES'b1<<6,
        STATE_COMPLETE      = `AMC_NUM_STATES'b1<<7;


    reg next_complete_signal;
    reg next_active_signal;
	reg signed [`RATE_BIT_WIDTH-1:0] next_z_linear_velocity;
	reg signed [31:0] z_linear_velocity_internal;
	reg signed [31:0] next_z_linear_velocity_internal;
	reg signed [31:0] z_linear_accel_latched;
	reg signed [31:0] next_z_linear_accel_latched;
	reg signed [31:0] z_linear_accel_latched_zero;
	reg signed [31:0] next_z_linear_accel_latched_zero;
    reg signed [31:0] next_z_linear_accel_zeroed;
    reg signed [31:0] next_debug;
	reg [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_val_latched;
	reg [`REC_VAL_BIT_WIDTH-1:0] next_throttle_pwm_val_latched;
	reg [`REC_VAL_BIT_WIDTH-1:0] next_throttle_pwm_val_out;

    reg [27:0]count_ms;   //  Count from 0 to value determined by clock rate, used to generate N us delay trigger
    reg clear_waiting_ms; //  Reset waiting X us timer.

    // state variables
    reg [`AMC_NUM_STATES-1:0] state, next_state;
    reg start_flag = `FALSE;
	reg direction;
	reg next_direction;
    //  Generates a multiple of 1us length duration delay trigger - Defaulted to 10 seconds total time
    //  When the count down counter wraps around the timer is triggered and stops counting
    always@(posedge us_clk, negedge clear_waiting_ms, negedge resetn) begin
        if(~resetn)
            count_ms <= 28'hFFFFFFF;
        else if( clear_waiting_ms == 1'b0 )
            count_ms <= (1_000*10); //Reset to 10 seconds with a 1ms clock
        else if( count_ms != 28'hFFFFFFF )
            count_ms <= (count_ms - 1'b1);
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
            complete_signal             <= `FALSE;
            active_signal               <= `FALSE;
			direction                   <= 1'b0;
			z_linear_accel_latched      <= 32'b0;
            z_linear_accel_latched_zero <= 32'b0;
			throttle_pwm_val_latched    <= 8'b0;
			z_linear_velocity_internal  <= 32'b0;
			throttle_pwm_val_out        <= 8'b0;
			z_linear_accel_zeroed       <= 32'b0;
			debug                       <= 32'b0;
        end
        else begin
            state                       <= next_state;
            complete_signal             <= next_complete_signal;
            active_signal               <= next_active_signal;
			direction                   <= next_direction;
			z_linear_accel_latched      <= next_z_linear_accel_latched;
            z_linear_accel_latched_zero <= next_z_linear_accel_latched_zero;
			throttle_pwm_val_latched    <= next_throttle_pwm_val_latched;
			z_linear_velocity_internal  <= next_z_linear_velocity_internal;
			throttle_pwm_val_out        <= next_throttle_pwm_val_out;
			z_linear_accel_zeroed       <= next_z_linear_accel_zeroed;
			debug                       <= next_debug;
        end
    end

    // Determine next state
    always@* begin
        if(!resetn)
            next_state = STATE_INIT;
        else
            case(state)
                STATE_INIT          : next_state = ~imu_good    ? STATE_INIT     : STATE_NOT_AUTO;
                STATE_NOT_AUTO      : next_state = start_flag   ? STATE_LATCH    : STATE_NOT_AUTO;
                STATE_WAIT_AUTO     : next_state = start_flag   ? STATE_LATCH    : STATE_WAIT_AUTO;
                STATE_LATCH         : next_state = STATE_ZERO_ACCEL;
                STATE_ZERO_ACCEL    : next_state = STATE_CALC_VELOCITY;
                //STATE_CALC_VELOCITY : next_state = switch_a[1]  ? STATE_UP_DOWN  : STATE_COMPLETE;
                // Ignore switch position for now, just go to complete
                STATE_CALC_VELOCITY : next_state = STATE_COMPLETE;
                STATE_UP_DOWN       : next_state = STATE_COMPLETE;
                STATE_COMPLETE      : next_state = count_ms[27] ? STATE_NOT_AUTO : STATE_WAIT_AUTO;
            endcase
    end


    // FSM values and output
    always@* begin
        if(!resetn) begin
            next_complete_signal             = `FALSE;
            next_active_signal               = `FALSE;
			clear_waiting_ms                 = `FALSE;
			next_z_linear_accel_latched      = 32'b0;
            next_z_linear_accel_latched_zero = 32'b0;
            next_z_linear_accel_zeroed       = 32'b0;
			next_throttle_pwm_val_latched    = 8'b0;
			next_direction                   = 1'b0;
			next_z_linear_velocity_internal  = 32'b0;
		end
        else begin
			clear_waiting_ms                 = `FALSE;
			next_z_linear_accel_latched      = z_linear_accel_latched;
            next_z_linear_accel_latched_zero = z_linear_accel_latched_zero;
            next_z_linear_accel_zeroed       = z_linear_accel_zeroed;
			next_throttle_pwm_val_latched    = throttle_pwm_val_latched;
			next_direction                   = switch_b[0]? 1'b1 : 1'b0;
			next_z_linear_velocity_internal  = z_linear_velocity_internal;
			next_throttle_pwm_val_out        = throttle_pwm_val_out;
            next_debug                       = debug;
            case(state)
                STATE_INIT          : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `FALSE;
			        next_throttle_pwm_val_out        = 16'b0;
                end
                STATE_NOT_AUTO      : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `FALSE;
			        clear_waiting_ms                 = `FALSE;
					next_throttle_pwm_val_out        = throttle_pwm_val_latched;
                end
                STATE_LATCH         : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = active_signal;
					next_z_linear_accel_latched      = $signed({z_linear_accel, 16'b0});
                    //if (throttle_pwm_val_in < 'd10)
                    // Decoupled from throttle for now, allows debug of function without running throttle
                    if (z_linear_accel_latched_zero == 32'd0)
                        next_z_linear_accel_latched_zero = z_linear_accel_latched;
					next_throttle_pwm_val_latched    = throttle_pwm_val_in;
                end
                STATE_ZERO_ACCEL         : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = active_signal;
                    next_z_linear_accel_zeroed       = $signed(z_linear_accel_latched - z_linear_accel_latched_zero);
                end
				STATE_CALC_VELOCITY : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = active_signal;
                    next_debug                       = z_linear_accel_latched_zero;
                    // Don't accumulate on small accelerations, remove from accumulated value to remove error over time
                    // if there is no accel, we're probably not moving. Zero acceleration is difficult to achieve in reality
                    // Subtract from vel if its positive and add to it if its negative
                    // remove amount is shifted 10 bits, so it's only removing 0.015625 m/s per iteration
                    if ((z_linear_accel_zeroed <= $signed((10)<<<16)) && (z_linear_accel_zeroed >= $signed((-10)<<<16))) begin
                        if ((z_linear_velocity_internal < $signed((1)<<<10)) && (z_linear_velocity_internal > $signed((-1)<<<10))) begin
                            next_z_linear_velocity_internal  = 0;
                        end
                        else if ($signed(z_linear_velocity_internal[31:16]) > 0) begin
				            next_z_linear_velocity_internal  = (z_linear_velocity_internal - $signed(1<<10));
                        end
                        else if ($signed(z_linear_velocity_internal[31:16]) < 0) begin
                            next_z_linear_velocity_internal  = (z_linear_velocity_internal + $signed(1<<10));
                        end
                        else begin
                            next_z_linear_velocity_internal  = 0;
                        end
                    end
                    else begin
                        if ($signed(z_linear_velocity_internal[31:16]) > 0) 
				            next_z_linear_velocity_internal  = calc_z_linear_velocity(z_linear_velocity_internal, z_linear_accel_zeroed) - $signed(1<<1);
                        else if ($signed(z_linear_velocity_internal[31:16]) < 0)
				            next_z_linear_velocity_internal  = calc_z_linear_velocity(z_linear_velocity_internal, z_linear_accel_zeroed) + $signed(1<<1);
                        else
				            next_z_linear_velocity_internal  = calc_z_linear_velocity(z_linear_velocity_internal, z_linear_accel_zeroed);
                    end
				end
                STATE_WAIT_AUTO     : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `FALSE;
					// Just here for debug, will be removed later
					next_throttle_pwm_val_out        = throttle_pwm_val_latched;
                end
                STATE_UP_DOWN       : begin
                    next_complete_signal             = `FALSE;
                    next_active_signal               = `TRUE;
                end
                STATE_COMPLETE      : begin
                    next_complete_signal             = `TRUE;
                    next_active_signal               = `FALSE;
                end
            endcase
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