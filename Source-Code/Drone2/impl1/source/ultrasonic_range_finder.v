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
`timescale 100ns / 100ns

`include "common_defines.v"

module ultrasonic_range_finder (
    output reg  active_signal,
    output reg  complete_signal,
    output reg  urf_trigger_out,
    output wire [9:0] urf_range,
    output reg  urf_valid,
    output wire [15:0] urf_debug,
    input  wire urf_echo_in,
    input  wire resetn,
    input  wire us_clk
);

    // number of total FSM states, determines the number of required bits for states
    `define URF_NUM_STATES 10
    // state names
    localparam
        STATE_INIT0      = `URF_NUM_STATES'b1<<0,
        STATE_INIT1      = `URF_NUM_STATES'b1<<1,
        STATE_WAIT       = `URF_NUM_STATES'b1<<2,
        STATE_TRIG_START = `URF_NUM_STATES'b1<<3,
        STATE_TRIG_WAIT  = `URF_NUM_STATES'b1<<4,
        STATE_ECHO_WAIT  = `URF_NUM_STATES'b1<<5,
        STATE_ECHO_COUNT = `URF_NUM_STATES'b1<<6,
        STATE_RANGE_CALC = `URF_NUM_STATES'b1<<7,
        STATE_COMPLETE   = `URF_NUM_STATES'b1<<8,
        STATE_ECHO_ERROR = `URF_NUM_STATES'b1<<9;

    reg next_complete_signal;
    reg next_active_signal;
    reg next_urf_trigger_out;
    reg next_urf_valid;
    reg [17:0]urf_range_calc;
    reg [17:0]next_urf_range_calc;
    reg [17:0]high_us_start;
    reg [17:0]next_high_us_start;
    reg [17:0]high_us_diff;
    reg [17:0]next_high_us_diff;

    reg [17:0]count_us_time; // Count from 0 to value determined by clock rate, used to generate N us delay trigger
                             // Each count value indicates one us
    reg clear_us_timer;      // Reset auto mode waiting X ms timer.

    // state variables
    reg [`URF_NUM_STATES-1:0] state, next_state;
    //  Generates counts up from 0 to max us count - Defaulted to 80us total time
    //  When the count reaches max value it stops incrementing
    always@(posedge us_clk, posedge clear_us_timer, negedge resetn) begin
        if(~resetn)
            count_us_time <= 18'd0; // Default to 0 us count
        else if( clear_us_timer == `TRUE )
            count_us_time <= 18'd0; // Clear to 0 us count
        else if( count_us_time <= (1000*80) ) // Increment only if counter hasn't reached max time yet
            count_us_time <= (count_us_time + 18'd1);
        // Otherwise leave count unchanged
    end

    // update state
    always @(posedge us_clk or negedge resetn) begin
        if(!resetn) begin
            state           <= STATE_INIT0;
            complete_signal <= `FALSE;
            active_signal   <= `FALSE;
            urf_trigger_out <= `FALSE;
            urf_valid       <= `FALSE;
			high_us_start   <= 0;
			high_us_diff    <= 0;
			urf_range_calc  <= 0;
            urf_valid       <= `FALSE;
        end
        else begin
            state           <= next_state;
            complete_signal <= next_complete_signal;
            active_signal   <= next_active_signal;
            urf_trigger_out <= next_urf_trigger_out;
            high_us_start   <= next_high_us_start;
            high_us_diff    <= next_high_us_diff;
			urf_range_calc  <= next_urf_range_calc;
            urf_valid       <= next_urf_valid;
        end
    end

    // Determine next state
    always@* begin
        if(!resetn) begin
            next_state = STATE_INIT0;
        end
        else begin
            case(state)
                STATE_INIT0      : next_state = STATE_INIT1;
                STATE_INIT1      : next_state = STATE_WAIT;
                STATE_WAIT       : begin
                    //Stay here until timer reaches max
                    if (count_us_time >= (1_000*80)) begin
                        if (urf_echo_in == `LOW) //Good
                            next_state = STATE_TRIG_START;
                        else //Still high, something weird happened
                            next_state = STATE_ECHO_ERROR;
                    end
                    else
                        next_state = STATE_WAIT;
                end
                STATE_TRIG_START : next_state = STATE_TRIG_WAIT;
                //Trigger to stay high for 10 us
                STATE_TRIG_WAIT  : begin
                    if (count_us_time == (1_000*80)) //Hit max time, dunno why
                        next_state = STATE_ECHO_ERROR;
                    else if  (count_us_time == 10)
                        next_state = STATE_ECHO_WAIT;
                    else
                        next_state = STATE_TRIG_WAIT;
                end
                STATE_ECHO_WAIT  : begin
                    if (count_us_time == (1_000*80)) //Hit max time but never saw echo, sensor connected?
                        next_state = STATE_ECHO_ERROR;
                    else if (urf_echo_in == `HIGH)
                        next_state = STATE_ECHO_COUNT;
                    else
                        next_state = STATE_ECHO_WAIT;
                end
                STATE_ECHO_COUNT : begin
                    if (count_us_time == (1_000*80)) //Hit max time but never saw echo go low, sensor error
                        next_state = STATE_ECHO_ERROR;
                    else if (urf_echo_in == `LOW)
                        next_state = STATE_RANGE_CALC;
                    else //Echo pin still high, count longer
                        next_state = STATE_ECHO_COUNT;
                end
                STATE_RANGE_CALC : next_state = STATE_COMPLETE;
                STATE_COMPLETE   : next_state = STATE_WAIT;
                STATE_ECHO_ERROR : next_state = STATE_COMPLETE;
            endcase
        end
            
    end


    // FSM values and output
    always@* begin
        if(!resetn) begin
            next_complete_signal = `FALSE;
            next_active_signal   = `FALSE;
            next_urf_trigger_out = `FALSE;
			clear_us_timer       = `FALSE;
            next_high_us_start   = 0;
            next_high_us_diff    = 0;
            next_urf_range_calc  = 0;
            next_urf_valid       = `FALSE;
		end
        else begin
            next_urf_trigger_out = urf_trigger_out;
			clear_us_timer       = `FALSE;
            next_high_us_start   = high_us_start;
            next_high_us_diff    = high_us_diff;
            next_urf_range_calc  = urf_range_calc;
            next_urf_valid       = urf_valid;
            case(state)
                STATE_INIT0      : begin
                    next_complete_signal = `FALSE;
                    next_active_signal   = `FALSE;
                    next_urf_valid       = `FALSE;
			        clear_us_timer       = `FALSE;
                end
                STATE_INIT1      : begin
                    next_complete_signal = `FALSE;
                    next_active_signal   = `FALSE;
                    next_urf_valid       = `FALSE;
			        clear_us_timer       = `TRUE;
                end
                STATE_WAIT       : begin
                    next_complete_signal = `FALSE;
                    next_active_signal   = `FALSE;
                end
                STATE_TRIG_START : begin
                    next_complete_signal = `FALSE;
                    next_active_signal   = `TRUE;
			        clear_us_timer       = `TRUE;
			        next_urf_trigger_out = `TRUE;
                end
                STATE_TRIG_WAIT  : begin
                    next_complete_signal = `FALSE;
                    next_active_signal   = `TRUE;
			        next_urf_trigger_out = `TRUE;
                end
                STATE_ECHO_WAIT  : begin
                    next_complete_signal = `FALSE;
                    next_active_signal   = `TRUE;
			        next_urf_trigger_out = `FALSE;
                    next_high_us_start   = count_us_time;
                end
                STATE_ECHO_COUNT : begin
                    next_complete_signal = `FALSE;
                    next_active_signal   = `TRUE;
                    next_high_us_diff    = ((count_us_time - high_us_start) + 18'd1);
                end
                STATE_RANGE_CALC : begin
                    next_complete_signal = `FALSE;
                    next_active_signal   = `TRUE;
                    // Max sensor range is 400cm, which gives max possible time of (58*401)-1 = 23257 us
                    // If max time range exceeded, leave old range value unchanged, but signal error
                    if(next_high_us_diff > 23257) begin
                        next_urf_range_calc  = urf_range_calc;
                        next_urf_valid       = `FALSE;
                    end
                    else begin
                        next_urf_range_calc  = calc_urf_range(high_us_diff);
                        next_urf_valid       = `TRUE;
                    end
                end
                STATE_COMPLETE   : begin
                    next_complete_signal = `TRUE;
                    next_active_signal   = `FALSE;
                end
                STATE_ECHO_ERROR : begin
                    next_complete_signal = `FALSE;
                    next_active_signal   = `TRUE;
                    // Leave old range value unchanged, but signal error
                    next_urf_valid       = `FALSE;
                    next_urf_range_calc  = urf_range_calc;
                end
            endcase
		end
    end
    
    
    assign urf_range = urf_range_calc[9:0];
    assign urf_debug = {6'd0, state};

    /*
     Calculate URF distance: (time in ms/58)
     
     Calculation in 32 bits from 18 bit  source data (reduces calculation error)
     
     REturns 10 bit result, 
     
     Formula:
     Distance = time (In microseconds) * 1/58
     1/58 = 0.0172413793103448
     in 32 bits, binary value = 0.00000100011010011110111001011000
               0.0  0  0  0  0  1  0  0  0  1  1  0  1  0  0  1  1  1  1  0  1  1  1  0  0  1  0  1  1  0  0  0
      SHAMT     01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32
      range = high_us_count * 1/58 = high_us_count>>6 + high_us_count>>10 + high_us_count>>11 + high_us_count>>13....
     
     */
    function automatic [17:0] calc_urf_range;
        input reg [17:0] urf_high_time_us;
        reg [31:0] urf_high_time_us_internal;
        reg [31:0] calc_urf_range_internal;
        begin
            //Put this valie in ipper 18 bits, pad lower 14 bits with 0 
            urf_high_time_us_internal = {urf_high_time_us, 14'd0};
            calc_urf_range_internal   = ( (urf_high_time_us_internal>>6) +
                                          (urf_high_time_us_internal>>10) +
                                          (urf_high_time_us_internal>>11) +
                                          (urf_high_time_us_internal>>13) +
                                          (urf_high_time_us_internal>>16) +
                                          (urf_high_time_us_internal>>17) +
                                          (urf_high_time_us_internal>>18) +
                                          (urf_high_time_us_internal>>19) +
                                          (urf_high_time_us_internal>>21) +
                                          (urf_high_time_us_internal>>22) +
                                          (urf_high_time_us_internal>>23) +
                                          (urf_high_time_us_internal>>26) +
                                          (urf_high_time_us_internal>>28) +
                                          (urf_high_time_us_internal>>29)  );
            //Remove padded low-order 14 bits from result
            calc_urf_range = calc_urf_range_internal[31:14];
        end
	endfunction

endmodule