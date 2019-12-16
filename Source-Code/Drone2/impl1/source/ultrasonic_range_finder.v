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
    output wire active_signal,
    output reg  urf_trigger_out,
    output wire [9:0] urf_range,
    output wire [15:0] z_linear_velocity, //8 integer bits and 8 fractional
    output wire [15:0] urf_debug,
    input  wire urf_echo_in,
    input  wire resetn,
    input  wire us_clk
);
  
    `define START         18'd0
    `define SET_TRIGGER   18'd10    //Assert trigger for 10us
    `define ECHO_COUNT    18'd23257 // Max sensor range is 400cm, which gives max possible time of (58*401)-1 = 23257 us
    `define RANGE_CALC    18'd23258
    `define VELOCITY_CALC 18'd23259
    `define WAIT          18'h3FFFF

    reg active;
    reg next_urf_trigger;
    reg error;
    reg next_error;
    reg [17:0]high_us_time;
    reg [17:0]next_high_us_time;
    reg [17:0]urf_range_calc;
    reg [17:0]next_urf_range_calc;
    reg [17:0]urf_range_last;
    reg [17:0]next_urf_range_last;
    reg signed [15:0]next_z_linear_velocity;
    reg signed [15:0]z_linear_velocity_calc;
    reg signed [15:0]next_z_linear_velocity_calc;

    reg [17:0]count_us_time; // Count from 0 to value determined by clock rate
                             // Each count value indicates one us

    //  Generates counts up from 0 to max us count (80ms)
    //  When the count reaches max value it wraps to 0
    always@(posedge us_clk, negedge resetn) begin
        if(~resetn)                          // Default to 0 us
            count_us_time <= 18'd0;
        else if( count_us_time < (1000*80) ) // Increment only if counter hasn't reached max time yet
            count_us_time <= (count_us_time + 18'd1);
        else
            count_us_time <= 18'd0;          // Count at max, set to 0
    end

    // Register Internals
    always @(posedge us_clk or negedge resetn) begin
        if(!resetn) begin
            urf_trigger_out        <= `FALSE;
            urf_range_calc         <= 0;
            urf_range_last         <= 0;
            z_linear_velocity_calc <= 0;
            high_us_time           <= 0;
            error                  <= `FALSE;
        end
        else begin
            urf_trigger_out        <= next_urf_trigger;
            urf_range_calc         <= next_urf_range_calc;
            urf_range_last         <= next_urf_range_last;
            z_linear_velocity_calc <= next_z_linear_velocity_calc;
            high_us_time           <= next_high_us_time;
            error                  <= next_error;
            
        end
    end

    // FSM values and output
    always@* begin
        if(!resetn) begin
            active                      = `FALSE;
            next_urf_trigger            = `FALSE;
            next_urf_range_calc         = 0;
            next_urf_range_last         = 0;
            next_z_linear_velocity_calc = 0;
            next_high_us_time           = 0;
            next_error                  = `FALSE;
        end
        else begin
            next_urf_trigger            = `FALSE;
            next_urf_range_calc         = urf_range_calc;
            next_urf_range_last         = urf_range_last;
            next_z_linear_velocity_calc = z_linear_velocity_calc;
            next_high_us_time           = high_us_time;
            next_error                  = error;
            case(`TRUE)
                in_range(count_us_time, `START, `SET_TRIGGER)      : begin   //Set Trigger
                    active                      = `TRUE;
                    next_urf_trigger            = `TRUE;
                    next_high_us_time           = 0;
                    next_urf_range_last         = urf_range;
                                                                             //If echo input already high, then generate error
                    next_error                  = urf_echo_in ? `TRUE : `FALSE;
                end
                in_range(count_us_time, `SET_TRIGGER, `ECHO_COUNT) : begin   //Count echo high time
                    active                      = `TRUE;
                    next_high_us_time           = urf_echo_in ? (high_us_time + 18'd1) : high_us_time;
                end
                in_range(count_us_time, `ECHO_COUNT, `RANGE_CALC)  : begin    //Distance calculation
                    active                      = `TRUE;
                                                                              // If echo error, hold previous range output
                    next_urf_range_calc         = error ? urf_range_calc : calc_urf_range(high_us_time + 18'd1);
                end
                in_range(count_us_time, `RANGE_CALC, `VELOCITY_CALC)  : begin //Velocity calculation
                    active                      = `TRUE;
                                                                              // If echo error, hold previous velocity output
                                                                              // Shifted right 8 bits to have 8 integer and 8 fractional bits for velocity
                    next_z_linear_velocity_calc = error ? z_linear_velocity_calc : calc_z_linear_velocity( ((urf_range_calc - urf_range_last)<<<8) );
                end
                in_range(count_us_time, `VELOCITY_CALC, `WAIT)        : begin // Set outputs, wait for next cycle
                    active                      = `FALSE;
                end
                default     : begin
                    active                      = `FALSE;
                    next_urf_trigger            = `FALSE;
                    next_urf_range_calc         = 0;
                    next_urf_range_last         = urf_range_last;
                    next_z_linear_velocity_calc = z_linear_velocity_calc;
                    next_high_us_time           = 0;
                    next_error                  = `FALSE;
                end
            endcase
        end
    end
    
    
    // Drive outputs
    assign urf_range         = urf_range_calc[9:0];
    assign active_signal     = active;
    assign urf_debug         = high_us_time[15:0];
    assign z_linear_velocity = z_linear_velocity_calc;

    /*
     Calculate URF distance: (time in ms/58)
     
     Calculation in 32 bits from 18 bit  source data (reduces calculation error)
     
     Returns 18 bit result, 
     
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
            //Put this value in upper 18 bits, pad lower 14 bits with 0 
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
    
    /*
     Calculate velocity from distance delta
     
     Calculation in 16 bits 
     
     
     Formula:
     Velocity = distance_delta * 0.080 (Runs every 80 ms)
     in 32 bits, binary value = 0.0001010001111011
               0.0  0  0  1  0  1  0  0  0  1  1  1  1  0  1  1
      SHAMT     01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16
     Velocity = z_linear_delta>>>4 + z_linear_delta>>>6 + z_linear_delta>>>10 + z_linear_delta>>>11....
     
     */
    function automatic signed [15:0] calc_z_linear_velocity;
        input reg signed [15:0] z_linear_delta;
        begin
            calc_z_linear_velocity    = ( (z_linear_delta>>>4)  +
                                          (z_linear_delta>>>6)  +
                                          (z_linear_delta>>>10) +
                                          (z_linear_delta>>>11) +
                                          (z_linear_delta>>>12) +
                                          (z_linear_delta>>>13) +
                                          (z_linear_delta>>>15) +
                                          (z_linear_delta>>>16)  );
        end
    endfunction   
    
    function automatic reg in_range;
        input reg [17:0]value;
        input reg [17:0]lower_lim;
        input reg [17:0]upper_lim;
        if ( (value >= lower_lim) && (value < upper_lim) )
            in_range = 1'b1;
        else
            in_range = 1'b0;
    endfunction

endmodule
