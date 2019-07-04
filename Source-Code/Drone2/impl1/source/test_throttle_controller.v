/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

`timescale 1ps / 1ps
`default_nettype none
`include "common_defines.v"

module test_throttle_controller;

    // PWM from receiver module
    reg signed [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_in = 0;
    wire signed [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_out;

    reg  resetn;
    wire sys_clk;
    wire us_clk;
    wire complete_signal;
    wire active_signal;
    reg  start_signal;
    integer i;


    defparam OSCH_inst.NOM_FREQ = "38.00";
    OSCH OSCH_inst (.STDBY(1'b0),
                    .OSC(sys_clk),
                    .SEDSTDBY());

    us_clk us_clk_divider (
        .us_clk(us_clk),
        .sys_clk(sys_clk),
        .resetn(resetn));

    // line up the parameters here to the ones internal to the receiver module
    throttle_controller DUT (
        .throttle_pwm_value_in(throttle_pwm_value_in),
        .throttle_pwm_value_out(throttle_pwm_value_out),
        .complete_signal(complete_signal),
        .active_signal(active_signal),
        .resetn(resetn),
        .start_signal(start_signal),
        .us_clk(us_clk));


    task run_test;
        input reg [15:0] task_throttle_pwm_value_in;
        begin
            //$display("%t: %m Throttle prev_latched_throttle=%d",$time, DUT.prev_latched_throttle);
            throttle_pwm_value_in = task_throttle_pwm_value_in;
            start_signal    = 1;
            repeat (2) @(posedge us_clk); start_signal    = 0; repeat (8) @(posedge us_clk);
            $display("%t: %m Throttle latched_throttle=%d",$time, DUT.latched_throttle);
            $display("%t: %m Throttle debounced_throttle=%d",$time, DUT.debounced_throttle);
            //$display("%t: %m Throttle scaled_throttle=%d",$time, DUT.scaled_throttle);
            $display("%t: %m Throttle limited_throttle=%d",$time, DUT.limited_throttle);
            $display("%t: %m Throttle prev_throttle_pwm_value_out=%d",$time, DUT.prev_throttle_pwm_value_out);
        end
    endtask

    initial begin
        //$display("%t: %m Reset throttle rate limiter", $time);
        resetn = 1;
        #10 resetn = 0;
        #10 resetn = 1;

        //$display("%t: %m Set initial values",$time);
        run_test(.task_throttle_pwm_value_in(0));

        $display("\n\n\n%t: %m Throttle to ranges 0 to 255, MAX",$time);
        
        /*for(i = 0; i < 256*8; i=i+1) begin
            $display("\n\n\n%t: %m iteration %d",$time, (i/8));
            run_test(.task_throttle_pwm_value_in((i/8)));
        end*/

        for(i = 0; i < 26*10; i=i+1) begin
            //$display("\n\n\n%t: %m iteration %d",$time, ((i/10)*10));
            run_test(.task_throttle_pwm_value_in(((i/10)*10)));
        end
        
        
        $display("%t: %m Test complete", $time);
        $display("\n\n\n%t: %m Throttle to ranges 255 to 0, MAX",$time);

        for(i = 25*10; i > 0; i=i-1) begin
            //$display("\n\n\n%t: %m iteration %d",$time, ((i/10)*10));
            run_test(.task_throttle_pwm_value_in(((i/10)*10)));
        end
        
        
        $display("%t: %m Test complete", $time);
        
        
        $display("\n\n\n%t: %m Throttle to 250, MAX",$time);

        for(i = 0; i < 62; i=i+1) begin
            run_test(.task_throttle_pwm_value_in(250));
        end
        
        $display("%t: %m Test with 1 PWM cycle of 0 input, nothing should happen", $time);
        run_test(.task_throttle_pwm_value_in(250));
        run_test(.task_throttle_pwm_value_in(0));
        run_test(.task_throttle_pwm_value_in(250));
        
        $display("%t: %m Test with 2 PWM cycle of 0 input, throttle should cut off after second 0 input PWM", $time);
        run_test(.task_throttle_pwm_value_in(250));
        run_test(.task_throttle_pwm_value_in(0));
        run_test(.task_throttle_pwm_value_in(0));
        run_test(.task_throttle_pwm_value_in(250));
        
        $display("%t: %m Test complete", $time);
        $stop;
        $display("\n\n\n%t: %m Throttle to 250, MAX",$time);
        
        for(i = 0; i < 64; i=i+1) begin
            //$display("%t: %m iteration %d",$time, i);
            run_test(.task_throttle_pwm_value_in(250));
        end
        

        //$display("\n\n\n%t: %m Throttle to 10, just above idle",$time);
        
        for(i = 0; i < 64; i=i+1) begin
            //$display("%t: %m iteration %d",$time, i);
            run_test(.task_throttle_pwm_value_in(10));
        end
        
        //$display("\n%t: %m Throttle to 250, MAX",$time);
        
        for(i = 0; i < 64; i=i+1) begin
            //$display("%t: %m iteration %d",$time, i);
            run_test(.task_throttle_pwm_value_in(250));
        end
        //$display("%t: %m Throttle to 0, idle",$time);
        
        for(i = 0; i < 64; i=i+1) begin
            //$display("%t: %m iteration %d",$time, i);
            run_test(.task_throttle_pwm_value_in(0));
        end
        
        
        //$display("\n\n\n%t: %m Noisy throttle input +/- 10 of 125",$time);
        
        //$display("%t: %m Throttle to 125, MID",$time);
        
        for(i = 0; i < 64; i=i+1) begin
            //$display("%t: %m iteration %d",$time, i);
            run_test(.task_throttle_pwm_value_in(125));
        end
        
        for(i = 0; i < 64; i=i+1) begin
            //$display("%t: %m iteration %d",$time, i);
            run_test(.task_throttle_pwm_value_in(135));
            //$display("%t: %m",$time);
            run_test(.task_throttle_pwm_value_in(115));
        end
        
        
        for(i = 0; i < 64; i=i+1) begin
            //$display("%t: %m iteration %d",$time, i);
            run_test(.task_throttle_pwm_value_in(250*0.40));
        end
        
        for(i = 0; i < 64; i=i+1) begin
            //$display("%t: %m iteration %d",$time, i);
            run_test(.task_throttle_pwm_value_in(250*0.60));
        end

        $display("%t: %m Test complete", $time);
        $stop;
    end

endmodule

