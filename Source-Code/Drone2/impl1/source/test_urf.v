`timescale 100ns / 100ns
`default_nettype none
`include "common_defines.v"

`define DEBUG_ENABLE
`undef DEBUG_ENABLE

module test_urf;
    wire urf_trigger;
    wire [9:0] urf_range;
    wire urf_active_signal;
    wire urf_complete_signal;
    wire urf_valid;
    wire us_clk;
    wire sys_clk;
    reg  urf_echo = 0;
    reg  resetn   = 0;
    
    /**
     * Generate System Clock
     */
    defparam OSCH_inst.NOM_FREQ = "38.00";
    OSCH OSCH_inst (
        .STDBY(1'b0),
        .OSC(sys_clk),
        .SEDSTDBY()
    );

    /**
     * Then scale system clock down to 1 microsecond
     *        file - us_clk.v
     */
    us_clk us_clk_divider (
        .us_clk(us_clk),
        .sys_clk(sys_clk),
        .resetn(resetn)
    );

        
    ultrasonic_range_finder URF (
        .urf_range(urf_range),
        .active_signal(urf_active_signal),
        .complete_signal(urf_complete_signal),
        .urf_trigger_out(urf_trigger),
        .urf_valid(urf_valid),
        .urf_echo_in(urf_echo),
        .resetn(resetn),
        .us_clk(us_clk)
    );
    
    task test_distance;
        input real task_echo_high_us;
        integer task_expected_range;
        task_expected_range = $floor(task_echo_high_us/58.0);
        begin
            $display("=================================================");
            $display("%t: %m Testing URF echo %.0fus, expect %dcm", $time, task_echo_high_us, task_expected_range);
            @(posedge urf_active_signal);
            @(negedge urf_trigger);
            repeat(1)@(posedge us_clk);
            urf_echo = 1;
            repeat(task_echo_high_us)@(posedge us_clk);
            urf_echo = 0;
            @(posedge urf_complete_signal);
            if(urf_valid && (urf_range != task_expected_range)) begin
                $display("%t: %m ERROR: Calculation error", $time);
                $display("%t: %m high_us end time = %dus, high_us_start = %dus", $time, URF.high_us_start+URF.high_us_diff, URF.high_us_start);
                $display("%t: %m Module internally calculated time delta = %dus", $time, URF.high_us_diff);
                $display("%t: %m Expected range = %dcm", $time, task_expected_range);
                $display("%t: %m Locally calculated range = %dcm", $time, (URF.high_us_diff)/18'd58);
                $display("%t: %m Calculated range = %dcm", $time, urf_range);
                $stop;
            end
            else if(~urf_valid) begin
                $display("%t: %m Warning: URF valid signal indicates calculation is invalid", $time);
                $display("%t: %m high_us end time = %dus, high_us_start = %dus", $time, URF.high_us_start+URF.high_us_diff, URF.high_us_start);
                $display("%t: %m Module internally calculated time delta = %dus", $time, URF.high_us_diff);
                $display("%t: %m Expected range = %dcm", $time, task_expected_range);
                $display("%t: %m Locally calculated range = %dcm", $time, (URF.high_us_diff)/18'd58);
                $display("%t: %m Calculated range = %dcm", $time, urf_range);
            end
            else begin
                $display("%t: %m URF valid signal indicates calculation is valid", $time);
                $display("%t: %m Calculated range = %dcm", $time, urf_range);
            end
        end
    endtask
    
`ifdef DEBUG_ENABLE
    always@(posedge urf_valid)
        $display("%t: %m URF data became valid", $time);
        
    always@(negedge urf_valid)
        $display("%t: %m URF data became invalid", $time);
        
    always@(posedge urf_active_signal)
        $display("%t: %m URF active asserted", $time);
        
    always@(negedge urf_active_signal)
        $display("%t: %m URF active de-asserted", $time);
        
    always@(posedge urf_complete_signal)
        $display("%t: %m URF complete asserted", $time);
        
    always@(negedge urf_complete_signal)
        $display("%t: %m URF complete de-asserted", $time);

    always@(posedge urf_trigger)
        $display("%t: %m URF trigger output asserted", $time);
        
    always@(negedge urf_trigger)
        $display("%t: %m URF trigger output de-asserted", $time);
`endif
        
    initial begin
        $display("%t: %m Reset URF module", $time);
        resetn = 1;
        urf_echo = 0;
        #2;
        resetn = 0;
        #10;
        resetn = 1;
        #2;
        
        $display("%t: %m URF Reset", $time);
        test_distance(115);     //Just before 2cm (Smallest value)
        test_distance(116);     //2cm
        test_distance(117);     //just after 2cm
        test_distance(58*3);
        test_distance(58*10);
        test_distance(58*11);
        test_distance(58*12);
        test_distance(58*13);
        test_distance(58*14);
        test_distance(58*15);
        test_distance(58*16);
        test_distance(58*17);
        test_distance(1022);  //Find error at count=1024 (Exceeded 10 bits)
        test_distance(1023);
        test_distance(1024);
        test_distance(1025);
        test_distance(58*18);
        test_distance(58*19);
        test_distance(58*20);
        test_distance(58*30);
        test_distance(58*40);
        test_distance(58*50);
        test_distance(58*100);
        // Test near maximum value
        test_distance((58*400)-1); //Just prior to 400cm
        test_distance((58*400));   //400cm
        test_distance((58*400)+1); //just beyond 400cm
        test_distance((58*401)-1); //last good time value for 400cm
        test_distance(58*401);     //first error value for 401cm
        test_distance(40_000);     //Test error handling at 40ms, ~half of polling interval, should go to error state
        test_distance(80_000);     //Test error handling at 80ms, full polling interval, should also go to error state
        test_distance(90_000);     //Test error handling at 90ms, full polling interval and 10ms, should also go to error state
        test_distance(58*10);      //Then go to a good 10cm measurement following error
        test_distance(58*10);      //Repeat 10cm measurement
        @(posedge us_clk);
        $display("=================================================");
        $display("%t: %m All tests complete", $time);
        $stop;
    end
endmodule