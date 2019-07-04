/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

`timescale 1ns / 1ns
`default_nettype none

module test_uart_top;
    reg  resetn;
    wire sys_clk;
    reg  start;
    wire sout;
    wire txrdy_n;
    reg  sin;
    wire rxrdy_n;
    reg  dcd_n;
    reg  cts_n;
    reg  dsr_n;
    reg  ri_n ;
    wire dtr_n;
    wire rts_n;
    

    defparam OSCH_inst.NOM_FREQ = "38.00";
    OSCH OSCH_inst (.STDBY(1'b0),
                       .OSC(sys_clk),
                       .SEDSTDBY());

    uart_top DUT (
        .resetn(resetn),
        .clk(sys_clk),
        .start(start),
        .sout(sout),
        .txrdy_n(txrdy_n),
        .sin(sin),
        .rxrdy_n(rxrdy_n)
        //.dcd_n(dcd_n ),
        //.cts_n(cts_n ),
        //.dsr_n(dsr_n ),
        //.ri_n (ri_n  ),
        //.dtr_n(dtr_n ),
        //.rts_n(rts_n )
    );

    initial begin
        $display("%t: %m Reset angle controller", $time);
        resetn = 1;
        start = 0;
        #10 resetn = 0;
        #10 resetn = 1;
        start = 1;
        @(posedge txrdy_n)
        start = 0;
        #20_000_000;
        #20_000_000;
        $display("%t: %m Test complete", $time);
        $stop;
    end

endmodule

