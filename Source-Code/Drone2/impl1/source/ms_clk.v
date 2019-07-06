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
 * ms_clk - Convert from us_clk to ms_clk (1 kHz and 1 ms period).
 */

`timescale 1ns / 1ns

module ms_clk #(parameter CLK_CONVERSION_HIGH = 500,
                parameter CLK_CONVERSION_LOW = 1000)
               (output reg ms_clk,
                input wire us_clk,
                input wire resetn);

    reg [9:0] us_clk_counter;

    always @(posedge us_clk or negedge resetn) begin
        if (!resetn) begin
            ms_clk <= 1'b1;
            us_clk_counter <= 10'd0;
        end
        else if (us_clk_counter == CLK_CONVERSION_HIGH) begin
            ms_clk <= 1'b0;
            us_clk_counter <= us_clk_counter + 1'b1;
        end
        else if (us_clk_counter == CLK_CONVERSION_LOW) begin
            ms_clk <= 1'b1;
            us_clk_counter <= 10'd0;
        end
        else
            us_clk_counter <= us_clk_counter + 1'b1;
    end
endmodule
