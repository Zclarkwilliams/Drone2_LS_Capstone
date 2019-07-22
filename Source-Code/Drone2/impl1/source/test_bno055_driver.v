`timescale 1ns / 1ns
`default_nettype none
`include "common_defines.v"

/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

module bno055_module_tb();
    wire scl_1;
    wire sda_1;
    wire scl_2;
    wire sda_2;
    reg resetn;
    reg purn;
    wire resetn_imu;
    wire [7:0] data_rx;
    wire sys_clk;
    wire done;
    reg go;
    reg read_write_in = 1;
    reg [3:0] i2c_count;
    reg i2c_ack;
    reg [7:0]sda_byte;
    reg next_mod_active;
    reg next_mod_active_cmd;
    wire valid_strobe;

    integer i;
    integer j;


    GSR GSR_INST (.GSR (resetn));
    PUR PUR_INST (.PUR (purn));

    defparam OSCH_inst.NOM_FREQ = "38.00";
    OSCH OSCH_inst (.STDBY(1'b0),
                    .OSC(sys_clk),
                    .SEDSTDBY());


    bno055_driver #(30) bno055(
        .scl_1(scl_1),
        .sda_1(sda_1),
        .scl_2(scl_2),
        .sda_2(sda_2),
        .resetn( (resetn) ),
        .resetn_imu(resetn_imu),
        .next_mod_active(next_mod_active),
        .valid_strobe(valid_strobe),
        .led_data_out(data_rx),
        .sys_clk(sys_clk)
        ); /* synthesis syn_hier=hard */;

// Generate a slave ACK every 9 i2c SCL posedges, regardless of what data is on the bus
    always@(posedge scl_1, negedge resetn) begin
        if(~resetn) begin
            i2c_count = 1'b0;
            i2c_ack = 1'b0;
        end
        else begin
            i2c_count = i2c_count + 1'b1;
            if(i2c_count == 4'd9) begin
                i2c_ack = 1'b1;
                #100
                i2c_ack = 1'b0;
                i2c_count = 1'b0;
                sda_byte = 8'b0;
            end
            else begin
                i2c_ack = 1'b0;
                sda_byte = {sda_byte[6:0], sda_1};
            end
        end
    end

    always@(posedge next_mod_active_cmd, negedge resetn) begin
        if (~resetn)
            next_mod_active <= `LOW;
        else begin
            next_mod_active <= `LOW;
            #1000;
            next_mod_active <= `HIGH;
            #100;
            next_mod_active <= `LOW;
            #1;
        end
    end

    always@(posedge sys_clk, negedge resetn) begin
        if (~resetn)
            next_mod_active_cmd <= `LOW;
        else if(valid_strobe && (~next_mod_active_cmd))
            next_mod_active_cmd <= `HIGH;
        else if(valid_strobe && (next_mod_active_cmd))
            next_mod_active_cmd <= `LOW;
        else
            next_mod_active_cmd <= `LOW;


    end

    assign ( pull1, strong0 ) scl_1 = 1'b1;
    assign ( pull1, strong0 ) sda_1 = i2c_ack ? 1'b0: 1'b1;
    initial begin
        resetn = 1;
        #10 resetn = 0;
        #10 resetn = 1;
        read_write_in = 0;
        //for(j = 0; j < 2; j = j + 1) begin
        //    for(i = 0; i < 10; i = i + 1) begin
        //        $display("efb_registers %1d EFB#%1d = %h", i[4:0], (j[4:0]+1), bno055.i2c.efb_registers[i][j]);
        //    end
        //end
        #500_000_000;
        $stop;
        end
endmodule

