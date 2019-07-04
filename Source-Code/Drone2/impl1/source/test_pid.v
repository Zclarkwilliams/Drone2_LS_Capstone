/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

module test_pid;

    localparam RATE_BIT_WIDTH     = 36;
    localparam VELOCITY_BIT_WIDTH = 36;
    localparam ROTATION_BIT_WIDTH = 36;

    // rates represented in 2's complement fixed point
    wire [RATE_BIT_WIDTH-1:0] yaw_rate_out;
    wire [RATE_BIT_WIDTH-1:0] roll_rate_out;
    wire [RATE_BIT_WIDTH-1:0] pitch_rate_out;

    // rates represented in 2's complement fixed point
    wire [RATE_BIT_WIDTH-1:0] yaw_rate_in;
    wire [RATE_BIT_WIDTH-1:0] roll_rate_in;
    wire [RATE_BIT_WIDTH-1:0] pitch_rate_in;

    // velocities represented in 2's complement fixed point
    wire [VELOCITY_BIT_WIDTH-1:0] x_velocity;
    wire [VELOCITY_BIT_WIDTH-1:0] y_velocity;
    wire [VELOCITY_BIT_WIDTH-1:0] z_velocity;

    // rotations represented in 2's complement fixed point
    wire [ROTATION_BIT_WIDTH-1:0] x_rotation;
    wire [ROTATION_BIT_WIDTH-1:0] y_rotation;
    wire [ROTATION_BIT_WIDTH-1:0] z_rotation;

    wire clk;

    // line up the parameters here to the ones internal to the receiver module
    pid #(RATE_BIT_WIDTH, VELOCITY_BIT_WIDTH, ROTATION_BIT_WIDTH) DUT (.yaw_rate_out(yaw_rate_out),
                                                                       .roll_rate_out(roll_rate_out),
                                                                       .pitch_rate_out(pitch_rate_out),
                                                                       .yaw_rate_in(yaw_rate_in),
                                                                       .roll_rate_in(roll_rate_in),
                                                                       .pitch_rate_in(pitch_rate_in),
                                                                       .x_velocity(x_velocity),
                                                                       .y_velocity(y_velocity),
                                                                       .z_velocity(z_velocity),
                                                                       .x_rotation(x_rotation),
                                                                       .y_rotation(y_rotation),
                                                                       .z_rotation(z_rotation),
                                                                       .sys_clk(sys_clk));

    initial begin
        $display("%m successful");
    end

endmodule

