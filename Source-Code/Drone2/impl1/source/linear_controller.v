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
 *	Linear Controller - 
 *
 *	Inputs:
 *	@
 *
 *	Outputs:
 *	@
 *
 */
 
 module linear_controller
	(// Outputs
	output reg signed [`RATE_BIT_WIDTH-1:0]	linear_error_z_axis,
	// Debug
	output reg [`RATE_BIT_WIDTH-1:0]	DEBUG_WIRE,
	// Inputs
	input wire us_clk,
	input wire resetn,
	input wire [`RATE_BIT_WIDTH-1:0]	throttle_val,
	input wire [`ALTIMETER_20BIT_WIDTH-1:0]	altitude,
	input wire signed [`ALTIMETER_20BIT_WIDTH-1:0]	altitude_delta
	);

	localparam 
		THROTTLE_F_PAD		= 4'b0,
		THROTTLE_R_PAD		= 4'b0;

	localparam LC_STATE_BIT_WIDTH = 4'd8;
	wire [LC_STATE_BIT_WIDTH-1:0]
		STARTUP	= 8'b00000001,
		CALC1	= 8'b00000010,
		CALC2	= 8'b00000100,
		CALC3	= 8'b00001000;
	reg [LC_STATE_BIT_WIDTH-1:0] state;
	
	localparam [`ALTIMETER_20BIT_WIDTH-1:0] ZERO_INIT_ALTITUDE = 0;

	reg	init_altitude_flag;
	reg [`RATE_BIT_WIDTH-1:0]			latched_throttle;
	reg [`RATE_BIT_WIDTH-1:0]			throttle_rate;
	reg [`RATE_BIT_WIDTH-1:0]			altitude_rate;	
	reg	[`ALTIMETER_20BIT_WIDTH-1:0]	latched_curr_altitude;
	reg	[`ALTIMETER_20BIT_WIDTH-1:0]	init_altitude_val;
	reg	[`ALTIMETER_20BIT_WIDTH-1:0]	last_altitude;

	always @(posedge us_clk or negedge resetn) begin
		if (!resetn) begin
			init_altitude_flag	<= `FALSE;
			init_altitude_val	<= 20'd0;
			latched_throttle	<= 16'd0;
			last_altitude 		<= 16'd0;
			altitude_rate		<= 16'd0;
			state				<= STARTUP;
			end
		else	begin
			case(state)
				STARTUP: begin
					init_altitude_val		<= altitude;
					last_altitude			<= 16'd0;
					latched_throttle		<= 16'd0;
					state					<= CALC1;
					end
				CALC1:	begin
					latched_curr_altitude	<= altitude;
					latched_throttle		<= throttle_val;
					state					<= CALC2;
					end
				CALC2:	begin
					altitude_rate			<= latched_curr_altitude - last_altitude;
					throttle_rate	 		<= {THROTTLE_F_PAD, throttle_val, THROTTLE_R_PAD};
					state					<= CALC3;
					end
				CALC3:	begin
					last_altitude			<= latched_curr_altitude;
					linear_error_z_axis		<= throttle_rate - altitude_rate;
					state					<= CALC1;
					end
				endcase
			end
		end
endmodule