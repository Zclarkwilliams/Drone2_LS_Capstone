/**
 * Team:
 *		Drone2 Capstone 2018
 * 			Daniel Christiansen, Zachary Clark-Williams, Brett Creeley, Ethan Grinnell, Kirk Hooper
 * Author:
 *		Brett Creeley
 */

/**
 * pwm_reader.v - Reads a pwm signal between `MIN_PWM_TIME_HIGH_US and `MAX_PWM_TIME_HIGH_US and
 *				  outputs that read time in microseconds
 *
 * Outputs:
 * @pwm_pulse_length_us: time that the last pwm pulse was high for
 *
 * Inputs:
 * @pwm: pwm from an external source
 * @us_clk: clock with a 1us time period
 * @resetn: negative triggered reset signal to put the module in a known good state
 */

`timescale 1ns / 1ns

`include "common_defines.v"

module pwm_reader #(parameter DEFAULT_PWM_TIME_HIGH_US = 1500)
				   (output wire [10:0] pwm_pulse_length_us,
					input wire pwm,
					input wire us_clk,
					input wire resetn);

	// PWM reader FSM states
	localparam [2:0] STATE_INIT   = 3'b000;
	localparam [2:0] STATE_LOW    = 3'b001;
	localparam [2:0] STATE_HIGH   = 3'b010;
	localparam [2:0] STATE_FALL   = 3'b011;
	localparam [2:0] STATE_RISE   = 3'b100;
	localparam [2:0] STATE_BOUNCE = 3'b101;

    /* Keeps track of the what the pwm is in regards to the pwm reader state machine.
	 * This does not follow the pwm signal exactly.  This allows for detecting a
	 * possibly unstable pwm signal.
	 */
	reg pwm_pulse_level_flag;
	// State of the FSM
	reg [2:0] state;

	// Where the pwm reader stores the count of how long the pulse has been high
	/* TODO: produce an error bit when this is outside of the range to some degree
	 *		 (i.e. if it is less than 500us or more than 2500us)
	 */
	reg [14:0] time_high_count;
	// Where the current time high count is stored until the next high pwm pulse
	reg [10:0] time_high_us;

	initial begin
		state = STATE_INIT;
	end

	// 1 always block model FSM to simplify the logic
	always @(posedge us_clk or negedge resetn) begin
		if (resetn == 0) begin
			state <= STATE_INIT;
			time_high_count <= 0;
			pwm_pulse_level_flag <= `LOW;
			time_high_us <= DEFAULT_PWM_TIME_HIGH_US;
		end
		else begin
			case (state)
				STATE_INIT: begin
					state <= STATE_LOW;
					time_high_count <= 0;
					pwm_pulse_level_flag <= `LOW;
					time_high_us <= DEFAULT_PWM_TIME_HIGH_US;
				end
				STATE_LOW: begin
					// time_high_us only changes on a falling edges
					time_high_us <= time_high_us;

					// The pwm pulse is already low so stay here
					if (pwm == `LOW && pwm_pulse_level_flag == `LOW) begin
						state <= STATE_LOW;
						pwm_pulse_level_flag <= pwm_pulse_level_flag;
						time_high_count <= 0;
					end
					// The pwm pulse was high so start the falling edge
					else if (pwm == `LOW && pwm_pulse_level_flag == `HIGH) begin
						state <= STATE_FALL;
						pwm_pulse_level_flag <= `LOW;
						time_high_count <= time_high_count;
					end
					// The current pwm input is high so start transitioning high
					else begin
						state <= STATE_BOUNCE;
						pwm_pulse_level_flag <= pwm_pulse_level_flag;
						time_high_count <= time_high_count;
					end
				end
				STATE_HIGH: begin
					// time_high_us only changes on falling edges
					time_high_us <= time_high_us;

					// The pwm pulse was already high so stay here and keep counting
					if (pwm == `HIGH && pwm_pulse_level_flag == `HIGH) begin
						state <= STATE_HIGH;
						pwm_pulse_level_flag <= pwm_pulse_level_flag;
						time_high_count <= time_high_count + 1'b1;
					end
					// The pwm pulse was low so start the rising edge
					else if (pwm == `HIGH && pwm_pulse_level_flag == `LOW) begin
						state <= STATE_RISE;
						pwm_pulse_level_flag <= `HIGH;
						time_high_count <= 0;
					end
					// The current pwm input is low so start transitioning low
					else begin
						state <= STATE_BOUNCE;
						pwm_pulse_level_flag <= pwm_pulse_level_flag;
						time_high_count <= time_high_count;
					end
				end
				// Falling edge so propegate new pwm pulse length to the output
				STATE_FALL: begin
						state <= STATE_LOW;
						pwm_pulse_level_flag <= pwm_pulse_level_flag;
						time_high_count <= 0;
						// Make sure we output a known good value
						if (time_high_count < `MIN_PWM_TIME_HIGH_US)
							time_high_us <= `MIN_PWM_TIME_HIGH_US;
						else if (time_high_count > `MAX_PWM_TIME_HIGH_US)
							time_high_us <= `MAX_PWM_TIME_HIGH_US;
						else
							time_high_us <= time_high_count;
				end
				// Rising edge so get ready to start counting the pwm pulse length
				STATE_RISE: begin
						state <= STATE_HIGH;
						pwm_pulse_level_flag <= pwm_pulse_level_flag;
						time_high_us <= time_high_us;
						time_high_count <= 0;
				end
				// Start transitioning in the direction of the current pwm input
				STATE_BOUNCE: begin
						time_high_us <= time_high_us;
						pwm_pulse_level_flag <= pwm_pulse_level_flag;
						time_high_count <= time_high_count;
						state <= (pwm == `LOW) ? STATE_LOW : STATE_HIGH;
				end
				// Error state, should never make it here
				default: begin
					state <= STATE_INIT;
					pwm_pulse_level_flag <= `LOW;
					time_high_us <= DEFAULT_PWM_TIME_HIGH_US;
					time_high_count <= 0;
				end
			endcase
		end
	end

	// Keep outputing the current time_high_us
	assign pwm_pulse_length_us = time_high_us;

endmodule
