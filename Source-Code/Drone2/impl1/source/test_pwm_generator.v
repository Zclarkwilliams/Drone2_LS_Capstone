module test_pwm_generator;
// pwm_generator module testbench

`timescale 1ns/1ns

/*
 * Tested:
 * Basic functionality
 *
 * To test:
 * Mid-period input value change
 * 0 input value
 * >250 input value
 */



reg [7:0] m_1_rate, m_2_rate, m_3_rate, m_4_rate;
wire m_1_pwm, m_2_pwm, m_3_pwm, m_4_pwm;
reg rst, clk;

// TODO: Fix these tests because the parameters don't match up
pwm_generator
	#(.INPUT_BIT_WIDTH(8)/*,
	.CLK_CONVERSION_HIGH(2),
	.CLK_CONVERSION_LOW(4)*/
	)
	pwm_dut
	(.motor_1_pwm(m_1_pwm),
	.motor_2_pwm(m_2_pwm),
	.motor_3_pwm(m_3_pwm),
	.motor_4_pwm(m_4_pwm),
	.motor_1_rate(m_1_rate),
	.motor_2_rate(m_2_rate),
	.motor_3_rate(m_3_rate),
	.motor_4_rate(m_4_rate),
	.resetn(rst),
	.us_clk(clk));

initial begin
	m_1_rate <= 0;
	m_2_rate <= 0;
	m_3_rate <= 0;
	m_4_rate <= 0;
end

initial begin
	clk <= 0;
	forever begin
		#1 clk <= ~clk;
	end
end

initial begin
	repeat (2) @(posedge clk);
	rst <= 1'b0;
	m_1_rate <= 8'b00011110;
	repeat (2) @(posedge clk);
	rst <= 1'b1;
	repeat (100) @(posedge clk);
	m_2_rate <= 8'b01010101;
	repeat (160000) @(posedge clk);
	$finish;
end


endmodule
