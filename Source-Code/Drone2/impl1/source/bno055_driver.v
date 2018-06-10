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

 *  Module inouts:
 *  	 	inout scl_1,          I2C Primary   EFB SDA wire - Module only uses Primary or secondary I2C, but can use either one
 *  		inout scl_2,          I2C Secondary EFB SDA wire
 *  		inout sda_1,          I2C Primary   EFB SDA wire
 *  		inout sda_2,          I2C Secondary EFB SDA wire
 
 *  Module takes as inputs:
 *  		sys_clk                 master clock
 *  		ac_active               Handshake signal from angle controller acknowledging the data valid strobe
 *  		rstn                    async negative reset signal 0 = reset, 1 = not reset
 
 * Module provides as output (all values are 16-bit, 2's complement):
 *  		led_data_out,          Module calibration status output for LED indication of IMU operating state      
 *  		rstn_imu               Low active reset signal to IMU hardware to trigger reset
 *  		imu_good               The IMU is either in an error or initial bootup states, measurements not yet active
 *  		valid_strobe           Strobe signal that indicates the end of the data collection poll, subsequent modules key off this strobe.
 *  		accel_rate_x           Accelerometer X-Axis                Precision: 1 m/s^2 = 100 LSB
 *  		accel_rate_y           Accelerometer Y-Axis                Precision: 1 m/s^2 = 100 LSB
 *  		accel_rate_z           Accelerometer Z-Axis                Precision: 1 m/s^2 = 100 LSB
 *  		magneto_rate_x         Magnetometer X-Axis                 Precision: 1uT = 16 LSB
 *  		magneto_rate_y         Magnetometer Y-Axis                 Precision: 1uT = 16 LSB
 *  		magneto_rate_z         Magnetometer Z-Axis                 Precision: 1uT = 16 LSB
 *  		gyro_rate_x            Gyroscope X-Axis                    Precision: Dps = 16 LSB
 *  		gyro_rate_y            Gyroscope Y-Axis                    Precision: Dps = 16 LSB
 *  		gyro_rate_z            Gyroscope Z-Axis                    Precision: Dps = 16 LSB
 *  		euler_angle_x          Euler angle X-Axis                  Precision: Deg = 16 LSB
 *  		euler_angle_y          Euler angle Y-Axis                  Precision: Deg = 16 LSB
 *  		euler_angle_z          Euler angle Z-Axis                  Precision: Deg = 16 LSB
 *  		quaternion_data_w      Quaternion X-Axis                   Precision: Unit = 2^14 LSB
 *  		quaternion_data_x      Quaternion X-Axis                   Precision: Unit = 2^14 LSB
 *  		quaternion_data_y      Quaternion Y-Axis                   Precision: Unit = 2^14 LSB
 *  		quaternion_data_z      Quaternion Z-Axis                   Precision: Unit = 2^14 LSB
 *  		linear_accel_x         Linear Acceleration X-Axis          Precision: 1 m/s^2 = 100 LSB
 *  		linear_accel_y         Linear Acceleration Y-Axis          Precision: 1 m/s^2 = 100 LSB
 *  		linear_accel_z         Linear Acceleration Z-Axis          Precision: 1 m/s^2 = 100 LSB
 *  		gravity_accel_x        Gravitational Acceleration X-Axis   Precision: 1 m/s^2 = 100 LSB
 *  		gravity_accel_y        Gravitational Acceleration Y-Axis   Precision: 1 m/s^2 = 100 LSB
 *  		gravity_accel_z        Gravitational Acceleration Z-Axis   Precision: 1 m/s^2 = 100 LSB
 *  		temperature            Temperature in degrees Celsius      Precision: 1 Deg C = 1 LSB
 *  		calib_status           Calibration status register
 *  		x_velocity             Linear velocity in the X direction, one byte signed integer
 *  		y_velocity             Linear velocity in the Y direction, one byte signed integer
 *  		z_velocity             Linear velocity in the Z direction, one byte signed integer
 *
 * TODO: Add module description with description of inputs/outputs
 * 		instead of with the variable. Make this file look like all of
 * 		the other files.
 */

`timescale 1ns / 1ns
`include "common_defines.v"
`include "bno055_defines.v"

module bno055_driver #(
	parameter INIT_TIME = 15'd650
)
(
 	inout wire scl_1,                  
	inout wire scl_2,                  
	inout wire sda_1,                  
	inout wire sda_2,                  
	input wire rstn,                   
	output wire [7:0]led_data_out,     
	input  wire sys_clk,               
	input wire ac_active,
	output wire rstn_imu,              
	output reg  imu_good,              
	output reg  valid_strobe,          
	output reg [15:0]accel_rate_x,     
	output reg [15:0]accel_rate_y,     
	output reg [15:0]accel_rate_z,     
	output reg [15:0]magneto_rate_x,   
	output reg [15:0]magneto_rate_y,   
	output reg [15:0]magneto_rate_z,   
	output reg [15:0]gyro_rate_x,      
	output reg [15:0]gyro_rate_y,      
	output reg [15:0]gyro_rate_z,      
	output reg [15:0]euler_angle_x,    
	output reg [15:0]euler_angle_y,    
	output reg [15:0]euler_angle_z,    
	output reg [15:0]quaternion_data_w,
	output reg [15:0]quaternion_data_x,
	output reg [15:0]quaternion_data_y,
	output reg [15:0]quaternion_data_z,
	output reg [15:0]linear_accel_x,   
	output reg [15:0]linear_accel_y,   
	output reg [15:0]linear_accel_z,   
	output reg [15:0]gravity_accel_x,  
	output reg [15:0]gravity_accel_y,  
	output reg [15:0]gravity_accel_z,  
	output reg [7:0]temperature,       
	output reg [7:0]calib_status,      
	output reg [15:0]x_velocity,       
	output reg [15:0]y_velocity,       
	output reg [15:0]z_velocity,       
);

	reg  read_write_in, next_read_write_in;           //  Value and next value of signal to i2c module to indicate read or write transaction, 1 = read, 0 = write
	reg  go;                                          //  Flag to i2c module signaling start of i2c transaction. All inputs must be valid before asserting this bit
	reg  next_go_flag;                                //  Next value of the i2c module GO flag
	wire one_byte_ready;                              //  Flag from i2c module indicating that a byte has been received, data_rx is valid
	wire busy;                                        //  Flag from i2c module indicating that a transaction is in progress
	wire [7:0]data_rx;                                //  Receives an RX data byte from i2c module
	reg  [7:0]data_reg;                               //  Wishbone address register
	reg  [7:0]data_tx;                                //  Temp storage of data to be written
	reg  [7:0]next_data_reg;                          //  Command register address
	reg  [7:0]next_data_tx;                           //  Data written to registers for this command
	reg  [6:0]slave_address;                          //  Slave address to access
	reg  [6:0]next_slave_address;                     //  Next value of slave address
	reg  [`BNO055_STATE_BITS-1:0]bno055_state;        //  State for bno055 command sequence FSM
	reg  [`BNO055_STATE_BITS-1:0]next_bno055_state;   //  Next FSM state
	reg  [`BNO055_STATE_BITS-1:0]return_state;        //  FSM return state from i2c sub state
	reg  [`BNO055_STATE_BITS-1:0]next_return_state;   //  Next value for FSM return state
	reg  [31:0]count_ms;                              //  Count from 0 to value determined by clock rate, used to generate N ms delay trigger
	reg  [15:0]wait_ms;                               //  The number of mS to wait in delay loop
	reg  [15:0]next_wait_ms;                          //  The next latched value of wait mS
	reg  clear_waiting_ms;                            //  Reset waiting X ms timer.
	reg  [5:0]target_read_count;                      //  The number of bytes to access for a read command (Writes are always for a single byte)
	reg  [5:0]next_target_read_count;                 //  Next value of target_read_count
	reg  [5:0]data_rx_reg_index;                      //  Index in data_rx_reg for current byte
	reg  [5:0]led_view_index;                         //  Index in data_rx_reg that is being monitored with status LEDs
	reg  [5:0]next_led_view_index;                    //  Next value of LED View Index
	reg  [7:0]data_rx_reg[`DATA_RX_BYTE_REG_CNT-1:0]; //  Store all measurement bytes from i2c read burst
	reg  rstn_buffer;                                 //  Negedge clears received measurement buffer
	reg  rx_data_latch_strobe;                        //  Strobe data output register, latch onto current data in rx buffer, asyncronous latch
	reg  rx_data_latch_tmp; 						  //  Syncronously latched value of the data latch strobe
	reg  next_imu_good;                               //  Next value of module imu_good bit
	reg  i2c_number;								  //  The i2c module to call, 0 = i2c EFB #1, 1 = i2c EFB #2
	reg [7:0]calibration_reg[`CAL_DATA_REG_CNT-1:0];
	reg [7:0]cal_reg_addr;
	reg [5:0]cal_restore_index;
	reg clear_cal_restore_index;
	reg increment_cal_restore_index;
	reg calibrated_once;
	reg next_calibrated_once;
	reg next_valid_strobe_enable;
	reg valid_strobe_enable;
	reg [31:0]master_trigger_count_ms;                //  Counter used to generate a periodic 20ms timer tick.

	//
	//  Module body
	//
	assign led_data_out = (bno055_state <= `BNO055_STATE_BOOT_WAIT ) ? 8'h81 : data_rx_reg[led_view_index]; //  Inverted output for LEDS, since they are low active

	//  Instantiate i2c driver
	i2c_module i2c(	.scl_1(scl_1),
					.sda_1(sda_1),
					.scl_2(scl_2),
					.sda_2(sda_2),
					.rstn(rstn),
					.rstn_imu(rstn_imu),
					.target_read_count(target_read_count),
					.slave_address(slave_address),
					.module_data_out(data_rx),
					.module_data_in(data_tx),
					.module_reg_in(data_reg),
					.read_write_in(read_write_in),
					.go(go),
					.busy(busy),
					.one_byte_ready(one_byte_ready),
					.i2c_number(i2c_number),
					.sys_clk(sys_clk)
	);

	//  Generates a multiple of 1ms length duration delay trigger - Defaulted to 650 ms for BNO055 reset and boot time
	always@(posedge sys_clk, negedge clear_waiting_ms, negedge rstn) begin
		if(~rstn)
			count_ms       <= 32'hFFFFFFFF;
		else if( clear_waiting_ms == `CLEAR_MS_TIMER )
			count_ms       <= (`WAIT_MS_DIVIDER*wait_ms);
		else if( count_ms != 32'hFFFFFFFF )
			count_ms       <= (count_ms - 1'b1);
		else
			count_ms       <= count_ms;
	end

	always@(posedge sys_clk, negedge rstn_buffer, negedge rstn) begin
		if(~rstn) begin
			for(data_rx_reg_index = 0; data_rx_reg_index < `DATA_RX_BYTE_REG_CNT; data_rx_reg_index = data_rx_reg_index+1'b1)
				data_rx_reg[data_rx_reg_index] <= 8'b0;
			data_rx_reg_index <= 0;
		end
		else if(~rstn_buffer ) begin
			data_rx_reg_index <= 0;
		end
		else if (one_byte_ready) begin
			if(data_rx_reg_index == (`DATA_RX_BYTE_REG_CNT - 1'b1)) begin
				data_rx_reg_index              <= 0;
				data_rx_reg[data_rx_reg_index] <= data_rx;
			end
			else begin
				data_rx_reg[data_rx_reg_index] <= data_rx;
				data_rx_reg_index              <= data_rx_reg_index + 1'b1;
			end
		end
	end

	//generate a 20ms timer that sends valid strobe every timer complete interval
	always@(posedge sys_clk, negedge rstn) begin
		if(~rstn) begin
			master_trigger_count_ms <= `WAIT_MS_DIVIDER*20;
			valid_strobe_enable     <= `FALSE;
		end
		else if( master_trigger_count_ms[31] == `TRUE && ~rx_data_latch_strobe) begin
			master_trigger_count_ms <= (`WAIT_MS_DIVIDER*20);
			valid_strobe_enable     <= `TRUE;
		end
		else if( master_trigger_count_ms[31] == `TRUE && rx_data_latch_strobe) begin
			master_trigger_count_ms <= master_trigger_count_ms;
			valid_strobe_enable     <= `FALSE;
		end
		else begin
			master_trigger_count_ms <= (master_trigger_count_ms - 1'b1);
			valid_strobe_enable     <= `FALSE;
		end
	end
	
	always@(posedge sys_clk, negedge rstn) begin
		if(~rstn)
			valid_strobe      <= `LOW;
		else if (valid_strobe_enable == `TRUE) begin
			if(~valid_strobe)                             // Valid not yet asserted
				valid_strobe      <= `HIGH;
			else if( valid_strobe && (~ac_active))        //Hold strobe until AC active
				valid_strobe      <= `HIGH;
			else                                          // De-assert valid strobe
				valid_strobe      <= `LOW;
		end
		else begin
			if( valid_strobe && (~ac_active))             //Hold strobe until AC active
				valid_strobe      <= `HIGH;
			else
				valid_strobe <= `LOW;
		end
	end


	task set_calibration_data_values;
	/*
		Calibration Values for the BNO055 IMU that we are using:

		Accelerometer: -27 -46 38
		Mag: 284 30 -93
		Gyro: -2 -3 0
		Accel Radius: 1000
		Mag Radius: 851
		Accel X MSB:255, X LSB:229, Y MSB:255, Y LSB:210, Z MSB0,    Z LSB:38
		Mag   X MSB:1,   X LSB:28,  Y MSB:0,   Y LSB:30,  Z MSB:255, Z LSB:163
		Gyro  X MSB:255, X LSB:254, Y MSB:255, Y LSB:253, Z MSB:0,   Z LSB:0
		Accel radius MSB:3, LSB:232
		Mag radius   MSB:3, LSB:83
	*/
		begin
			calibration_reg[`ACCEL_OFFSET_X_LSB_INDEX] <= 8'd229;
			calibration_reg[`ACCEL_OFFSET_X_MSB_INDEX] <= 8'd255;
			calibration_reg[`ACCEL_OFFSET_Y_LSB_INDEX] <= 8'd210;
			calibration_reg[`ACCEL_OFFSET_Y_MSB_INDEX] <= 8'd255;
			calibration_reg[`ACCEL_OFFSET_Z_LSB_INDEX] <= 8'd38;
			calibration_reg[`ACCEL_OFFSET_Z_MSB_INDEX] <= 8'd0;
			calibration_reg[`MAG_OFFSET_X_LSB_INDEX  ] <= 8'd28;
			calibration_reg[`MAG_OFFSET_X_MSB_INDEX  ] <= 8'd1;
			calibration_reg[`MAG_OFFSET_Y_LSB_INDEX  ] <= 8'd30;
			calibration_reg[`MAG_OFFSET_Y_MSB_INDEX  ] <= 8'd0;
			calibration_reg[`MAG_OFFSET_Z_LSB_INDEX  ] <= 8'd163;
			calibration_reg[`MAG_OFFSET_Z_MSB_INDEX  ] <= 8'd255;
			calibration_reg[`GYRO_OFFSET_X_LSB_INDEX ] <= 8'd254;
			calibration_reg[`GYRO_OFFSET_X_MSB_INDEX ] <= 8'd255;
			calibration_reg[`GYRO_OFFSET_Y_LSB_INDEX ] <= 8'd253;
			calibration_reg[`GYRO_OFFSET_Y_MSB_INDEX ] <= 8'd255;
			calibration_reg[`GYRO_OFFSET_Z_LSB_INDEX ] <= 8'd0;
			calibration_reg[`GYRO_OFFSET_Z_MSB_INDEX ] <= 8'd0;
			calibration_reg[`ACCEL_RADIUS_LSB_INDEX  ] <= 8'd3;
			calibration_reg[`ACCEL_RADIUS_MSB_INDEX  ] <= 8'd232;
			calibration_reg[`MAG_RADIUS_LSB_INDEX    ] <= 8'd3;
			calibration_reg[`MAG_RADIUS_MSB_INDEX    ] <= 8'd83;
		end
	endtask

	always@(posedge sys_clk, negedge rstn) begin
		if(~rstn) begin
			accel_rate_x      <= 16'b0;
			accel_rate_y      <= 16'b0;
			accel_rate_z      <= 16'b0;
			magneto_rate_x    <= 16'b0;
			magneto_rate_y    <= 16'b0;
			magneto_rate_z    <= 16'b0;
			gyro_rate_x       <= 16'b0;
			gyro_rate_y       <= 16'b0;
			gyro_rate_z       <= 16'b0;
			euler_angle_x     <= 16'b0;
			euler_angle_y     <= 16'b0;
			euler_angle_z     <= 16'b0;
			quaternion_data_w <= 16'b0;
			quaternion_data_x <= 16'b0;
			quaternion_data_y <= 16'b0;
			quaternion_data_z <= 16'b0;
			linear_accel_x    <= 16'b0;
			linear_accel_y    <= 16'b0;
			linear_accel_z    <= 16'b0;
			gravity_accel_x   <= 16'b0;
			gravity_accel_y   <= 16'b0;
			gravity_accel_z   <= 16'b0;
			temperature       <= 8'b0;
			calib_status      <= 8'b0;
			x_velocity        <= 8'b0;
			y_velocity        <= 8'b0;
			z_velocity        <= 8'b0;
			set_calibration_data_values();
		end
		else if(rx_data_latch_strobe) begin
			accel_rate_x      <= {data_rx_reg[`ACC_DATA_X_MSB_INDEX],data_rx_reg[`ACC_DATA_X_LSB_INDEX]};
			accel_rate_y      <= {data_rx_reg[`ACC_DATA_Y_MSB_INDEX],data_rx_reg[`ACC_DATA_Y_LSB_INDEX]};
			accel_rate_z      <= {data_rx_reg[`ACC_DATA_Z_MSB_INDEX],data_rx_reg[`ACC_DATA_Z_LSB_INDEX]};
			magneto_rate_x    <= {data_rx_reg[`MAG_DATA_X_MSB_INDEX],data_rx_reg[`MAG_DATA_X_LSB_INDEX]};
			magneto_rate_y    <= {data_rx_reg[`MAG_DATA_Y_MSB_INDEX],data_rx_reg[`MAG_DATA_Y_LSB_INDEX]};
			magneto_rate_z    <= {data_rx_reg[`MAG_DATA_Z_MSB_INDEX],data_rx_reg[`MAG_DATA_Z_LSB_INDEX]};
			gyro_rate_x       <= {data_rx_reg[`GYR_DATA_X_MSB_INDEX],data_rx_reg[`GYR_DATA_X_LSB_INDEX]};
			gyro_rate_y       <= {data_rx_reg[`GYR_DATA_Y_MSB_INDEX],data_rx_reg[`GYR_DATA_Y_LSB_INDEX]};
			gyro_rate_z       <= {data_rx_reg[`GYR_DATA_Z_MSB_INDEX],data_rx_reg[`GYR_DATA_Z_LSB_INDEX]};
			euler_angle_x     <= {data_rx_reg[`EUL_DATA_X_MSB_INDEX],data_rx_reg[`EUL_DATA_X_LSB_INDEX]};
			euler_angle_y     <= {data_rx_reg[`EUL_DATA_Y_MSB_INDEX],data_rx_reg[`EUL_DATA_Y_LSB_INDEX]};
			euler_angle_z     <= {data_rx_reg[`EUL_DATA_Z_MSB_INDEX],data_rx_reg[`EUL_DATA_Z_LSB_INDEX]};
			quaternion_data_w <= {data_rx_reg[`QUA_DATA_W_MSB_INDEX],data_rx_reg[`QUA_DATA_W_LSB_INDEX]};
			quaternion_data_x <= {data_rx_reg[`QUA_DATA_X_MSB_INDEX],data_rx_reg[`QUA_DATA_X_LSB_INDEX]};
			quaternion_data_y <= {data_rx_reg[`QUA_DATA_Y_MSB_INDEX],data_rx_reg[`QUA_DATA_Y_LSB_INDEX]};
			quaternion_data_z <= {data_rx_reg[`QUA_DATA_Z_MSB_INDEX],data_rx_reg[`QUA_DATA_Z_LSB_INDEX]};
			linear_accel_x    <= {data_rx_reg[`LIN_DATA_X_MSB_INDEX],data_rx_reg[`LIN_DATA_X_LSB_INDEX]};
			linear_accel_y    <= {data_rx_reg[`LIN_DATA_Y_MSB_INDEX],data_rx_reg[`LIN_DATA_Y_LSB_INDEX]};
			linear_accel_z    <= {data_rx_reg[`LIN_DATA_Z_MSB_INDEX],data_rx_reg[`LIN_DATA_Z_LSB_INDEX]};
			gravity_accel_x   <= {data_rx_reg[`GRA_DATA_X_MSB_INDEX],data_rx_reg[`GRA_DATA_X_LSB_INDEX]};
			gravity_accel_y   <= {data_rx_reg[`GRA_DATA_Y_MSB_INDEX],data_rx_reg[`GRA_DATA_Y_LSB_INDEX]};
			gravity_accel_z   <= {data_rx_reg[`GRA_DATA_Z_MSB_INDEX],data_rx_reg[`GRA_DATA_Z_LSB_INDEX]};
			temperature       <= data_rx_reg[`TEMPERATURE_DATA_INDEX];
			calib_status      <= data_rx_reg[`CALIBRATION_DATA_INDEX];
			// Set these to 0 for now, just to have something connected, need to make it a velocity later
			x_velocity        <= 8'b0;
			y_velocity        <= 8'b0;
			z_velocity        <= 8'b0;
			set_calibration_data_values();
		end
	end

	//Increment calibration restore index and calibration register address by 1 or clear back to initial values
	always@(posedge sys_clk) begin
		if( ~clear_cal_restore_index) begin
			cal_reg_addr      <= `BNO055_ACCEL_OFFSET_X_LSB_ADDR;
			cal_restore_index <= 0;
		end
		else if( increment_cal_restore_index) begin
			cal_reg_addr      <= (cal_reg_addr      + 1'b1);
			cal_restore_index <= (cal_restore_index + 1'b1);
		end
		else begin
			cal_reg_addr      <= cal_reg_addr;
			cal_restore_index <= cal_restore_index;
		end
	end


	//  Advance state and registered data at each positive clock edge
	always@(posedge sys_clk, negedge rstn) begin
		if(~rstn) begin
			data_reg            <= `BYTE_ALL_ZERO;
			data_tx             <= `BYTE_ALL_ZERO;
			read_write_in       <= `I2C_READ;
			go                  <= `NOT_GO;
			bno055_state        <= `BNO055_STATE_RESET;
			return_state        <= `FALSE;
			target_read_count   <= `FALSE;
			led_view_index      <= `FALSE;
			wait_ms             <= INIT_TIME; // Reset to Normal takes 650 ms for BNO055;
			slave_address       <= next_slave_address;
			imu_good            <= `FALSE;
			calibrated_once     <= `FALSE;
			rx_data_latch_tmp   <= `LOW;
		end
		else begin
			data_reg            <= next_data_reg;
			data_tx             <= next_data_tx;
			read_write_in       <= next_read_write_in;
			go                  <= next_go_flag;
			bno055_state        <= next_bno055_state;
			return_state        <= next_return_state;
			target_read_count   <= next_target_read_count;
			led_view_index      <= next_led_view_index;
			wait_ms             <= next_wait_ms;
			slave_address       <= next_slave_address;
			imu_good            <= next_imu_good;
			calibrated_once     <= next_calibrated_once;
			rx_data_latch_tmp   <= rx_data_latch_strobe;
		end
	end


	//  Determine next state of FSM and drive i2c module inputs
	always@(*) begin
		if( ~(rstn & rstn_imu) ) begin
			next_imu_good             = `FALSE;
			clear_waiting_ms          = `RUN_MS_TIMER;
			next_bno055_state         = `BNO055_STATE_RESET;
			next_return_state         = `BNO055_STATE_RESET;
			next_go_flag              = `NOT_GO;
			next_data_reg             = `BYTE_ALL_ZERO;
			next_data_tx              = `BYTE_ALL_ZERO;
			next_read_write_in        = `I2C_READ;
			next_led_view_index       = `FALSE;
			next_wait_ms              = INIT_TIME; // Reset to Normal takes 650 ms for BNO055
			rstn_buffer               = `LOW;
			next_target_read_count    = 1'b1;
			rx_data_latch_strobe      = `LOW;
			i2c_number 			      = 1'b0; // Default to i2c EFB #1
			next_slave_address        = slave_address;
			increment_cal_restore_index = 1'b0;
			clear_cal_restore_index     = 1'b0;
		end
		else begin
			// Default to preserve these values, can be altered in lower steps
			next_imu_good             = imu_good;
			clear_waiting_ms          = `RUN_MS_TIMER;
			next_go_flag              = `NOT_GO;
			next_bno055_state         = bno055_state;
			next_return_state         = return_state;
			next_data_reg             = data_reg;
			next_data_tx              = data_tx;
			next_read_write_in        = read_write_in;
			next_led_view_index       = led_view_index;
			next_wait_ms              = wait_ms;
			rstn_buffer               = `HIGH;
			next_target_read_count    = target_read_count;
			rx_data_latch_strobe      = `LOW;
			i2c_number 			      = 1'b0; // Default to i2c EFB #1
			next_slave_address        = `BNO055_SLAVE_ADDRESS;
			next_calibrated_once      = calibrated_once;
			increment_cal_restore_index = 1'b0;
			clear_cal_restore_index     = 1'b1;
			case(bno055_state)
				`BNO055_STATE_RESET: begin
					next_imu_good      = `FALSE;
					clear_waiting_ms   = `CLEAR_MS_TIMER; //  Clear and set to wait_ms value
					next_bno055_state  = `BNO055_STATE_BOOT;
					next_slave_address = `BNO055_SLAVE_ADDRESS;
					clear_cal_restore_index = 1'b0;
					next_calibrated_once    = 1'b0;
				end
				`BNO055_STATE_BOOT: begin
					next_imu_good      = `FALSE;
					clear_waiting_ms   = `RUN_MS_TIMER;
					next_bno055_state  = `BNO055_STATE_BOOT_WAIT;
					next_slave_address = `BNO055_SLAVE_ADDRESS;
				end
				`BNO055_STATE_BOOT_WAIT: begin
					next_imu_good      = `FALSE;
					clear_waiting_ms   = `RUN_MS_TIMER;
					next_bno055_state  = `BNO055_STATE_BOOT_WAIT;
					next_slave_address = `BNO055_SLAVE_ADDRESS;
					if((~busy) && (count_ms[31] == 1'b1) ) // Wait for i2c to be in not busy state and count_ms wrapped around to 0x3FFFFFF
						next_bno055_state = `BNO055_STATE_READ_CHIP_ID;
				end
				`BNO055_STATE_READ_CHIP_ID: begin //  Page 0
					next_imu_good          = `FALSE;
					next_slave_address     = `BNO055_SLAVE_ADDRESS;
					next_go_flag           = `NOT_GO;
					next_bno055_state      = `BNO055_SUB_STATE_START;
					next_return_state      = `BNO055_STATE_SET_UNITS;
					next_data_reg          = `BNO055_CHIP_ID_ADDR;
					next_data_tx           = `BYTE_ALL_ZERO;
					next_read_write_in     = `I2C_READ;
					next_target_read_count = 1'b1;
					next_led_view_index    = 1'b0;
				end
				`BNO055_STATE_SET_UNITS: begin //  Page 0
					next_imu_good      = `FALSE;
					next_slave_address = `BNO055_SLAVE_ADDRESS;
					next_go_flag       = `NOT_GO;
					next_bno055_state  = `BNO055_SUB_STATE_START;
					next_return_state  = `BNO055_STATE_SET_POWER_MODE;
					next_data_reg      = `BNO055_UNIT_SEL_ADDR;
					// This line Modified from Adafruit Bosch BNO055 Arduino driver code, downloaded from: https://github.com/adafruit/Adafruit_BNO055
					next_data_tx       = ((1 << 7) |  // Orientation = Windows - Range (Windows format) -180° to +180° corresponds with turning clockwise and increases values
										 ( 0 << 4) |  // Temperature = Celsius
										 ( 0 << 2) |  // Euler = Degrees
										 ( 0 << 1) |  // Gyro = Degrees/Sec
										 ( 0 << 0));  // Accelerometer = m/s^2;
					next_read_write_in = `I2C_WRITE;
				end
				`BNO055_STATE_SET_POWER_MODE: begin //  Page 0
					next_imu_good      = `FALSE;
					next_slave_address = `BNO055_SLAVE_ADDRESS;
					clear_waiting_ms   = `RUN_MS_TIMER;
					next_go_flag       = `NOT_GO;
					next_bno055_state  = `BNO055_SUB_STATE_START;
					next_return_state  = `BNO055_STATE_CAL_RESTORE_DATA;
					next_data_reg      = `BNO055_PWR_MODE_ADDR;
					next_data_tx       = `BNO055_POWER_MODE_NORMAL;
					next_read_write_in = `I2C_WRITE;
				end
				`BNO055_STATE_CAL_RESTORE_DATA: begin
					next_imu_good      = `FALSE;
					next_go_flag       = `NOT_GO;
					next_slave_address = `BNO055_SLAVE_ADDRESS;
					next_data_reg      = cal_reg_addr;
					next_read_write_in = `I2C_WRITE;
					next_data_tx       = calibration_reg[cal_restore_index];
					next_bno055_state  = `BNO055_STATE_CAL_RESTORE_START;
				end
				`BNO055_STATE_CAL_RESTORE_START: begin
					next_imu_good      = `FALSE;
					next_go_flag       = `GO;
					next_slave_address = slave_address;
					next_data_reg      = data_reg;
					next_data_tx       = data_tx;
					next_read_write_in = read_write_in;
					if(busy)
						next_bno055_state = `BNO055_STATE_CAL_RESTORE_WAIT;
					else
						next_bno055_state = `BNO055_STATE_CAL_RESTORE_START;
				end
				`BNO055_STATE_CAL_RESTORE_WAIT: begin // Wait until send completes
					next_imu_good      = `FALSE;
					next_go_flag       = `NOT_GO;
					next_slave_address = slave_address;
					next_data_reg      = data_reg;
					next_data_tx       = data_tx;
					next_read_write_in = read_write_in;
					if(~busy) begin
						increment_cal_restore_index = 1'b1;
						next_bno055_state = `BNO055_STATE_CAL_RESTORE_STOP;
					end
					else
						next_bno055_state = `BNO055_STATE_CAL_RESTORE_WAIT;
				end
				`BNO055_STATE_CAL_RESTORE_STOP: begin // See if this was the last, loop around if more, otherwise, exit loop
					next_imu_good      = `FALSE;
					next_go_flag       = `NOT_GO;
					next_slave_address = slave_address;
					next_data_reg      = data_reg;
					next_data_tx       = data_tx;
					next_read_write_in = read_write_in;
					if(cal_restore_index >= (`CAL_DATA_REG_CNT)) begin
						clear_cal_restore_index = 1'b0;
						next_bno055_state       = `BNO055_STATE_CAL_RESTORE_AGAIN;
					end
					else begin
						next_bno055_state  = `BNO055_STATE_CAL_RESTORE_DATA;
					end
				end
				`BNO055_STATE_CAL_RESTORE_AGAIN: begin // Restore calibration two times, to ensure that one calibration parameter doesn't need to be written before another.
					next_imu_good        = `FALSE;
					next_go_flag         = `NOT_GO;
					next_calibrated_once = 1'b1;
					if(calibrated_once == 1'b1)
						next_bno055_state = `BNO055_STATE_SET_EXT_CRYSTAL;
					else
						next_bno055_state = `BNO055_STATE_CAL_RESTORE_DATA;
				end
				`BNO055_STATE_SET_EXT_CRYSTAL: begin //  Has to be done after caliubration restore, for some odd reason not documented in IMU docs
					next_imu_good      = `FALSE;
					next_slave_address = `BNO055_SLAVE_ADDRESS;
					next_go_flag       = `NOT_GO;
					next_bno055_state  = `BNO055_SUB_STATE_START;
					next_return_state  = `BNO055_STATE_SET_RUN_MODE;
					next_data_reg      = `BNO055_SYS_TRIGGER_ADDR;
					next_data_tx       = 8'h80; //Enable external crystal, set bit 7 to 1'b1
					next_read_write_in = `I2C_WRITE;
					next_wait_ms       = 12'd20; //  used in change run mode state, but set here
				end
				`BNO055_STATE_SET_RUN_MODE: begin // Change to run mode, changing run mode takes 7 to 19 ms depending on modes
					next_imu_good      = `FALSE;
					next_slave_address = `BNO055_SLAVE_ADDRESS;
					clear_waiting_ms   = `CLEAR_MS_TIMER; //  Clear and set to wait_ms value
					next_go_flag       = `NOT_GO;
					next_bno055_state  = `BNO055_SUB_STATE_START;
					next_return_state  = `BNO055_STATE_WAIT_20MS;
					next_data_reg      = `BNO055_OPR_MODE_ADDR;
					next_data_tx       = `BNO055_OPERATION_MODE_NDOF;
					next_read_write_in = `I2C_WRITE;
				end
				`BNO055_STATE_WAIT_20MS: begin // Wait 20ms to go from config to running mode
					next_imu_good      = `FALSE;
					next_slave_address = `BNO055_SLAVE_ADDRESS;
					clear_waiting_ms   = `RUN_MS_TIMER;
					next_data_reg      = `BYTE_ALL_ZERO;
					next_data_tx       = `BYTE_ALL_ZERO;
					next_go_flag       = `NOT_GO;
					next_bno055_state  = `BNO055_STATE_WAIT_20MS;
					rstn_buffer        = `LOW; //  Clear RX data buffer index before starting next state's read burst
					if((count_ms[31] == 1'b1) ) begin // Wait for count_ms wrapped around to 0x3FFFFFF
						next_wait_ms       = 'd20; //  Pause for 20 ms between iterations, for next wait state, not used in this one
						next_bno055_state  = `BNO055_STATE_READ_IMU_DATA_BURST;
					end
				end
				`BNO055_STATE_READ_IMU_DATA_BURST: begin //  Page 0 - Read from Acceleration Data X-Axis LSB to Calibration Status registers - 46 bytes
					clear_waiting_ms       = `CLEAR_MS_TIMER; //  Clear and set to wait_ms value
					next_wait_ms           = 'd20; //  Pause for 20 ms between iterations, for next wait state, not used here
					next_slave_address     = `BNO055_SLAVE_ADDRESS;
					next_go_flag           = `NOT_GO;
					next_bno055_state      = `BNO055_SUB_STATE_START;
					next_return_state      = `BNO055_STATE_WAIT_IMU_POLL_TIME;
					next_data_reg          = `BNO055_ACCEL_DATA_X_LSB_ADDR;
					next_data_tx           = `BYTE_ALL_ZERO;
					next_read_write_in     = `I2C_READ;
					next_target_read_count = `DATA_RX_BYTE_REG_CNT;
					next_led_view_index    = (`DATA_RX_BYTE_REG_CNT-1); //  Calibration status will be in the last byte buffer, index 45
				end
				`BNO055_STATE_WAIT_IMU_POLL_TIME: begin 	// Wait 20 ms between polls to maintain 50Hz polling rate
												//wait time is i2c time + time spent here, for a total of 20ms,
												//i2c time is variable and dependent on slave
												//This timer starts at the beginning of the the previous state
					next_imu_good      = `TRUE;
					next_slave_address = `BNO055_SLAVE_ADDRESS;
					clear_waiting_ms   = `RUN_MS_TIMER;
					next_data_reg      = `BYTE_ALL_ZERO;
					next_data_tx       = `BYTE_ALL_ZERO;
					next_go_flag       = `NOT_GO;
					next_bno055_state  = `BNO055_STATE_WAIT_IMU_POLL_TIME;
					rstn_buffer        = `LOW; //  Clear the RX data buffer index starting next state's read burst
					if((count_ms[31] == 1'b1) ) begin // Wait for count_ms wrapped around to 0x3FFFFFF
						next_bno055_state  = `BNO055_STATE_READ_IMU_DATA_BURST;
					end
				end

				// FSM Sub States - Repeated for each i2c transaction
				`BNO055_SUB_STATE_START: begin //  Begin i2c transaction, wait for busy to be asserted
					next_slave_address     = `BNO055_SLAVE_ADDRESS;
					next_go_flag           = `GO;
					if(busy && rstn_imu) // Stay here until i2c is busy AND the IMU isn't in reset (Prevent glitch at WD event)
						next_bno055_state = `BNO055_SUB_STATE_WAIT_I2C;
					else
						next_bno055_state = `BNO055_SUB_STATE_START;
				end
				`BNO055_SUB_STATE_WAIT_I2C: begin //  Wait for end of i2c transaction, wait for busy to be cleared
					next_go_flag           = `NOT_GO;
					next_slave_address     = `BNO055_SLAVE_ADDRESS;
					if(~busy && rstn_imu) // Stay here until i2c is not busy AND the IMU isn't in reset (Prevent glitch at WD event)
						next_bno055_state = `BNO055_SUB_STATE_STOP;
					else
						next_bno055_state = `BNO055_SUB_STATE_WAIT_I2C;
				end //  Set output data latch strobe and return to major FSM state
				`BNO055_SUB_STATE_STOP: begin
					next_go_flag           = `NOT_GO;
					next_slave_address     = `BNO055_SLAVE_ADDRESS;
					next_data_reg          = `BYTE_ALL_ZERO;
					next_data_tx           = `BYTE_ALL_ZERO;
					if( (read_write_in == `I2C_READ)) begin //  Only latch data if this was a read
						rx_data_latch_strobe = `HIGH;	
					end
					next_bno055_state      = return_state;
				end

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
				// Default case, shouldn't be triggered
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
				default: begin
					next_imu_good      = `FALSE;
					next_go_flag       = `NOT_GO;
					next_bno055_state  = `BNO055_STATE_RESET;
					next_return_state  = `BYTE_ALL_ZERO;
					next_data_reg      = `BYTE_ALL_ZERO;
					next_data_tx       = `BYTE_ALL_ZERO;
					next_read_write_in = `I2C_READ;
					rx_data_latch_strobe = `LOW;
				end
			endcase
		end
	end
endmodule