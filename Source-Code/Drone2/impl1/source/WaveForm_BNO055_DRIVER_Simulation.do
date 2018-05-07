onerror { resume }
transcript off
add wave -noreg -hexadecimal -literal {/bno055_module_tb/data_rx}
add wave -noreg -logic {/bno055_module_tb/go}
add wave -noreg -logic {/bno055_module_tb/i2c_ack}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/i2c_count}
add wave -noreg -logic {/bno055_module_tb/purn}
add wave -noreg -logic {/bno055_module_tb/read_write_in}
add wave -noreg -logic {/bno055_module_tb/rstn}
add wave -noreg -logic {/bno055_module_tb/rstn_imu}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/data_reg}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/data_tx}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/bno055_state}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/count_ms}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/addr}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/i2c/i2c_cmd_state}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/data_tx}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/data_rx}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/module_data_out}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/module_data_in}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/module_reg_in}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/read_write_in}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/go}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/busy}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/one_byte_ready}
add wave -noreg -logic {/bno055_module_tb/i2c_ack}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/i2c/target_read_count}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/go}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/one_byte_ready}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/busy}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/rstn_imu}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/data_latch}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/next_data_latch}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/i2c/bytes_read_remain}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/clear_read_count}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/count_us}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/ack_flag}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/next_ack_flag}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/ack}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/clear_read_count}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/data_sim_regs}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/efb_registers}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/accel_rate_x}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/accel_rate_y}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/accel_rate_z}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/magneto_rate_x}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/magneto_rate_y}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/magneto_rate_z}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/gyro_rate_x}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/gyro_rate_y}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/gyro_rate_z}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/euler_angle_x}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/euler_angle_y}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/euler_angle_z}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/quaternion_data_w}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/quaternion_data_x}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/quaternion_data_y}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/quaternion_data_z}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/linear_accel_x}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/linear_accel_y}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/linear_accel_z}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/gravity_accel_x}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/gravity_accel_y}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/gravity_accel_z}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/temperature}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/calib_status}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/x_velocity}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/y_velocity}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/z_velocity}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/slave_address}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/next_slave_address}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/bno055_state}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/next_bno055_state}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/return_state}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/next_return_state}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/data_rx_reg_index}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/led_view_index}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/next_led_view_index}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/data_rx_reg}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c_number}
add wave -noreg -logic {/bno055_module_tb/bno055/test_data_flag}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/test_data_index}
add wave -noreg -logic {/bno055_module_tb/bno055/clear_test_data_index}
add wave -noreg -logic {/bno055_module_tb/bno055/increment_test_data_index}
cursor "Cursor 1" 680112705.47ns  
transcript on
