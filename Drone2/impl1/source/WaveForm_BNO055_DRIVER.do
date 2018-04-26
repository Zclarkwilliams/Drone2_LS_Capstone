onerror { resume }
transcript off
add wave -noreg -logic {/bno055_module_tb/clk}
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
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/bno055_next_state}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/bno055_return_state}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/count_ms}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/addr}
add wave -noreg -decimal -literal {/bno055_module_tb/bno055/i2c/i2c_cmd_state}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/count_i2c_delay}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/clear_wait_i2c_clk}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/data_tx}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/data_rx}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/module_data_out}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/module_data_in}
add wave -noreg -hexadecimal -literal {/bno055_module_tb/bno055/i2c/module_reg_in}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/read_write_in}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/go}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/busy}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/one_byte_ready}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/sda1}
add wave -noreg -logic {/bno055_module_tb/bno055/i2c/scl1}
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
cursor "Cursor 1" 0ps  
transcript on
