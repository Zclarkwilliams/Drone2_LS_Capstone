onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /i2c_module_tb/i2c/scl
add wave -noupdate /i2c_module_tb/i2c/sda
add wave -noupdate /i2c_module_tb/i2c/rstn
add wave -noupdate /i2c_module_tb/i2c/clk
add wave -noupdate /i2c_module_tb/i2c/adr
add wave -noupdate /i2c_module_tb/i2c/we
add wave -noupdate /i2c_module_tb/i2c/stb
add wave -noupdate /i2c_module_tb/i2c/cyc
add wave -noupdate /i2c_module_tb/i2c/ack
add wave -noupdate /i2c_module_tb/i2c/inta
add wave -noupdate /i2c_module_tb/i2c/READ_CMD
add wave -noupdate /i2c_module_tb/i2c/WRITE_CMD
add wave -noupdate -radix binary /i2c_module_tb/i2c/delay
add wave -noupdate -radix decimal /i2c_module_tb/i2c/cmd_addr
add wave -noupdate -radix decimal /i2c_module_tb/i2c/cmd_data
add wave -noupdate -radix decimal /i2c_module_tb/i2c/CSRcaller_next_state
add wave -noupdate -radix decimal /i2c_module_tb/i2c/WRcaller_next_state
add wave -noupdate -radix decimal /i2c_module_tb/i2c/RDcaller_next_state
add wave -noupdate -radix decimal /i2c_module_tb/i2c/check_SR
add wave -noupdate -radix decimal /i2c_module_tb/i2c/data_rxr
add wave -noupdate -radix decimal /i2c_module_tb/i2c/I2C_CMD_State
add wave -noupdate -radix decimal /i2c_module_tb/i2c/I2C_CMD_NextState
add wave -noupdate /i2c_module_tb/i2c/I2C_clk
add wave -noupdate /i2c_module_tb/i2c/stb0
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 258
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ns} {140212 ns}
