onerror { resume }
transcript off
add wave -noreg -logic {/i2c_module_tb/i2c/sda}
add wave -noreg -logic {/i2c_module_tb/i2c/scl}
add wave -noreg -logic {/i2c_module_tb/i2c/clk}
add wave -noreg -hexadecimal -literal {/i2c_module_tb/i2c/addr}
add wave -noreg -hexadecimal -literal {/i2c_module_tb/i2c/dataTX}
add wave -noreg -logic {/i2c_module_tb/i2c/modRegInReg}
add wave -noreg -logic {/i2c_module_tb/i2c/modDataInReg}
add wave -noreg -hexadecimal -literal {/i2c_module_tb/i2c/dataRX}
add wave -noreg -logic {/i2c_module_tb/i2c/ack}
add wave -noreg -logic {/i2c_module_tb/i2c/ack_flag}
add wave -noreg -logic {/i2c_module_tb/rstn}
add wave -noreg -logic {/i2c_module_tb/i2c/rstn}
add wave -noreg -logic {/i2c_module_tb/i2c/rstn_local}
add wave -noreg -logic {/i2c_module_tb/i2c/we}
add wave -noreg -logic {/i2c_module_tb/i2c/nextWe}
add wave -noreg -logic {/i2c_module_tb/i2c/stb}
add wave -noreg -logic {/i2c_module_tb/i2c/nextStb}
add wave -noreg -logic {/i2c_module_tb/i2c/cyc}
add wave -noreg -logic {/i2c_module_tb/i2c/readAction}
add wave -noreg -logic {/i2c_module_tb/i2c/writeAction}
add wave -noreg -hexadecimal -literal {/i2c_module_tb/i2c/nextAddr}
add wave -noreg -hexadecimal -literal {/i2c_module_tb/i2c/nextDataTX}
add wave -noreg -decimal -literal {/i2c_module_tb/i2c/i2cCmdState}
add wave -noreg -decimal -literal {/i2c_module_tb/i2c/i2cCmdNextState}
add wave -noreg -decimal -literal {/i2c_module_tb/i2c/count_1us}
add wave -noreg -logic {/i2c_module_tb/i2c/irq_out}
add wave -noreg -logic {/i2c_module_tb/i2c/waiting1us}
add wave -noreg -logic {/i2c_module_tb/i2c/clearWaiting1us}
cursor "Cursor 1" 99597.86ns  
transcript on
