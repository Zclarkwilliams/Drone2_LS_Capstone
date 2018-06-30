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
 *    Freescale Semiconductor MPL3115A2 Altimeter register offsets
 */
 
`define MPL3115A2_STATUS                 8'h00 // Sensor Status Register, Alias for Data Ready register [Auto increments through data registers from here]
`define MPL3115A2_OUT P_MSB              8'h01 // Pressure Data Out MSB
`define MPL3115A2_OUT_P_CSB              8'h02 // Pressure Data Out CSB
`define MPL3115A2_OUT_P _LSB             8'h03 // Pressure Data Out LSB
`define MPL3115A2_OUT_T_MSB              8'h04 // Temperature Data Out MSB
`define MPL3115A2_OUT_T _LSB             8'h05 // Temperature Data Out LSB [Auto increment starts from 0x00 again here]
`define MPL3115A2_DR_STATUS              8'h06 // Sensor Status Register, Data Ready register [Auto increments through data delta registers from here]
`define MPL3115A2_OUT_P_DELTA MSB        8'h07 // Pressure Data Out Delta MSB
`define MPL3115A2_OUT_P_DELTA_CSB        8'h08 // Pressure Data Out Delta CSB
`define MPL3115A2_OUT_P_DELTA_LSB        8'h09 // Pressure Data Out Delta LSB
`define MPL3115A2_OUT_T_DELTA_MSB        8'h0A // Temperature Data Out Delta MSB
`define MPL3115A2_OUT_T_DELTA_LSB        8'h0B // Temperature Data Out Delta LSB [Auto increment starts from 0x06 again here]
`define MPL3115A2_WHO_AM_I               8'h0C // Device Identification Register
`define MPL3115A2_F_STATUS               8'h0D // FIFO Status Register
`define MPL3115A2_F_DATA                 8'h0E // FIFO 8-bit Data Access
`define MPL3115A2_F_SETUP                8'h0F // FIFO Setup Register
`define MPL3115A2_TIME_DLY               8'h10 // Time Delay Register
`define MPL3115A2_SYSMOD                 8'h11 // System Mode Register
`define MPL3115A2_INT_SOURCE             8'h12 // Interrupt Source Register
`define MPL3115A2_PT_DATA_CFG            8'h13 // PT Data Configuration Register
`define MPL3115A2_BAR_IN_MSB             8'h14 // BAR Input in MSB
`define MPL3115A2_BAR_IN_LSB             8'h15 // BAR Input in LSB
`define MPL3115A2_P_TGT_MSB              8'h16 // Pressure Target MSB
`define MPL3115A2_P_TGT_LSB              8'h17 // Pressure Target LSB
`define MPL3115A2_T_TGT                  8'h18 // Temperature Target
`define MPL3115A2_P_WND_MSB              8'h19 // Pressure/Altitude Window MSB
`define MPL3115A2_P_WND_LSB              8'h1A // Pressure/Altitude Window LSB
`define MPL3115A2_T_WND                  8'h1B // Temperature Window
`define MPL3115A2_P_MIN_MSB              8'h1C // Minimum Pressure Data Out MSB
`define MPL3115A2_P_MIN_CSB              8'h1D // Minimum Pressure Data Out CSB
`define MPL3115A2_P_MIN_LSB              8'h1E // Minimum Pressure Data Out LSB
`define MPL3115A2_T_MIN_MSB              8'h1F // Minimum Temperature Data Out MSB
`define MPL3115A2_T_MIN_LSB              8'h20 // Minimum Temperature Data Out LSB
`define MPL3115A2_P_MAX_MSB              8'h21 // Maximum Pressure Data Out MSB
`define MPL3115A2_P_MAX_CSB              8'h22 // Maximum Pressure Data Out CSB
`define MPL3115A2_P_MAX_LSB              8'h23 // Maximum Pressure Data Out LSB
`define MPL3115A2_T_MAX_MSB              8'h24 // Maximum Temperature Data Out MSB
`define MPL3115A2_T_MAX_LSB              8'h25 // Maximum Temperature Data Out LSB
`define MPL3115A2_CTRL_REG1              8'h26 // Control Register 1
`define MPL3115A2_CTRL_REG2              8'h27 // Control Register 2
`define MPL3115A2_CTRL_REG3              8'h28 // Control Register 3
`define MPL3115A2_CTRL_REG4              8'h29 // Control Register 4
`define MPL3115A2_CTRL_REG5              8'h2A // Control Register 5
`define MPL3115A2_OFF_P                  8'h2B // Pressure Data User Offset Register
`define MPL3115A2_OFF_T                  8'h2C // Temperature Data User Offset Register
`define MPL3115A2_OFF_H                  8'h2D // Altitude Data User Offset Register


// rx_data_reg register indices
`define MPL3115A2_STATUS_INDEX           46
`define MPL3115A2_OUT P_MSB_INDEX        47
`define MPL3115A2_OUT_P_CSB_INDEX        48
`define MPL3115A2_OUT_P _LSB_INDEX       49
`define MPL3115A2_OUT_T_MSB_INDEX        50
`define MPL3115A2_OUT_T _LSB_INDEX       51
`define MPL3115A2_DR_STATUS_INDEX        52
`define MPL3115A2_OUT_P_DELTA MSB_INDEX  53
`define MPL3115A2_OUT_P_DELTA_CSB_INDEX  54
`define MPL3115A2_OUT_P_DELTA_LSB_INDEX  55
`define MPL3115A2_OUT_T_DELTA_MSB_INDEX  56
`define MPL3115A2_OUT_T_DELTA_LSB_INDEX  57


// Ready-Only, system device ID value
`define MPL3115A2_WHO_AM_I_ID            8'hC4 //MPL3115A2 fixed device ID


// Ready-Only, system mode bit either ACTIVE or STANDBY
`define MPL3115A2_SYSMOD_STANDBY         8'h00 // System in standby
`define MPL3115A2_SYSMOD_ACTIVE          8'h01 // System active

// Read/Write - Configure system controls
`define MPL3115A2_CTRL_REG1_STYB_STANDBY (1'b0<<0)   // System in standby
`define MPL3115A2_CTRL_REG1_STYB_ACTIVE  (1'b1<<0)   // System active
`define MPL3115A2_CTRL_REG1_OST_DISABLE  (1'b0<<1)   // One-shot mode disabled
`define MPL3115A2_CTRL_REG1_OST_ENABLE   (1'b1<<1)   // One-shot mode enabled for a single measurement
`define MPL3115A2_CTRL_REG1_RST_DISABLE  (1'b0<<2)   // Software reset not started
`define MPL3115A2_CTRL_REG1_RST_ENABLE   (1'b1<<2)   // Software reset start
`define MPL3115A2_CTRL_REG1_OS_6MS       (3'b000<<3) // Oversample ratio set to 1,   polling interval 6ms minimum
`define MPL3115A2_CTRL_REG1_OS_10MS      (3'b001<<3) // Oversample ratio set to 2,   polling interval 10ms minimum
`define MPL3115A2_CTRL_REG1_OS_18MS      (3'b010<<3) // Oversample ratio set to 4,   polling interval 18ms minimum
`define MPL3115A2_CTRL_REG1_OS_34MS      (3'b011<<3) // Oversample ratio set to 8,   polling interval 34ms minimum
`define MPL3115A2_CTRL_REG1_OS_66MS      (3'b100<<3) // Oversample ratio set to 16,  polling interval 66ms minimum
`define MPL3115A2_CTRL_REG1_OS_130MS     (3'b101<<3) // Oversample ratio set to 32,  polling interval 130ms minimum
`define MPL3115A2_CTRL_REG1_OS_258MS     (3'b110<<3) // Oversample ratio set to 64,  polling interval 258ms minimum
`define MPL3115A2_CTRL_REG1_OS_512MS     (3'b111<<3) // Oversample ratio set to 128, polling interval 512ms minimum
`define MPL3115A2_CTRL_REG1_RAW_DISABLE  (1'b0<<6)   // Raw output mode disabled
`define MPL3115A2_CTRL_REG1_RAW_ENABLE   (1'b1<<6)   // Raw output mode enabled
`define MPL3115A2_CTRL_REG1_ALT_DISABLE  (1'b0<<7)   // Altimeter mode disabled, running as a barometer and providing pressure values in kPa
`define MPL3115A2_CTRL_REG1_ALT_ENABLE   (1'b1<<7)   // Altimeter mode enabled, running as an altimeter and providing altitude in meters

// I2C slave address of MPL3115A2
`define MPL3115A2_SLAVE_ADDRESS          8'b1100_0000 //  Slave address is 0x60


`define MPL3115A2_DATA_RX_BYTE_REG_CNT  6 //The number of bytes to read the system status, altitude/pressure, and temperature bytes
`define MPL3115A2_DELTA_RX_BYTE_REG_CNT 6 //The number of bytes to read the system status, altitude/pressure value delta, and temperature delta bytes
