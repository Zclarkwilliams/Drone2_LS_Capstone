`define STATE_BITS 5 //  The number of bits used to represent the current state
`define BNO055_CHIP_ID_REG 8'h00

`define WAIT_1US_DIVIDER 133 //  Wait 1us at 133MHz

// Registers

`define I2C_1_CR    8'h40 //  Control register
`define I2C_1_CMDR  8'h41 //  Command register
`define I2C_1_BR0   8'h42 //  clock pre-scaler low byte
`define I2C_1_BR1   8'h43 //  clock pre-scaler high byte
`define I2C_1_TXDR  8'h44 //  Transmit data
`define I2C_1_SR    8'h45 //  Status
`define I2C_1_GCDR  8'h46 //  General Call
`define I2C_1_RXDR  8'h47 //  Receive data
`define I2C_1_IRQ   8'h48 //  IRQ
`define I2C_1_IRQEN 8'h49 //  IRQ Enable

// WE bits
`define I2C_1_WE_WRITE 1
`define I2C_1_WE_READ  0

// Control register bits
`define I2C_1_CR_I2CEN             (1'b1<<7)  //  Enable I2C module
`define I2C_1_CR_GCEN              (1'b1<<6)  //  Enable general call
`define I2C_1_CR_WKUPEN            (1'b1<<5)  //  Enable Wakeup from sleep
`define I2C_1_CR_SDA_DEL_SEL_300NS (2'b00<<2) //  SDA output delay of 300ns
`define I2C_1_CR_SDA_DEL_SEL_150NS (2'b01<<2) //  SDA output delay of 150ns
`define I2C_1_CR_SDA_DEL_SEL_75NS  (2'b10<<2) //  SDA output delay of 75s
`define I2C_1_CR_SDA_DEL_SEL_0NS   (2'b11<<2) //  SDA output delay of 0ns

// Command register bits
`define I2C_1_CMDR_STA             (8'b1000_0000) //(1'b1<<7) //  Generate start condition
`define I2C_1_CMDR_STO             (8'b0100_0000) //(1'b1<<6) //  Generate stop condition
`define I2C_1_CMDR_RD              (8'b0010_0000) //(1'b1<<5) //  Indicate READ from slave
`define I2C_1_CMDR_WR              (8'b0001_0000) //(1'b1<<4) //  Indicate WRITE to slave
`define I2C_1_CMDR_ACK             (8'b0000_1000) //(1'b1<<3) //  Ack option when receiving, 0 ACK, 1 NACK
`define I2C_1_CMDR_CKDIS           (8'b0000_0100) //(1'b1<<2) //  Disable clock stretching by slave, 0 Clock Stretching ENABLED, 1 = Disabled

// Status register bit numbers
`define I2C_1_SR_TIP   3'h7 	//  Transmit In Progress. The current data byte is being transferred.signal synchronization. This bit could be high after configuration wake-
										//  up and before the first valid I2C transfer start (when BUSY is low), and it is not indicating byte in transfer, but an invalid indicator.
										//  1 Busy, 0 Not Busy
`define I2C_1_SR_BUSY  3'h6	//  I2C Bus Busy - 1 Busy, 0 Not Busy
`define I2C_1_SR_RARC  3'h5	//  Received Acknowledge, 0 Ack Received, 1 NO Ack Received
`define I2C_1_SR_SRW   3'h4	//  Slave Read/Write - Indicates transmit or receive mode 1 Master receive, 0 Master transmitting
`define I2C_1_SR_ARBL  3'h3	//  Arbitration lost - The core lost arbitration in Master mode, 1 Arbitration lost, 0 Normal
`define I2C_1_SR_TRRDY 3'h2	//  Transmitter or Receiver Ready Ready to receive or transmit, 1 Ready, 0 Not Ready
`define I2C_1_SR_TROE  3'h3	//  Transmitter/Receiver Overrun Error, 1 overrun, 0 Normal
`define I2C_1_SR_HGC   3'h2	//  Hardware General Call received, 1 General call in slave mode, 0 Normal

// Status Register Bit Masks - From http://www.latticesemi.com/view_document?document_id=45881
`define I2C_1_SR_MASK_TIP     8'h80 
`define I2C_1_SR_MASK_BUSY    8'h40 
`define I2C_1_SR_MASK_RARC    8'h20 
`define I2C_1_SR_MASK_SRW     8'h10 
`define I2C_1_SR_MASK_ARBL    8'h08 
`define I2C_1_SR_MASK_TRRDY   8'h04 
`define I2C_1_SR_MASK_TROE    8'h02 
`define I2C_1_SR_MASK_HGC     8'h01 

// Value aliases
`define START         1'b1 //  Start bit for both stb and cyc
`define STOP          1'b0 //  Stop  bit for both stb and cyc
`define START_1US_TIMER 1'b1
`define STOP_1US_TIMER  1'b0
`define FALSE         1'b0
`define TRUE          1'b1
`define RD_BIT        1'b1  //  Set Read/Write to READ, appended to SLAVE_ADDRESS as SDA transmits to bus
`define WR_BIT        1'b0  //  Set Read/Write to WRITE, appended to SLAVE_ADDRESS as SDA transmits to bus
`define BUSY          1'b1  //  Bus busy with a transaction
`define NOT_BUSY      1'b0  //  Bus not busy
`define SRW_MODE      1'b1  //  Master receiving / Slave Transmitting
`define NOT_SRW_MODE  1'b0  //  Master Transmitting / Slave Receiving
`define ALL_ZERO      8'h00 // A byte of all zeros
`define BNO055_SLAVE_ADDRESS 7'b01010_01 // BNO055 SLAVE address 0x29
`define WB_SLAVE_ADDRESS     7'b10000_01 //
// 
//  State Definitions
// 
// Initial default state
`define I2C_CMD_STATE_RESET             0
`define I2C_CMD_STATE_INIT1_BOOT_WAIT1  1
`define I2C_CMD_STATE_INIT1_BOOT_WAIT2  2
`define I2C_CMD_STATE_PRESCALE_LOW      3
`define I2C_CMD_STATE_PRESCALE_HI       4
`define I2C_CMD_STATE_INIT2_BOOT_WAIT   5
`define I2C_CMD_STATE_INIT_ENA          6
`define I2C_CMD_STATE_WAIT              7
`define I2C_CMD_STATE_WAIT_NOT_BUSY     8

// Write states
`define I2C_CMD_STATE_W_SET_SLAVE       9
`define I2C_CMD_STATE_W_SET_WRITE       10
`define I2C_CMD_STATE_W_READ_CHK_SR1    11
`define I2C_CMD_STATE_W_SET_S_REG       12
`define I2C_CMD_STATE_W_WRITE_REG       13
`define I2C_CMD_STATE_W_READ_CHK_SR2    14
`define I2C_CMD_STATE_W_SET_REG_VAL     15
`define I2C_CMD_STATE_W_WRITE_REG_VAL   16
`define I2C_CMD_STATE_W_READ_CHK_SR3    17

// Read states
`define I2C_CMD_STATE_R_SET_SLAVE_WRITE 18
`define I2C_CMD_STATE_R_SET_WRITE1      19
`define I2C_CMD_STATE_R_READ_CHK_SR1    20
`define I2C_CMD_STATE_R_SET_S_REG       21
`define I2C_CMD_STATE_R_WRITE_REG       22
`define I2C_CMD_STATE_R_READ_CHK_SR2    23
`define I2C_CMD_STATE_R_SET_SLAVE_READ  24
`define I2C_CMD_STATE_R_SET_WRITE2      25
`define I2C_CMD_STATE_R_WAIT_SRW        26
`define I2C_CMD_STATE_R_SET_READ_STOP   27
`define I2C_CMD_STATE_R_READ_CHK_SR3    28
`define I2C_CMD_STATE_R_READ_DATA       29