/*

ECE 412-413 Capstone Winter/Spring 2018
Team 32 Drone2 SOC
Ethan Grinnell, Brett Creely, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams

*/

`define I2C_STATE_BITS 6     //  The number of bits used to represent the current state

// Registers

//EFB I2C Module #1 (Primary I2C)
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

//EFB I2C Module #2 (Secondary I2C)
`define I2C_2_CR    8'h4A //  Control register
`define I2C_2_CMDR  8'h4B //  Command register
`define I2C_2_BR0   8'h4C //  clock pre-scaler low byte
`define I2C_2_BR1   8'h4D //  clock pre-scaler high byte
`define I2C_2_TXDR  8'h4E //  Transmit data
`define I2C_2_SR    8'h4F //  Status
`define I2C_2_GCDR  8'h50 //  General Call
`define I2C_2_RXDR  8'h51 //  Receive data
`define I2C_2_IRQ   8'h52 //  IRQ
`define I2C_2_IRQEN 8'h53 //  IRQ Enable
// WE bits
`define I2C_WE_WRITE 1
`define I2C_WE_READ  0

// Control register bits
`define I2C_CR_I2CEN             (8'b1000_0000) //(1'b1<<7)  //  Enable I2C module
`define I2C_CR_GCEN              (8'b0100_0000) //(1'b1<<6)  //  Enable general call
`define I2C_CR_WKUPEN            (8'b0010_0000) //(1'b1<<5)  //  Enable Wakeup from sleep
`define I2C_CR_SDA_DEL_SEL_300NS (8'b0000_0000) //(2'b00<<2) //  SDA output delay of 300ns
`define I2C_CR_SDA_DEL_SEL_150NS (8'b0000_0100) //(2'b01<<2) //  SDA output delay of 150ns
`define I2C_CR_SDA_DEL_SEL_75NS  (8'b0000_1000) //(2'b10<<2) //  SDA output delay of 75s
`define I2C_CR_SDA_DEL_SEL_0NS   (8'b0000_1100) //(2'b11<<2) //  SDA output delay of 0ns

// Command register bits
`define I2C_CMDR_STA             (8'b1000_0000) //(1'b1<<7) //  Generate start condition
`define I2C_CMDR_STO             (8'b0100_0000) //(1'b1<<6) //  Generate stop condition
`define I2C_CMDR_RD              (8'b0010_0000) //(1'b1<<5) //  Indicate READ from slave
`define I2C_CMDR_WR              (8'b0001_0000) //(1'b1<<4) //  Indicate WRITE to slave
`define I2C_CMDR_ACK             (8'b0000_1000) //(1'b1<<3) //  Ack option when receiving, 0 ACK, 1 NACK
`define I2C_CMDR_CKSDIS          (8'b0000_0100) //(1'b1<<2) //  Disable clock stretching by slave, 0 Clock Stretching ENABLED, 1 = Disabled

// Status register bit numbers
`define I2C_SR_TIP   3'd7 	//  Transmit In Progress. The current data byte is being transferred.signal synchronization. This bit could be high after configuration wake-
										//  up and before the first valid I2C transfer start (when BUSY is low), and it is not indicating byte in transfer, but an invalid indicator.
										//  1 Busy, 0 Not Busy
`define I2C_SR_BUSY  3'd6	//  I2C Bus Busy - 1 Busy, 0 Not Busy
`define I2C_SR_RARC  3'd5	//  Received Acknowledge, 0 Ack Received, 1 NO Ack Received
`define I2C_SR_SRW   3'd4	//  Slave Read/Write - Indicates transmit or receive mode 1 Master receive, 0 Master transmitting
`define I2C_SR_ARBL  3'd3	//  Arbitration lost - The core lost arbitration in Master mode, 1 Arbitration lost, 0 Normal
`define I2C_SR_TRRDY 3'd2	//  Transmitter or Receiver Ready Ready to receive or transmit, 1 Ready, 0 Not Ready
`define I2C_SR_TROE  3'd3	//  Transmitter/Receiver Overrun Error, 1 overrun, 0 Normal
`define I2C_SR_HGC   3'd2	//  Hardware General Call received, 1 General call in slave mode, 0 Normal

// Status Register Bit Masks - From http://www.latticesemi.com/view_document?document_id=45881
`define I2C_SR_MASK_TIP     8'h80 
`define I2C_SR_MASK_BUSY    8'h40 
`define I2C_SR_MASK_RARC    8'h20 
`define I2C_SR_MASK_SRW     8'h10 
`define I2C_SR_MASK_ARBL    8'h08 
`define I2C_SR_MASK_TRRDY   8'h04 
`define I2C_SR_MASK_TROE    8'h02 
`define I2C_SR_MASK_HGC     8'h01 

// Value aliases
`define RUN_US_TIMER            1'b1         //  Flag that starts multi us timer
`define CLEAR_US_TIMER          1'b0         //  Flag that stops/clears multi us timer
`define RUN_WD_TIMER            1'b1         //  Flag that runs the watchdog timer - Default
`define CLEAR_WD_TIMER          1'b0         //  Flag that clears watchdog timer and sets to max value
`define I2C_CMD_START           1'b1         //  Start bit for both stb and cyc
`define I2C_CMD_STOP            1'b0         //  Stop  bit for both stb and cyc
`define I2C_BUS_RD_BIT          1'b1         //  Set Read/Write to READ, appended to SLAVE_ADDRESS as SDA transmits to bus
`define I2C_BUS_WR_BIT          1'b0         //  Set Read/Write to WRITE, appended to SLAVE_ADDRESS as SDA transmits to bus
`define I2C_BUS_XFER_IN_PROG    1'b1         //  Bus transfer in progress
`define I2C_BUS_NO_XFER_IN_PROG 1'b0         //  No bus transfer in progress, transfer completed
`define I2C_BUS_BUSY            1'b1         //  Bus busy with a transaction
`define I2C_BUS_NOT_BUSY        1'b0         //  Bus not busy
`define I2C_BUS_RARC            1'b0         //  Ack/NAck received
`define I2C_BUS_NO_RARC         1'b1         //  No Ack/NAck received
`define I2C_BUS_SRW_MASTER_RX   1'b1         //  Master receiving / Slave Transmitting
`define I2C_BUS_SRW_MASTER_TX   1'b0         //  Master Transmitting / Slave Receiving
`define I2C_BUS_ARBL_LOST       1'b1         //  Arbitration Lost
`define I2C_BUS_ARBL_NORMAL     1'b0         //  Arbitration Not Lost
`define I2C_BUS_TRRDY_READY     1'b1         //  Transmit/Receive Ready
`define I2C_BUS_TRRDY_NOT_READY 1'b0         //  Transmit/Receive Not Ready
`define I2C_BUS_TROE_OVERRUN    1'b1         //  Transmit/Receive Overrun Error
`define I2C_BUS_TROE_NORMAL     1'b0         //  No Transmit/Receiver Overrun Error
`define I2C_BUS_HGC_RECEIVED    1'b1         //  Received General Call
`define I2C_BUS_HGC_NORMAL      1'b0         //  No General Call Received
`define WB_SLAVE_ADDRESS        7'b10000_01  //  Wishbone bus slave address
// 
//  State Definitions
// 
// Initial default state

`define I2C_STATE_RESET             0
`define I2C_STATE_SET_PRESCALE_LOW  1
`define I2C_STATE_SET_PRESCALE_HI   2
`define I2C_STATE_INIT_BOOT_WAIT1   3
`define I2C_STATE_INIT_BOOT_WAIT2   4
`define I2C_STATE_INIT_ENA          5
`define I2C_STATE_WAIT              6
`define I2C_STATE_WAIT_NOT_BUSY     7



// Write states
`define I2C_STATE_W_SET_SLAVE_WRITE 8
`define I2C_STATE_W_SET_WRITE       9
`define I2C_STATE_W_READ_CHK_SR1    10
`define I2C_STATE_W_SET_SLAVE_REG   11
`define I2C_STATE_W_WRITE_SLAVE_REG 12
`define I2C_STATE_W_READ_CHK_SR2    13
`define I2C_STATE_W_SET_REG_VAL     14
`define I2C_STATE_W_WRITE_REG_VAL   15
`define I2C_STATE_W_READ_CHK_SR3    16
`define I2C_STATE_W_WRITE_STOP      17
`define I2C_STATE_W_READ_CHK_SR4    18

// Read states
`define I2C_STATE_R_SET_SLAVE_WRITE 19
`define I2C_STATE_R_SET_WRITE1      20
`define I2C_STATE_R_READ_CHK_SR1    21
`define I2C_STATE_R_SET_SLAVE_REG   22
`define I2C_STATE_R_WRITE_REG       23
`define I2C_STATE_R_READ_CHK_SR2    24
`define I2C_STATE_R_SET_SLAVE_READ  25
`define I2C_STATE_R_SET_WRITE2      26
`define I2C_STATE_R_WAIT_SRW        27
`define I2C_STATE_R_SET_READ        28
`define I2C_STATE_R_READ_CHK_SR3    29
`define I2C_STATE_R_READ_DATA1      30
`define I2C_STATE_R_SET_READ_STOP   31
`define I2C_STATE_R_READ_CHK_SR4    32
`define I2C_STATE_R_READ_DATA2      33
`define I2C_STATE_R_READ_CHK_SR5    34