

`define VL53L1X_SOFT_RESET                                                   16'h0000
`define VL53L1X_I2C_SLAVE__DEVICE_ADDRESS                                    16'h0001
`define VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND                        16'h0008
`define VL53L1X_VHV_CONFIG__INIT                                             16'h000B
`define VL53L1X_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS               16'h0016
`define VL53L1X_ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS           16'h0018
`define VL53L1X_ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS           16'h001A
`define VL53L1X_ALGO__PART_TO_PART_RANGE_OFFSET_MM                           16'h001E
`define VL53L1X_MM_CONFIG__INNER_OFFSET_MM                                   16'h0020
`define VL53L1X_MM_CONFIG__OUTER_OFFSET_MM                                   16'h0022
`define VL53L1X_GPIO_HV_MUX__CTRL                                            16'h0030
`define VL53L1X_GPIO__TIO_HV_STATUS                                          16'h0031
`define VL53L1X_SYSTEM__INTERRUPT_CONFIG_GPIO                                16'h0046
`define VL53L1X_PHASECAL_CONFIG__TIMEOUT_MACROP                              16'h004B
`define VL53L1X_RANGE_CONFIG__TIMEOUT_MACROP_A_HI                            16'h005E
`define VL53L1X_RANGE_CONFIG__VCSEL_PERIOD_A                                 16'h0060
`define VL53L1X_RANGE_CONFIG__VCSEL_PERIOD_B                                 16'h0063
`define VL53L1X_RANGE_CONFIG__TIMEOUT_MACROP_B_HI                            16'h0061
`define VL53L1X_RANGE_CONFIG__TIMEOUT_MACROP_B_LO                            16'h0062
`define VL53L1X_RANGE_CONFIG__SIGMA_THRESH                                   16'h0064
`define VL53L1X_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS                  16'h0066
`define VL53L1X_RANGE_CONFIG__VALID_PHASE_HIGH                               16'h0069
`define VL53L1X_SYSTEM__INTERMEASUREMENT_PERIOD                              16'h006C
`define VL53L1X_SYSTEM__THRESH_HIGH                                          16'h0072
`define VL53L1X_SYSTEM__THRESH_LOW                                           16'h0074
`define VL53L1X_SD_CONFIG__WOI_SD0                                           16'h0078
`define VL53L1X_SD_CONFIG__INITIAL_PHASE_SD0                                 16'h007A
`define VL53L1X_ROI_CONFIG__USER_ROI_CENTRE_SPAD                             16'h007F
`define VL53L1X_ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE                16'h0080
`define VL53L1X_SYSTEM__SEQUENCE_CONFIG                                      16'h0081
`define VL53L1X_SYSTEM__GROUPED_PARAMETER_HOLD                               16'h0082
`define VL53L1X_POWER_MANAGEMENT__GO1_POWER_FORCE                            16'h0083
`define VL53L1X_SYSTEM__INTERRUPT_CLEAR                                      16'h0086
`define VL53L1X_SYSTEM__MODE_START                                           16'h0087
`define VL53L1X_RESULT__RANGE_STATUS                                         16'h0089
`define VL53L1X_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0                       16'h008C
`define VL53L1X_RESULT__AMBIENT_COUNT_RATE_MCPS_SD                           16'h0090
`define VL53L1X_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0               16'h0096
`define VL53L1X_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0  16'h0098
`define VL53L1X_RESULT__OSC_CALIBRATE_VAL                                    16'h00DE
`define VL53L1X_ANA_CONFIG__POWERDOWN_GO1                                    16'h00E0
`define VL53L1X_FIRMWARE__SYSTEM_STATUS                                      16'h00E5
`define VL53L1X_INTERRUPT_MANAGER__ENABLES                                   16'h00FD
`define VL53L1X_IDENTIFICATION__MODEL_ID                                     16'h010F // has the value 0xEACC
`define VL53L1X_ROI_CONFIG__MODE_ROI_CENTRE_SPAD                             16'h013E


`define VL53L1X_IDENTIFICATION__MODEL_ID_VAL                                 16'hEACC     //  Chip ID of VL53L1X
`define VL53L1X_SLAVE_ADDRESS                                                7'b101_0010  //  VL53L1X SLAVE address 0x52

`define VL53L1X_DATA_RX_BYTE_REG_CNT                                         8            //  The number of byte registers used to receive all measurement data
`define VL53L1X_CAL_DATA_REG_CNT                                             22           //  The number of byte registers needed for all calibration data