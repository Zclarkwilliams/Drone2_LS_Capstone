

// Register addresses for the VL53L1X LiDAR sensor
`define VL53L1X_SOFT_RESET_ADDR                                                 16'h0000
`define VL53L1X_I2C_SLAVE_DEVICE_ADDRESS_ADDR                                   16'h0001
`define VL53L1X_ANA_CONFIG_VHV_REF_SEL_VDDPIX_ADDR                              16'h0002
`define VL53L1X_ANA_CONFIG_VHV_REF_SEL_VQUENCH_ADDR                             16'h0003
`define VL53L1X_ANA_CONFIG_REG_AVDD1V2_SEL_ADDR                                 16'h0004
`define VL53L1X_ANA_CONFIG_FAST_OSC_TRIM_ADDR                                   16'h0005
`define VL53L1X_OSC_MEASURED_FAST_OSC_FREQUENCY_HI_ADDR                         16'h0006
`define VL53L1X_OSC_MEASURED_FAST_OSC_FREQUENCY_LO_ADDR                         16'h0007
`define VL53L1X_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND_ADDR                       16'h0008
`define VL53L1X_VHV_CONFIG_COUNT_THRESH_ADDR                                    16'h0009
`define VL53L1X_VHV_CONFIG_OFFSET_ADDR                                          16'h000A
`define VL53L1X_VHV_CONFIG_INIT_ADDR                                            16'h000B
// Not sure what would go here                                                              16'h000C
`define VL53L1X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0_ADDR                           16'h000D
`define VL53L1X_GLOBAL_CONFIG_SPAD_ENABLES_REF_1_ADDR                           16'h000E
`define VL53L1X_GLOBAL_CONFIG_SPAD_ENABLES_REF_2_ADDR                           16'h000F
`define VL53L1X_GLOBAL_CONFIG_SPAD_ENABLES_REF_3_ADDR                           16'h0010
`define VL53L1X_GLOBAL_CONFIG_SPAD_ENABLES_REF_4_ADDR                           16'h0011
`define VL53L1X_GLOBAL_CONFIG_SPAD_ENABLES_REF_5_ADDR                           16'h0012
`define VL53L1X_GLOBAL_CONFIG_REF_EN_START_SELECT_ADDR                          16'h0013
`define VL53L1X_REF_SPAD_MAN_NUM_REQUESTED_REF_SPADS_ADDR                       16'h0014
`define VL53L1X_REF_SPAD_MAN_REF_LOCATION_ADDR                                  16'h0015
`define VL53L1X_ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS_HI_ADDR           16'h0016
`define VL53L1X_ALGO_CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS_LO_ADDR           16'h0017
`define VL53L1X_ALGO_CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS_HI_ADDR       16'h0018
`define VL53L1X_ALGO_CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS_LO_ADDR       16'h0019
`define VL53L1X_ALGO_CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS_HI_ADDR       16'h001A
`define VL53L1X_ALGO_CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS_LO_ADDR       16'h001B
`define VL53L1X_REF_SPAD_CHAR_TOTAL_RATE_TARGET_MCPS_HI_ADDR                    16'h001C
`define VL53L1X_REF_SPAD_CHAR_TOTAL_RATE_TARGET_MCPS_LO_ADDR                    16'h001D
`define VL53L1X_ALGO_PART_TO_PART_RANGE_OFFSET_MM_HI_ADDR                       16'h001E
`define VL53L1X_ALGO_PART_TO_PART_RANGE_OFFSET_MM_LO_ADDR                       16'h001F
`define VL53L1X_MM_CONFIG_INNER_OFFSET_MM_HI_ADDR                               16'h0020
`define VL53L1X_MM_CONFIG_INNER_OFFSET_MM_LO_ADDR                               16'h0021
`define VL53L1X_MM_CONFIG_OUTER_OFFSET_MM_HI_ADDR                               16'h0022
`define VL53L1X_MM_CONFIG_OUTER_OFFSET_MM_LO_ADDR                               16'h0023
`define VL53L1X_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_HI_ADDR                       16'h0024
`define VL53L1X_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_LO_ADDR                       16'h0025
`define VL53L1X_DEBUG_CTRL_ADDR                                                 16'h0026
`define VL53L1X_TEST_MODE_CTRL_ADDR                                             16'h0027
`define VL53L1X_CLK_GATING_CTRL_ADDR                                            16'h0028
`define VL53L1X_NVM_BIST_CTRL_ADDR                                              16'h0029
`define VL53L1X_NVM_BIST_NUM_NVM_WORDS_ADDR                                     16'h002A
`define VL53L1X_NVM_BIST_START_ADDRESS_ADDR                                     16'h002B
`define VL53L1X_HOST_IF_STATUS_ADDR                                             16'h002C
`define VL53L1X_PAD_I2C_HV_CONFIG_ADDR                                          16'h002D
`define VL53L1X_PAD_I2C_HV_EXTSUP_CONFIG_ADDR                                   16'h002E
`define VL53L1X_GPIO_HV_PAD_CTRL_ADDR                                           16'h002F
`define VL53L1X_GPIO_HV_MUX_CTRL_ADDR                                           16'h0030
`define VL53L1X_GPIO_TIO_HV_STATUS_ADDR                                         16'h0031
`define VL53L1X_GPIO_FIO_HV_STATUS_ADDR                                         16'h0032
`define VL53L1X_ANA_CONFIG_SPAD_SEL_PSWIDTH_ADDR                                16'h0033
`define VL53L1X_ANA_CONFIG_VCSEL_PULSE_WIDTH_OFFSET_ADDR                        16'h0034
`define VL53L1X_ANA_CONFIG_FAST_OSC_CONFIG_CTRL_ADDR                            16'h0035
`define VL53L1X_SIGMA_ESTIMATOR_EFFECTIVE_PULSE_WIDTH_NS_ADDR                   16'h0036
`define VL53L1X_SIGMA_ESTIMATOR_EFFECTIVE_AMBIENT_WIDTH_NS_ADDR                 16'h0037
`define VL53L1X_SIGMA_ESTIMATOR_SIGMA_REF_MM_ADDR                               16'h0038
`define VL53L1X_ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT_MM_ADDR                16'h0039
`define VL53L1X_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_0_ADDR                    16'h003A
`define VL53L1X_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_1_ADDR                    16'h003B
`define VL53L1X_ALGO_RANGE_IGNORE_THRESHOLD_MCPS_HI_ADDR                        16'h003C
`define VL53L1X_ALGO_RANGE_IGNORE_THRESHOLD_MCPS_LO_ADDR                        16'h003D
`define VL53L1X_ALGO_RANGE_IGNORE_VALID_HEIGHT_MM_ADDR                          16'h003E
`define VL53L1X_ALGO_RANGE_MIN_CLIP_ADDR                                        16'h003F
`define VL53L1X_ALGO_CONSISTENCY_CHECK_TOLERANCE_ADDR                           16'h0040
`define VL53L1X_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_2_ADDR                    16'h0041
`define VL53L1X_SD_CONFIG_RESET_STAGES_MSB_ADDR                                 16'h0042
`define VL53L1X_SD_CONFIG_RESET_STAGES_LSB_ADDR                                 16'h0043
`define VL53L1X_GPH_CONFIG_STREAM_COUNT_UPDATE_VALUE_ADDR                       16'h0044
`define VL53L1X_GLOBAL_CONFIG_STREAM_DIVIDER_ADDR                               16'h0045
`define VL53L1X_SYSTEM_INTERRUPT_CONFIG_GPIO_ADDR                               16'h0046
`define VL53L1X_CAL_CONFIG_VCSEL_START_ADDR                                     16'h0047
`define VL53L1X_CAL_CONFIG_REPEAT_RATE_HI_ADDR                                  16'h0048
`define VL53L1X_CAL_CONFIG_REPEAT_RATE_LO_ADDR                                  16'h0049
`define VL53L1X_GLOBAL_CONFIG_VCSEL_WIDTH_ADDR                                  16'h004A
`define VL53L1X_PHASECAL_CONFIG_TIMEOUT_MACROP_ADDR                             16'h004B
`define VL53L1X_PHASECAL_CONFIG_TARGET_ADDR                                     16'h004C
`define VL53L1X_PHASECAL_CONFIG_OVERRIDE_ADDR                                   16'h004D
`define VL53L1X_UNNAMED_REG_0x004E_ADDR                                         16'h004E
`define VL53L1X_DSS_CONFIG_ROI_MODE_CONTROL_ADDR                                16'h004F
`define VL53L1X_SYSTEM_THRESH_RATE_HIGH_HI_ADDR                                 16'h0050
`define VL53L1X_SYSTEM_THRESH_RATE_HIGH_LO_ADDR                                 16'h0051
`define VL53L1X_SYSTEM_THRESH_RATE_LOW_HI_ADDR                                  16'h0052
`define VL53L1X_SYSTEM_THRESH_RATE_LOW_LO_ADDR                                  16'h0053
`define VL53L1X_DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT_HI_ADDR                16'h0054
`define VL53L1X_DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT_LO_ADDR                16'h0055
`define VL53L1X_DSS_CONFIG_MANUAL_BLOCK_SELECT_ADDR                             16'h0056
`define VL53L1X_DSS_CONFIG_APERTURE_ATTENUATION_ADDR                            16'h0057
`define VL53L1X_DSS_CONFIG_MAX_SPADS_LIMIT_ADDR                                 16'h0058
`define VL53L1X_DSS_CONFIG_MIN_SPADS_LIMIT_ADDR                                 16'h0059
`define VL53L1X_MM_CONFIG_TIMEOUT_MACROP_A_HI_ADDR                              16'h005A
`define VL53L1X_MM_CONFIG_TIMEOUT_MACROP_A_LO_ADDR                              16'h005B
`define VL53L1X_MM_CONFIG_TIMEOUT_MACROP_B_HI_ADDR                              16'h005C
`define VL53L1X_MM_CONFIG_TIMEOUT_MACROP_B_LO_ADDR                              16'h005D
`define VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_A_HI_ADDR                           16'h005E
`define VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_A_LO_ADDR                           16'h005F
`define VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_A_ADDR                                16'h0060
`define VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_B_HI_ADDR                           16'h0061
`define VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_B_LO_ADDR                           16'h0062
`define VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_B_ADDR                                16'h0063
`define VL53L1X_RANGE_CONFIG_SIGMA_THRESH_HI_ADDR                               16'h0064
`define VL53L1X_RANGE_CONFIG_SIGMA_THRESH_LO_ADDR                               16'h0065
`define VL53L1X_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI_ADDR              16'h0066
`define VL53L1X_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO_ADDR              16'h0067
`define VL53L1X_RANGE_CONFIG_VALID_PHASE_LOW_ADDR                               16'h0068
`define VL53L1X_RANGE_CONFIG_VALID_PHASE_HIGH_ADDR                              16'h0069
`define VL53L1X_UNNAMED_REG_0x006A_ADDR                                         16'h006A
`define VL53L1X_UNNAMED_REG_0x006B_ADDR                                         16'h006B
`define VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD_3_ADDR                           16'h006C
`define VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD_2_ADDR                           16'h006D
`define VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD_1_ADDR                           16'h006E
`define VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD_0_ADDR                           16'h006F
`define VL53L1X_SYSTEM_FRACTIONAL_ENABLE_ADDR                                   16'h0070
`define VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD_0_ADDR                            16'h0071
`define VL53L1X_SYSTEM_THRESH_HIGH_HI_ADDR                                      16'h0072
`define VL53L1X_SYSTEM_THRESH_HIGH_LO_ADDR                                      16'h0073
`define VL53L1X_SYSTEM_THRESH_LOW_HI_ADDR                                       16'h0074
`define VL53L1X_SYSTEM_THRESH_LOW_LO_ADDR                                       16'h0075
`define VL53L1X_SYSTEM_ENABLE_XTALK_PER_QUADRANT_ADDR                           16'h0076
`define VL53L1X_SYSTEM_SEED_CONFIG_ADDR                                         16'h0077
`define VL53L1X_SD_CONFIG_WOI_SD0_ADDR                                          16'h0078
`define VL53L1X_SD_CONFIG_WOI_SD1_ADDR                                          16'h0079
`define VL53L1X_SD_CONFIG_INITIAL_PHASE_SD0_ADDR                                16'h007A
`define VL53L1X_SD_CONFIG_INITIAL_PHASE_SD1_ADDR                                16'h007B
`define VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD_1_ADDR                            16'h007C
`define VL53L1X_SD_CONFIG_FIRST_ORDER_SELECT_ADDR                               16'h007D
`define VL53L1X_SD_CONFIG_QUANTIFIER_ADDR                                       16'h007E
`define VL53L1X_ROI_CONFIG_USER_ROI_CENTRE_SPAD_ADDR                            16'h007F
`define VL53L1X_ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE_ADDR               16'h0080
`define VL53L1X_SYSTEM_SEQUENCE_CONFIG_ADDR                                     16'h0081
`define VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD_ADDR                              16'h0082
`define VL53L1X_POWER_MANAGEMENT_GO1_POWER_FORCE_ADDR                           16'h0083
`define VL53L1X_SYSTEM_STREAM_COUNT_CTRL_ADDR                                   16'h0084
`define VL53L1X_FIRMWARE_ENABLE_ADDR                                            16'h0085
`define VL53L1X_SYSTEM_INTERRUPT_CLEAR_ADDR                                     16'h0086
`define VL53L1X_SYSTEM_MODE_START_ADDR                                          16'h0087
`define VL53L1X_RESULT_RANGE_STATUS_ADDR                                        16'h0089
`define VL53L1X_RESULT_DSS_ACTUAL_EFFECTIVE_SPADS_SD0_ADDR                      16'h008C
`define VL53L1X_RESULT_AMBIENT_COUNT_RATE_MCPS_SD_ADDR                          16'h0090
`define VL53L1X_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_ADDR              16'h0096
`define VL53L1X_RESULT_PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_ADDR 16'h0098
`define VL53L1X_RESULT_OSC_CALIBRATE_VAL_ADDR                                   16'h00DE
`define VL53L1X_ANA_CONFIG_POWERDOWN_GO1_ADDR                                   16'h00E0
`define VL53L1X_FIRMWARE_SYSTEM_STATUS_ADDR                                     16'h00E5
`define VL53L1X_INTERRUPT_MANAGER_ENABLES_ADDR                                  16'h00FD
`define VL53L1X_IDENTIFICATION_MODEL_ID_ADDR                                    16'h010F // has the value 0xEACC
`define VL53L1X_ROI_CONFIG_MODE_ROI_CENTRE_SPAD_ADDR                            16'h013E


`define VL53L1X_IDENTIFICATION_MODEL_ID                                         16'hEACC     //  Chip ID of VL53L1X
`define VL53L1X_SLAVE_ADDRESS                                                   7'b101001    //  VL53L1X SLAVE address 0x29


`define VL53L1X_DATA_RX_BYTE_REG_CNT                                            8            //  The number of byte registers used to receive all measurement data
`define VL53L1X_CAL_DATA_REG_CNT                                                91           //  The number of byte registers needed for all calibration data




`define VL53L1X_INIT_VAL_PAD_I2C_HV_CONFIG                                      8'h00  // set bit 2 and 5 to 1 for fast plus mode (1MHz I2C) or 0x00 for regular (400 KHz or 50 KHz) 
`define VL53L1X_INIT_VAL_PAD_I2C_HV_EXTSUP_CONFIG                               8'h00  // bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) 
`define VL53L1X_INIT_VAL_GPIO_HV_PAD_CTRL                                       8'h00  // bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)  
`define VL53L1X_INIT_VAL_GPIO_HV_MUX_CTRL                                       8'h01  // set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1) 
`define VL53L1X_INIT_VAL_GPIO_TIO_HV_STATUS                                     8'h02  // bit 1 high = interrupt depending on the polarity
`define VL53L1X_INIT_VAL_GPIO_FIO_HV_STATUS                                     8'h00 
`define VL53L1X_INIT_VAL_ANA_CONFIG_SPAD_SEL_PSWIDTH                            8'h02 
`define VL53L1X_INIT_VAL_ANA_CONFIG_VCSEL_PULSE_WIDTH_OFFSET                    8'h08 
`define VL53L1X_INIT_VAL_ANA_CONFIG_FAST_OSC_CONFIG_CTRL                        8'h00 
`define VL53L1X_INIT_VAL_SIGMA_ESTIMATOR_EFFECTIVE_PULSE_WIDTH_NS               8'h08 
`define VL53L1X_INIT_VAL_SIGMA_ESTIMATOR_EFFECTIVE_AMBIENT_WIDTH_NS             8'h10 
`define VL53L1X_INIT_VAL_SIGMA_ESTIMATOR_SIGMA_REF_MM                           8'h01 
`define VL53L1X_INIT_VAL_ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT_MM            8'h01 
`define VL53L1X_INIT_VAL_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_0                8'h00 
`define VL53L1X_INIT_VAL_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_1                8'h00 
`define VL53L1X_INIT_VAL_ALGO_RANGE_IGNORE_THRESHOLD_MCPS_HI                    8'h00 
`define VL53L1X_INIT_VAL_ALGO_RANGE_IGNORE_THRESHOLD_MCPS_LO                    8'h00 
`define VL53L1X_INIT_VAL_ALGO_RANGE_IGNORE_VALID_HEIGHT_MM                      8'hFF 
`define VL53L1X_INIT_VAL_ALGO_RANGE_MIN_CLIP                                    8'h00 
`define VL53L1X_INIT_VAL_ALGO_CONSISTENCY_CHECK_TOLERANCE                       8'h0F 
`define VL53L1X_INIT_VAL_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_2                8'h00 
`define VL53L1X_INIT_VAL_SD_CONFIG_RESET_STAGES_MSB                             8'h00 
`define VL53L1X_INIT_VAL_SD_CONFIG_RESET_STAGES_LSB                             8'h00 
`define VL53L1X_INIT_VAL_GPH_CONFIG_STREAM_COUNT_UPDATE_VALUE                   8'h00 
`define VL53L1X_INIT_VAL_GLOBAL_CONFIG_STREAM_DIVIDER                           8'h00 
`define VL53L1X_INIT_VAL_SYSTEM_INTERRUPT_CONFIG_GPIO                           8'h20  // interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready
`define VL53L1X_INIT_VAL_CAL_CONFIG_VCSEL_START                                 8'h0B 
`define VL53L1X_INIT_VAL_CAL_CONFIG_REPEAT_RATE_HI                              8'h00 
`define VL53L1X_INIT_VAL_CAL_CONFIG_REPEAT_RATE_LO                              8'h00 
`define VL53L1X_INIT_VAL_GLOBAL_CONFIG_VCSEL_WIDTH                              8'h02 
`define VL53L1X_INIT_VAL_PHASECAL_CONFIG_TIMEOUT_MACROP                         8'h0A 
`define VL53L1X_INIT_VAL_PHASECAL_CONFIG_TARGET                                 8'h21 
`define VL53L1X_INIT_VAL_PHASECAL_CONFIG_OVERRIDE                               8'h00 
`define VL53L1X_INIT_VAL_UNNAMED_REG_0x004E                                     8'h00  
`define VL53L1X_INIT_VAL_DSS_CONFIG_ROI_MODE_CONTROL                            8'h05 
`define VL53L1X_INIT_VAL_SYSTEM_THRESH_RATE_HIGH_HI                             8'h00 
`define VL53L1X_INIT_VAL_SYSTEM_THRESH_RATE_HIGH_LO                             8'h00 
`define VL53L1X_INIT_VAL_SYSTEM_THRESH_RATE_LOW_HI                              8'h00 
`define VL53L1X_INIT_VAL_SYSTEM_THRESH_RATE_LOW_LO                              8'h00 
`define VL53L1X_INIT_VAL_DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT_HI            8'hC8 
`define VL53L1X_INIT_VAL_DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT_LO            8'h00 
`define VL53L1X_INIT_VAL_DSS_CONFIG_MANUAL_BLOCK_SELECT                         8'h00 
`define VL53L1X_INIT_VAL_DSS_CONFIG_APERTURE_ATTENUATION                        8'h38 
`define VL53L1X_INIT_VAL_DSS_CONFIG_MAX_SPADS_LIMIT                             8'hFF 
`define VL53L1X_INIT_VAL_DSS_CONFIG_MIN_SPADS_LIMIT                             8'h01 
`define VL53L1X_INIT_VAL_MM_CONFIG_TIMEOUT_MACROP_A_HI                          8'h00 
`define VL53L1X_INIT_VAL_MM_CONFIG_TIMEOUT_MACROP_A_LO                          8'h08 
`define VL53L1X_INIT_VAL_MM_CONFIG_TIMEOUT_MACROP_B_HI                          8'h00 
`define VL53L1X_INIT_VAL_MM_CONFIG_TIMEOUT_MACROP_B_LO                          8'h00 
`define VL53L1X_INIT_VAL_RANGE_CONFIG_TIMEOUT_MACROP_A_HI                       8'h01 
`define VL53L1X_INIT_VAL_RANGE_CONFIG_TIMEOUT_MACROP_A_LO                       8'hDB 
`define VL53L1X_INIT_VAL_RANGE_CONFIG_VCSEL_PERIOD_A                            8'h0F 
`define VL53L1X_INIT_VAL_RANGE_CONFIG_TIMEOUT_MACROP_B_HI                       8'h01 
`define VL53L1X_INIT_VAL_RANGE_CONFIG_TIMEOUT_MACROP_B_LO                       8'hF1 
`define VL53L1X_INIT_VAL_RANGE_CONFIG_VCSEL_PERIOD_B                            8'h0D 
`define VL53L1X_INIT_VAL_RANGE_CONFIG_SIGMA_THRESH_HI                           8'h01  // Sigma threshold MSB (mm in 14.2 format for MSB+LSB), default value 90 mm  
`define VL53L1X_INIT_VAL_RANGE_CONFIG_SIGMA_THRESH_LO                           8'h68  // Sigma threshold LSB  
`define VL53L1X_INIT_VAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI          8'h00  // Min count Rate MSB (MCPS in 9.7 format for MSB+LSB) 
`define VL53L1X_INIT_VAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO          8'h80  // Min count Rate LSB  
`define VL53L1X_INIT_VAL_RANGE_CONFIG_VALID_PHASE_LOW                           8'h08 
`define VL53L1X_INIT_VAL_RANGE_CONFIG_VALID_PHASE_HIGH                          8'hB8 
`define VL53L1X_INIT_VAL_UNNAMED_REG_0x006A                                     8'h00 
`define VL53L1X_INIT_VAL_UNNAMED_REG_0x006B                                     8'h00 
`define VL53L1X_INIT_VAL_SYSTEM_INTERMEASUREMENT_PERIOD_3                       8'h00  // Intermeasurement period MSB, 32 bits register  
`define VL53L1X_INIT_VAL_SYSTEM_INTERMEASUREMENT_PERIOD_2                       8'h00  // Intermeasurement period  
`define VL53L1X_INIT_VAL_SYSTEM_INTERMEASUREMENT_PERIOD_1                       8'h0F  // Intermeasurement period  
`define VL53L1X_INIT_VAL_SYSTEM_INTERMEASUREMENT_PERIOD_0                       8'h89  // Intermeasurement period LSB  
`define VL53L1X_INIT_VAL_SYSTEM_FRACTIONAL_ENABLE                               8'h00 
`define VL53L1X_INIT_VAL_SYSTEM_GROUPED_PARAMETER_HOLD_0                        8'h00 
`define VL53L1X_INIT_VAL_SYSTEM_THRESH_HIGH_HI                                  8'h00  // distance threshold high MSB (in mm, MSB+LSB) 
`define VL53L1X_INIT_VAL_SYSTEM_THRESH_HIGH_LO                                  8'h00  // distance threshold high LSB  
`define VL53L1X_INIT_VAL_SYSTEM_THRESH_LOW_HI                                   8'h00  // distance threshold low MSB ( in mm, MSB+LSB) 
`define VL53L1X_INIT_VAL_SYSTEM_THRESH_LOW_LO                                   8'h00  // distance threshold low LSB  
`define VL53L1X_INIT_VAL_SYSTEM_ENABLE_XTALK_PER_QUADRANT                       8'h00 
`define VL53L1X_INIT_VAL_SYSTEM_SEED_CONFIG                                     8'h01 
`define VL53L1X_INIT_VAL_SD_CONFIG_WOI_SD0                                      8'h0F 
`define VL53L1X_INIT_VAL_SD_CONFIG_WOI_SD1                                      8'h0D 
`define VL53L1X_INIT_VAL_SD_CONFIG_INITIAL_PHASE_SD0                            8'h0E 
`define VL53L1X_INIT_VAL_SD_CONFIG_INITIAL_PHASE_SD1                            8'h0E 
`define VL53L1X_INIT_VAL_SYSTEM_GROUPED_PARAMETER_HOLD_1                        8'h00 
`define VL53L1X_INIT_VAL_SD_CONFIG_FIRST_ORDER_SELECT                           8'h00 
`define VL53L1X_INIT_VAL_SD_CONFIG_QUANTIFIER                                   8'h02 
`define VL53L1X_INIT_VAL_ROI_CONFIG_USER_ROI_CENTRE_SPAD                        8'hC7  // ROI center 
`define VL53L1X_INIT_VAL_ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE           8'hFF  // XY ROI (X=Width, Y=Height) 
`define VL53L1X_INIT_VAL_SYSTEM_SEQUENCE_CONFIG                                 8'h9B 
`define VL53L1X_INIT_VAL_SYSTEM_GROUPED_PARAMETER_HOLD                          8'h00 
`define VL53L1X_INIT_VAL_POWER_MANAGEMENT_GO1_POWER_FORCE                       8'h01  // POWER_MANAGEMENT_GO1_POWER_FORCE enable   
`define VL53L1X_INIT_VAL_SYSTEM_STREAM_COUNT_CTRL                               8'h00 
`define VL53L1X_INIT_VAL_FIRMWARE_ENABLE                                        8'h01 
`define VL53L1X_INIT_VAL_SYSTEM_INTERRUPT_CLEAR                                 8'h00  // clear interrupt, write 0x01 here
`define VL53L1X_INIT_VAL_SYSTEM_MODE_START                                      8'h00  // start/stop ranging 0x40 starts and 0x00 stops

`define VL53L1X_CAL_PAD_I2C_HV_CONFIG_INDEX                                     'd22
`define VL53L1X_CAL_PAD_I2C_HV_EXTSUP_CONFIG_INDEX                              'd23
`define VL53L1X_CAL_GPIO_HV_PAD_CTRL_INDEX                                      'd24
`define VL53L1X_CAL_GPIO_HV_MUX_CTRL_INDEX                                      'd25
`define VL53L1X_CAL_GPIO_TIO_HV_STATUS_INDEX                                    'd26
`define VL53L1X_CAL_GPIO_FIO_HV_STATUS_INDEX                                    'd27
`define VL53L1X_CAL_ANA_CONFIG_SPAD_SEL_PSWIDTH_INDEX                           'd28
`define VL53L1X_CAL_ANA_CONFIG_VCSEL_PULSE_WIDTH_OFFSET_INDEX                   'd29
`define VL53L1X_CAL_ANA_CONFIG_FAST_OSC_CONFIG_CTRL_INDEX                       'd30
`define VL53L1X_CAL_SIGMA_ESTIMATOR_EFFECTIVE_PULSE_WIDTH_NS_INDEX              'd31
`define VL53L1X_CAL_SIGMA_ESTIMATOR_EFFECTIVE_AMBIENT_WIDTH_NS_INDEX            'd32
`define VL53L1X_CAL_SIGMA_ESTIMATOR_SIGMA_REF_MM_INDEX                          'd33
`define VL53L1X_CAL_ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT_MM_INDEX           'd34
`define VL53L1X_CAL_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_0_INDEX               'd35
`define VL53L1X_CAL_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_1_INDEX               'd36
`define VL53L1X_CAL_ALGO_RANGE_IGNORE_THRESHOLD_MCPS_HI_INDEX                   'd37
`define VL53L1X_CAL_ALGO_RANGE_IGNORE_THRESHOLD_MCPS_LO_INDEX                   'd38
`define VL53L1X_CAL_ALGO_RANGE_IGNORE_VALID_HEIGHT_MM_INDEX                     'd39
`define VL53L1X_CAL_ALGO_RANGE_MIN_CLIP_INDEX                                   'd40
`define VL53L1X_CAL_ALGO_CONSISTENCY_CHECK_TOLERANCE_INDEX                      'd41
`define VL53L1X_CAL_SPARE_HOST_CONFIG_STATIC_CONFIG_SPARE_2_INDEX               'd42
`define VL53L1X_CAL_SD_CONFIG_RESET_STAGES_MSB_INDEX                            'd43
`define VL53L1X_CAL_SD_CONFIG_RESET_STAGES_LSB_INDEX                            'd44
`define VL53L1X_CAL_GPH_CONFIG_STREAM_COUNT_UPDATE_VALUE_INDEX                  'd45
`define VL53L1X_CAL_GLOBAL_CONFIG_STREAM_DIVIDER_INDEX                          'd46
`define VL53L1X_CAL_SYSTEM_INTERRUPT_CONFIG_GPIO_INDEX                          'd47
`define VL53L1X_CAL_CAL_CONFIG_VCSEL_START_INDEX                                'd48
`define VL53L1X_CAL_CAL_CONFIG_REPEAT_RATE_INDEX                                'd49
`define VL53L1X_CAL_CAL_CONFIG_REPEAT_RATE_HI_INDEX                             'd50
`define VL53L1X_CAL_CAL_CONFIG_REPEAT_RATE_LO_INDEX                             'd51
`define VL53L1X_CAL_GLOBAL_CONFIG_VCSEL_WIDTH_INDEX                             'd52
`define VL53L1X_CAL_PHASECAL_CONFIG_TIMEOUT_MACROP_INDEX                        'd53
`define VL53L1X_CAL_PHASECAL_CONFIG_TARGET_INDEX                                'd54
`define VL53L1X_CAL_PHASECAL_CONFIG_OVERRIDE_INDEX                              'd55
`define VL53L1X_CAL_DSS_CONFIG_ROI_MODE_CONTROL_INDEX                           'd56
`define VL53L1X_CAL_SYSTEM_THRESH_RATE_HIGH_HI_INDEX                            'd57
`define VL53L1X_CAL_SYSTEM_THRESH_RATE_HIGH_LO_INDEX                            'd58
`define VL53L1X_CAL_SYSTEM_THRESH_RATE_LOW_HI_INDEX                             'd59
`define VL53L1X_CAL_SYSTEM_THRESH_RATE_LOW_LO_INDEX                             'd60
`define VL53L1X_CAL_DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT_HI_INDEX           'd61
`define VL53L1X_CAL_DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT_LO_INDEX           'd62
`define VL53L1X_CAL_DSS_CONFIG_MANUAL_BLOCK_SELECT_INDEX                        'd63
`define VL53L1X_CAL_DSS_CONFIG_APERTURE_ATTENUATION_INDEX                       'd64
`define VL53L1X_CAL_DSS_CONFIG_MAX_SPADS_LIMIT_INDEX                            'd65
`define VL53L1X_CAL_DSS_CONFIG_MIN_SPADS_LIMIT_INDEX                            'd66
`define VL53L1X_CAL_MM_CONFIG_TIMEOUT_MACROP_A_HI_INDEX                         'd67
`define VL53L1X_CAL_MM_CONFIG_TIMEOUT_MACROP_A_LO_INDEX                         'd68
`define VL53L1X_CAL_MM_CONFIG_TIMEOUT_MACROP_B_HI_INDEX                         'd69
`define VL53L1X_CAL_MM_CONFIG_TIMEOUT_MACROP_B_LO_INDEX                         'd70
`define VL53L1X_CAL_RANGE_CONFIG_TIMEOUT_MACROP_A_HI_INDEX                      'd71
`define VL53L1X_CAL_RANGE_CONFIG_TIMEOUT_MACROP_A_LO_INDEX                      'd72
`define VL53L1X_CAL_RANGE_CONFIG_VCSEL_PERIOD_A_INDEX                           'd73
`define VL53L1X_CAL_RANGE_CONFIG_TIMEOUT_MACROP_B_HI_INDEX                      'd74
`define VL53L1X_CAL_RANGE_CONFIG_TIMEOUT_MACROP_B_LO_INDEX                      'd75
`define VL53L1X_CAL_RANGE_CONFIG_VCSEL_PERIOD_B_INDEX                           'd76
`define VL53L1X_CAL_RANGE_CONFIG_SIGMA_THRESH_HI_INDEX                          'd77
`define VL53L1X_CAL_RANGE_CONFIG_SIGMA_THRESH_LO_INDEX                          'd78
`define VL53L1X_CAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI_INDEX         'd79
`define VL53L1X_CAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO_INDEX         'd80
`define VL53L1X_CAL_RANGE_CONFIG_VALID_PHASE_LOW_INDEX                          'd81
`define VL53L1X_CAL_RANGE_CONFIG_VALID_PHASE_HIGH_INDEX                         'd82
`define VL53L1X_CAL_UNNAMED_REG_0x006A_INDEX                                    'd83
`define VL53L1X_CAL_UNNAMED_REG_0x006B_INDEX                                    'd84
`define VL53L1X_CAL_SYSTEM_INTERMEASUREMENT_PERIOD_3_INDEX                      'd85
`define VL53L1X_CAL_SYSTEM_INTERMEASUREMENT_PERIOD_2_INDEX                      'd86
`define VL53L1X_CAL_SYSTEM_INTERMEASUREMENT_PERIOD_1_INDEX                      'd87
`define VL53L1X_CAL_SYSTEM_INTERMEASUREMENT_PERIOD_0_INDEX                      'd88
`define VL53L1X_CAL_SYSTEM_FRACTIONAL_ENABLE_INDEX                              'd89
`define VL53L1X_CAL_SYSTEM_GROUPED_PARAMETER_HOLD_0_INDEX                       'd90
`define VL53L1X_CAL_SYSTEM_THRESH_HIGH_HI_INDEX                                 'd91
`define VL53L1X_CAL_SYSTEM_THRESH_HIGH_LO_INDEX                                 'd92
`define VL53L1X_CAL_SYSTEM_THRESH_LOW_HI_INDEX                                  'd93
`define VL53L1X_CAL_SYSTEM_THRESH_LOW_LO_INDEX                                  'd94
`define VL53L1X_CAL_SYSTEM_ENABLE_XTALK_PER_QUADRANT_INDEX                      'd95
`define VL53L1X_CAL_SYSTEM_SEED_CONFIG_INDEX                                    'd96
`define VL53L1X_CAL_SD_CONFIG_WOI_SD0_INDEX                                     'd97
`define VL53L1X_CAL_SD_CONFIG_WOI_SD1_INDEX                                     'd98
`define VL53L1X_CAL_SD_CONFIG_INITIAL_PHASE_SD0_INDEX                           'd99
`define VL53L1X_CAL_SD_CONFIG_INITIAL_PHASE_SD1_INDEX                           'd100
`define VL53L1X_CAL_SYSTEM_GROUPED_PARAMETER_HOLD_1_INDEX                       'd101
`define VL53L1X_CAL_SD_CONFIG_FIRST_ORDER_SELECT_INDEX                          'd102
`define VL53L1X_CAL_SD_CONFIG_QUANTIFIER_INDEX                                  'd103
`define VL53L1X_CAL_ROI_CONFIG_USER_ROI_CENTRE_SPAD_INDEX                       'd104
`define VL53L1X_CAL_ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE_INDEX          'd105
`define VL53L1X_CAL_SYSTEM_SEQUENCE_CONFIG_INDEX                                'd106
`define VL53L1X_CAL_SYSTEM_GROUPED_PARAMETER_HOLD_INDEX                         'd107
`define VL53L1X_CAL_POWER_MANAGEMENT_GO1_POWER_FORCE_INDEX                      'd108
`define VL53L1X_CAL_SYSTEM_STREAM_COUNT_CTRL_INDEX                              'd109
`define VL53L1X_CAL_FIRMWARE_ENABLE_INDEX                                       'd110
`define VL53L1X_CAL_SYSTEM_INTERRUPT_CLEAR_INDEX                                'd111
`define VL53L1X_CAL_SYSTEM_MODE_START_INDEX                                     'd112


// Location of received data in the data_rx_reg array
`define VL53L1X_DATA_RX_REG_CHIP_ID_HI_INDEX                                    'd0
`define VL53L1X_DATA_RX_REG_CHIP_ID_LO_INDEX                                    'd1
`define VL53L1X_DATA_RX_REG_FIRMWARE_SYSTEM_STATUS_INDEX                        'd2
`define VL53L1X_DATA_RX_REG_DATA_RDY_INDEX                                      'd3
`define VL53L1X_DATA_RX_REG_RESULT_OSC_CAL_VAL_HI_INDEX                         'd4
`define VL53L1X_DATA_RX_REG_RESULT_OSC_CAL_VAL_LO_INDEX                         'd5
`define VL53L1X_DATA_RX_REG_RESULT_RANGE_MEASURE_MM_HI_INDEX                    'd6
`define VL53L1X_DATA_RX_REG_RESULT_RANGE_MEASURE_MM_LO_INDEX                    'd7