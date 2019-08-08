#include <Wire.h>

#define SHUTDOWN_PIN 2

#define SOFT_RESET                                                   0x0000
#define I2C_SLAVE__DEVICE_ADDRESS                                    0x0001
#define VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND                        0x0008
#define VHV_CONFIG__INIT                                             0x000B
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS               0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS           0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS           0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM                           0x001E
#define MM_CONFIG__INNER_OFFSET_MM                                   0x0020
#define MM_CONFIG__OUTER_OFFSET_MM                                   0x0022
#define GPIO_HV_MUX__CTRL                                            0x0030
#define GPIO__TIO_HV_STATUS                                          0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO                                0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP                              0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI                            0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A                                 0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B                                 0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI                            0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO                            0x0062
#define RANGE_CONFIG__SIGMA_THRESH                                   0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS                  0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH                               0x0069
#define SYSTEM__INTERMEASUREMENT_PERIOD                              0x006C
#define SYSTEM__THRESH_HIGH                                          0x0072
#define SYSTEM__THRESH_LOW                                           0x0074
#define SD_CONFIG__WOI_SD0                                           0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0                                 0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD                             0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE                0x0080
#define SYSTEM__SEQUENCE_CONFIG                                      0x0081
#define SYSTEM__GROUPED_PARAMETER_HOLD                               0x0082
#define SYSTEM__INTERRUPT_CLEAR                                      0x0086
#define SYSTEM__MODE_START                                           0x0087
#define RESULT__RANGE_STATUS                                         0x0089
#define RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0                       0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD                           0x0090
#define RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0               0x0096
#define RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0  0x0098
#define RESULT__OSC_CALIBRATE_VAL                                    0x00DE
#define FIRMWARE__SYSTEM_STATUS                                      0x00E5
#define IDENTIFICATION__MODEL_ID                                     0x010F // has the value 0xEACC
#define ROI_CONFIG__MODE_ROI_CENTRE_SPAD                             0x013E
#define DEFAULT_DEVICE_ADDRESS                                       0x52   //1010010


const uint8_t DEFAULT_CONFIGURATION[] = {
0x00,  // 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch  
0x00,  // 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)  
0x00,  // 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)  
0x01,  // 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity()  
0x02,  // 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady()  
0x00,  // 0x32 : not user-modifiable  
0x02,  // 0x33 : not user-modifiable  
0x08,  // 0x34 : not user-modifiable  
0x00,  // 0x35 : not user-modifiable  
0x08,  // 0x36 : not user-modifiable  
0x10,  // 0x37 : not user-modifiable  
0x01,  // 0x38 : not user-modifiable  
0x01,  // 0x39 : not user-modifiable  
0x00,  // 0x3a : not user-modifiable  
0x00,  // 0x3b : not user-modifiable  
0x00,  // 0x3c : not user-modifiable  
0x00,  // 0x3d : not user-modifiable  
0xff,  // 0x3e : not user-modifiable  
0x00,  // 0x3f : not user-modifiable  
0x0F,  // 0x40 : not user-modifiable  
0x00,  // 0x41 : not user-modifiable  
0x00,  // 0x42 : not user-modifiable  
0x00,  // 0x43 : not user-modifiable  
0x00,  // 0x44 : not user-modifiable  
0x00,  // 0x45 : not user-modifiable  
0x20,  // 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC  
0x0b,  // 0x47 : not user-modifiable  
0x00,  // 0x48 : not user-modifiable  
0x00,  // 0x49 : not user-modifiable  
0x02,  // 0x4a : not user-modifiable  
0x0a,  // 0x4b : not user-modifiable  
0x21,  // 0x4c : not user-modifiable  
0x00,  // 0x4d : not user-modifiable  
0x00,  // 0x4e : not user-modifiable  
0x05,  // 0x4f : not user-modifiable  
0x00,  // 0x50 : not user-modifiable  
0x00,  // 0x51 : not user-modifiable  
0x00,  // 0x52 : not user-modifiable  
0x00,  // 0x53 : not user-modifiable  
0xc8,  // 0x54 : not user-modifiable  
0x00,  // 0x55 : not user-modifiable  
0x00,  // 0x56 : not user-modifiable  
0x38,  // 0x57 : not user-modifiable  
0xff,  // 0x58 : not user-modifiable  
0x01,  // 0x59 : not user-modifiable  
0x00,  // 0x5a : not user-modifiable  
0x08,  // 0x5b : not user-modifiable  
0x00,  // 0x5c : not user-modifiable  
0x00,  // 0x5d : not user-modifiable  
0x01,  // 0x5e : not user-modifiable  
0xdb,  // 0x5f : not user-modifiable  
0x0f,  // 0x60 : not user-modifiable  
0x01,  // 0x61 : not user-modifiable  
0xf1,  // 0x62 : not user-modifiable  
0x0d,  // 0x63 : not user-modifiable  
0x01,  // 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm   
0x68,  // 0x65 : Sigma threshold LSB  
0x00,  // 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold()  
0x80,  // 0x67 : Min count Rate LSB  
0x08,  // 0x68 : not user-modifiable  
0xb8,  // 0x69 : not user-modifiable  
0x00,  // 0x6a : not user-modifiable  
0x00,  // 0x6b : not user-modifiable  
0x00,  // 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs()  
0x00,  // 0x6d : Intermeasurement period  
0x0f,  // 0x6e : Intermeasurement period  
0x89,  // 0x6f : Intermeasurement period LSB  
0x00,  // 0x70 : not user-modifiable  
0x00,  // 0x71 : not user-modifiable  
0x00,  // 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold()  
0x00,  // 0x73 : distance threshold high LSB  
0x00,  // 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold()  
0x00,  // 0x75 : distance threshold low LSB  
0x00,  // 0x76 : not user-modifiable  
0x01,  // 0x77 : not user-modifiable  
0x0f,  // 0x78 : not user-modifiable  
0x0d,  // 0x79 : not user-modifiable  
0x0e,  // 0x7a : not user-modifiable  
0x0e,  // 0x7b : not user-modifiable  
0x00,  // 0x7c : not user-modifiable  
0x00,  // 0x7d : not user-modifiable  
0x02,  // 0x7e : not user-modifiable  
0xc7,  // 0x7f : ROI center, use SetROI()  
0xff,  // 0x80 : XY ROI (X=Width, Y=Height), use SetROI()  
0x9B,  // 0x81 : not user-modifiable  
0x00,  // 0x82 : not user-modifiable  
0x00,  // 0x83 : not user-modifiable  
0x00,  // 0x84 : not user-modifiable  
0x01,  // 0x85 : not user-modifiable  
0x00,  // 0x86 : clear interrupt, use ClearInterrupt()  
0x00   // 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after init() call, put 0x40 in location 0x87  

};

int Init(void);
int WaitFirmwareStat(void);
int StartMeasure(void);
int GetMeasurement(uint16_t*);
int StopMeasure(void);
int PollReady(void);
int ClearInt(void);
int SetTemp(void);
int SetMeasPeriod(void);
int I2CWriteByte(TwoWire * i2c, uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToWrite = 1);
int I2CWriteWord(TwoWire * i2c, uint8_t DeviceAddr, uint16_t RegisterAddr, uint16_t* data);
int I2CWriteDWord(TwoWire * i2c, uint8_t DeviceAddr, uint16_t RegisterAddr, uint32_t* data);
int I2CReadByte(TwoWire * i2c, uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToRead = 1);
int I2CReadWord(TwoWire * i2c, uint8_t DeviceAddr, uint16_t RegisterAddr, uint16_t* data);
int I2CReadDWord(TwoWire * i2c, uint8_t DeviceAddr, uint16_t RegisterAddr, uint32_t* data);
uint8_t I2C_ID     = DEFAULT_DEVICE_ADDRESS;