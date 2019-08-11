#include <Keyboard.h>

#include <Sparkfun-VL53L1-Lidar-ToF-Sensor.h>


#define ENABLE_XSHUT_PIN

//Debugging - disable all #defines for normal operation
//#define PRINT_DEBUG
//#define DEBUG_READ_BACK_ALL_REGS
//#define DEBUG_READ_BACK_STARTUP_REGS
//#define PRINT_DATA_RDY_DEBUG
//#define PRINT_RD_WR_1B_DEBUG
//#define PRINT_RD_WR_2B_DEBUG
//#define PRINT_RD_WR_4B_DEBUG

void setup(void)
{
  int result = Init();
}

void loop(void)
{
  uint16_t range = 0;
  uint8_t result = 0;
  result = PollReady();
  result = GetMeasurement(&range);
  result = ClearInt();
  Serial.print("Range = ");
  Serial.print(range);
  Serial.println(" mm");
  delay(200);
}


int Init(void)
{
  uint16_t Addr = 0x0000;
  uint8_t result;
  int numElements = 91;
  uint16_t deviceID = 0;
  uint8_t data[1];
  int i = 0;

  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial.println("============================================================");
  Serial.println("VL53L1X Test");

#ifdef ENABLE_XSHUT_PIN
  //Ensure device isn't in shutdown mode
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW);
  delay(100);
  digitalWrite(SHUTDOWN_PIN, HIGH);
  delayMicroseconds(2000);
#endif

  //wait for firmware to indicate that it's ready to proceed
  result = WaitFirmwareStat();

  result =  I2CReadWord(&Wire, I2C_ID, IDENTIFICATION__MODEL_ID, &deviceID);
  Serial.print("Read chip ID: ");
  Serial.println(  deviceID, HEX);
  if(deviceID != 0xEACC) {
    Serial.print("Halting driver, incorrect chip ID detected, expected 0xEACC");
    while(1);
  }


  Serial.print("VL53L1X Begin setup, Write configuration to sensor I2C ID ");
  Serial.println(I2C_ID, HEX);
  for (Addr = 0x002D; Addr < (numElements + 0x002D); Addr++) {
    result = I2CWriteByte(&Wire, I2C_ID, Addr, &DEFAULT_CONFIGURATION[Addr - 0x002D], (uint8_t)1);
  }

#if defined DEBUG_READ_BACK_STARTUP_REGS
  Addr = 0x002D;
  Serial.println("Reading back startup values");
  for (Addr = 0x002D; Addr < (numElements + 0x002D); Addr++)
  {
    result = I2CReadByte(&Wire, I2C_ID, Addr, data, (uint8_t)1);
    Serial.print(data[0]);
    Serial.print("@");
    Serial.println(Addr, HEX);
  }
#endif

  result = StartMeasure();
  result = PollReady();
  result = ClearInt();
  result = StopMeasure();
  result = SetTemp();
  result = SetMeasPeriod();
  result = StartMeasure();


#ifdef DEBUG_READ_BACK_ALL_REGS
  Addr = 0x0000;
  Serial.println("Reading back ALL register values");
  for (Addr = 0x0000; Addr < (0x0FFF+1); Addr++) //Debug - Read ALL registers from 0x0000 to 0xFFF
  {
    result = I2CReadByte(&Wire, I2C_ID, Addr, data, (uint8_t)1);
    Serial.print(data[0], HEX);
    Serial.print("@");
    Serial.println(Addr, HEX);
  }
  //while(1); //Spin here forever (stop program)
#endif

  Serial.println("VL53L1X Looper starting...");
  return 0;
}

int StartMeasure(void)
{
  uint8_t result = 0;
  uint8_t valueToWrite = 0x40;
#ifdef PRINT_DEBUG
  Serial.println("Starting measurements");
#endif
  while (!result) {
    result = I2CWriteByte(&Wire, I2C_ID, SYSTEM__MODE_START,  &valueToWrite,  (uint8_t)1);
  }
  return result;
}

int StopMeasure(void)
{
  uint8_t result = 0;
  uint8_t valueToWrite = 0x00;
#ifdef PRINT_DEBUG
  Serial.println("\nStopping measurements");
#endif
  while (!result)
    result = I2CWriteByte(&Wire, I2C_ID, SYSTEM__MODE_START,  &valueToWrite,  (uint8_t)1);
  return result;
}

int GetMeasurement(uint16_t * range)
{
  uint8_t result = 0;
  uint16_t pBuffer[1];
#ifdef PRINT_DEBUG
  Serial.println("Getting measurement");
#endif
  while (!result) {
#ifdef PRINT_DEBUG
    Serial.println("Measurement read");
#endif
    result =  I2CReadWord(&Wire, I2C_ID, RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, pBuffer);
    *range = pBuffer[0];
    //delay(1); //Wait one millisecond
  }
  return result;
}

int PollReady(void)
{
  uint8_t result = 0;
  uint8_t pBuffer[1];
  pBuffer[0] = 0;
#ifdef PRINT_DATA_RDY_DEBUG
  Serial.println("Polling data ready");
#endif
  while (!result || ((pBuffer[0] & 0x01) != 0x01)) {
    pBuffer[0] = 0;
    result =  I2CReadByte(&Wire, I2C_ID, GPIO__TIO_HV_STATUS, pBuffer, (uint8_t)1);
    pBuffer[0] = pBuffer[0] & 0x01;
#ifdef PRINT_DATA_RDY_DEBUG
    Serial.print("Data ready poll=");
    Serial.println(pBuffer[0]);
    Serial.print("result=");
    Serial.println(result);
#endif
    //delay(1); //Wait one millisecond
  }
#ifdef PRINT_DATA_RDY_DEBUG
  Serial.print("Good data ready poll=");
  Serial.println(pBuffer[0]);
  Serial.print("result=");
  Serial.println(result);
#endif
  return (pBuffer[0] & 0x01);
}

int ClearInt(void)
{
  uint8_t result = 0;
  uint8_t valueToWrite = 0x01;
#ifdef PRINT_DATA_RDY_DEBUG
  Serial.println("Clearing interrrupts");
#endif
  while (!result) {
    result = I2CWriteByte(&Wire, I2C_ID, SYSTEM__INTERRUPT_CLEAR, &valueToWrite, (uint8_t)1);
    //delay(1); //Wait one millisecond
  }
  return result;
}

int SetTemp(void)
{
  uint16_t result = 0;
  uint8_t valueToWrite;
#ifdef PRINT_DEBUG
  Serial.println("Set temperature");
#endif
  while (!result) {
    valueToWrite = 0x09;
    result = I2CWriteByte(&Wire, VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, &valueToWrite, (uint8_t)1);
    //delay(1); //Wait one millisecond
  }
  result = 0;
  while (!result) {
    valueToWrite = 0x00;
    result = I2CWriteByte(&Wire, I2C_ID,  VHV_CONFIG__INIT, &valueToWrite, (uint8_t)1);
    //delay(1); //Wait one millisecond
  }
  return result;
}

int SetMeasPeriod(void)
{
  uint8_t result = 0;
  uint16_t ClockPLL = 0;
  uint32_t valueToWrite = 0;
#ifdef PRINT_DEBUG
  Serial.println("Set measurement period");
#endif
  while (!result || !ClockPLL) {
    ClockPLL = 0;
    result =  I2CReadWord(&Wire, I2C_ID, RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
    //delay(1); //Wait one millisecond
  }
  result = 0;
  ClockPLL = ClockPLL & 0x03FF;
  Serial.print("ClockPLL = ");
  Serial.println(ClockPLL);
  while (!result) {
    valueToWrite = (ClockPLL * 100 * 1.075);
    result = I2CWriteDWord(&Wire, I2C_ID, SYSTEM__INTERMEASUREMENT_PERIOD, valueToWrite);
    //delay(1); //Wait one millisecond
    Serial.print("Wrote intermeasurement period = ");
    Serial.println(valueToWrite);
  }
  return ClockPLL;
}


int WaitFirmwareStat(void)
{
  int result;
  int status;
  uint8_t pBuffer[1];
#ifdef PRINT_DEBUG
  Serial.println("Check firmware status");
#endif
  do {
    Serial.println("Polling firmware status");
    result =  I2CReadByte(&Wire, I2C_ID, FIRMWARE__SYSTEM_STATUS, pBuffer, (uint8_t)1);
    status = pBuffer[0] & 0x01;
    Serial.print("Firmware system status reg value : 0x");
    Serial.println(pBuffer[0]);
    //delay(1);
  } while (status == 0);
  Serial.println("Firmware initialized\n");
  return 1;
}


int I2CWriteByte(TwoWire * i2c, uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToWrite)
{
  uint8_t result;
  uint8_t buffer[2] = {0, 0};
  buffer[0] = (uint8_t) ((RegisterAddr >> 8) & 0x00FF);
  buffer[1] = (uint8_t) (RegisterAddr & 0x00FF);
  int i = 0;

  DeviceAddr = ((uint8_t)(((DeviceAddr) >> 1) & 0x7F));

#ifdef PRINT_RD_WR_1B_DEBUG
  Serial.println("Write byte transmission started");
#endif
  i2c->beginTransmission(DeviceAddr);
  i2c->write(buffer, 2);
  for (i = 0 ; i < NumByteToWrite ; i++) {
    i2c->write(pBuffer[i]);
#ifdef PRINT_RD_WR_1B_DEBUG
    Serial.print("Wrote 0x");
    Serial.print(pBuffer[i], HEX);
    Serial.print("@ Addr 0x");
    Serial.println(RegisterAddr + i, HEX);
#endif
  }

  i2c->endTransmission(true);
#ifdef PRINT_RD_WR_1B_DEBUG
  Serial.println("Write byte transmission ended");
#endif
  return 1;
}

int I2CWriteWord(TwoWire * i2c, uint8_t DeviceAddr, uint16_t RegisterAddr, uint16_t data)
{
  uint8_t result;
  uint8_t buffer[2] = {0, 0};
  buffer[0] = (uint8_t) ((RegisterAddr >> 8) & 0x00FF);
  buffer[1] = (uint8_t) (RegisterAddr & 0x00FF);
  int i = 0;

  DeviceAddr = ((uint8_t)(((DeviceAddr) >> 1) & 0x7F));

  Serial.println("Write word transmission started");
  i2c->beginTransmission(DeviceAddr);
  i2c->write(buffer, 2);
  buffer[0] = (data >> 8) & 0x00FF;
  buffer[1] =  data    & 0x00FF;
  i2c->write(buffer[0]);
  i2c->write(buffer[1]);

#ifdef PRINT_RD_WR_2B_DEBUG
  Serial.print("Wrote 0x");
  Serial.print(data, HEX);
  Serial.print("@ Addr 0x");
  Serial.println(RegisterAddr + 1, HEX);
#endif

  i2c->endTransmission(true);
#ifdef PRINT_RD_WR_2B_DEBUG
  Serial.println("Write word transmission ended");
#endif
  return 1;
}

int I2CWriteDWord(TwoWire * i2c, uint8_t DeviceAddr, uint16_t RegisterAddr, uint32_t data)
{
  uint8_t result;
  uint8_t buffer[4] = {0, 0, 0, 0};
  buffer[0] = (uint8_t) ((RegisterAddr >> 8) & 0x00FF);
  buffer[1] = (uint8_t) (RegisterAddr & 0x00FF);
  int i = 0;

  DeviceAddr = ((uint8_t)(((DeviceAddr) >> 1) & 0x7F));

  Serial.println("Write Dword transmission started");
  i2c->beginTransmission(DeviceAddr);
  i2c->write(buffer, 2);
  buffer[0] = (data >> 24) & 0x000000FF;
  buffer[1] = (data >> 16) & 0x000000FF;
  buffer[2] = (data >> 8) & 0x000000FF;
  buffer[3] =  data     & 0x000000FF;
  i2c->write(buffer[0]);
  i2c->write(buffer[1]);
  i2c->write(buffer[2]);
  i2c->write(buffer[3]);

#ifdef PRINT_RD_WR_4B_DEBUG
  Serial.print("Wrote 0x");
  Serial.print(data, HEX);
  Serial.print("@ Addr 0x");
  Serial.println(RegisterAddr + 1, HEX);
#endif

  i2c->endTransmission(true);
#ifdef PRINT_RD_WR_4B_DEBUG
  Serial.print("Write Dword transmission ended");
#endif
  return 1;
}


int  I2CReadByte(TwoWire * i2c, uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t* pBuffer, uint8_t NumByteToRead)
{
  uint8_t result;
  uint8_t buffer[2] = {0, 0};
  buffer[0] = (uint8_t) ((RegisterAddr >> 8) & 0xFF);
  buffer[1] = (uint8_t) (RegisterAddr & 0xFF);
  int i = 0;

#ifdef PRINT_RD_WR_1B_DEBUG
  Serial.print("Reading ");
  Serial.print(NumByteToRead);
  Serial.print(" byte");
  if (NumByteToRead > 1)
    Serial.print("s");
  Serial.println();
#endif
  DeviceAddr = ((uint8_t)(((DeviceAddr) >> 1) & 0x7F));

  do {
    i2c->beginTransmission(DeviceAddr);
    i2c->write(buffer, 2);
    result = i2c->endTransmission(false);
  } while (result != 0);

#ifdef PRINT_RD_WR_1B_DEBUG
  Serial.print("@ Addr Starting 0x");
  Serial.println( (buffer[0] << 8) + buffer[1], HEX);
#endif
  i2c->requestFrom(DeviceAddr, (byte) NumByteToRead);

  while (i2c->available())
  {
    pBuffer[i] = i2c->read();
#ifdef PRPRINT_RD_WR_1B_DEBUG
    Serial.print("Read 0x");
    Serial.print(pBuffer[i], HEX);
    Serial.print("@ Addr 0x");
    Serial.println( (buffer[0] << 8) + buffer[1] + i, HEX);
#endif
    i++;
  }
#ifdef PRINT_RD_WR_1B_DEBUG
  Serial.print("Completed read of ");
  Serial.print(NumByteToRead, DEC);
  Serial.print(" byte");
  if (NumByteToRead > 1)
    Serial.print("s");
  Serial.print(" of data ");
  Serial.println(result);
  Serial.println(pBuffer[i - 1], HEX);
#endif
  return 1;
}

int  I2CReadWord(TwoWire * i2c, uint8_t DeviceAddr, uint16_t RegisterAddr, uint16_t* data)
{
  uint8_t result;
  uint8_t buffer[2] = {0, 0};
  uint8_t rdBuffer[2] = {0, 0};
  buffer[0] = (uint8_t) ((RegisterAddr >> 8) & 0x00FF);
  buffer[1] = (uint8_t) (RegisterAddr & 0x00FF);
  int i = 0;

#ifdef PRINT_RD_WR_2B_DEBUG
  Serial.println("Reading 2 bytes");
#endif
  DeviceAddr = ((uint8_t)(((DeviceAddr) >> 1) & 0x7F));

  do {
    i2c->beginTransmission(DeviceAddr);
    i2c->write(buffer, 2);
    result = i2c->endTransmission(false);
  } while (result != 0);
#ifdef PRINT_RD_WR_2B_DEBUG
  Serial.print("@ Addr Starting 0x");
  Serial.println( (buffer[0] << 8) + buffer[1], HEX);
#endif
  i2c->requestFrom(DeviceAddr, (uint8_t)2);

  while (i2c->available())
  {
    rdBuffer[i] = i2c->read();
#ifdef PRINT_RD_WR_2B_DEBUG
    Serial.print("Read 0x");
    Serial.print(rdBuffer[i], HEX);
    Serial.print("@ Addr 0x");
    Serial.println( (buffer[0] << 8) + buffer[1] + i, HEX);
#endif
    i++;
  }
  data[0] = (rdBuffer[0] << 8) + rdBuffer[1];
#ifdef PRINT_RD_WR_2B_DEBUG
  Serial.print("2 byte read final result: 0x");
  Serial.print(data[0], HEX);
  Serial.print("@ Addr 0x");
  Serial.println( RegisterAddr, HEX);
#endif
  return 1;
}

int  I2CReadDWord(TwoWire * i2c, uint8_t DeviceAddr, uint16_t RegisterAddr, uint32_t* data)
{
  uint8_t result;
  uint8_t buffer[2] = {0, 0};
  uint8_t rdBuffer[4] = {0, 0, 0, 0};
  buffer[0] = (uint8_t) ((RegisterAddr >> 8) & 0x00FF);
  buffer[1] = (uint8_t) (RegisterAddr & 0x00FF);
  int i = 0;

#ifdef PRINT_RD_WR_4B_DEBUG
  Serial.println("Reading 4 bytes");
#endif
  DeviceAddr = ((uint8_t)(((DeviceAddr) >> 1) & 0x7F));

  do {
    i2c->beginTransmission(DeviceAddr);
    i2c->write(buffer, 2);
    result = i2c->endTransmission(false);
  } while (result != 0);
#ifdef PRINT_RD_WR_4B_DEBUG
  Serial.print("@ Addr Starting 0x");
  Serial.println( (buffer[0] << 8) + buffer[1], HEX);
#endif
  i2c->requestFrom(rdBuffer, (uint8_t)4);

  while (i2c->available())
  {
    rdBuffer[i] = i2c->read();
#ifdef PRINT_RD_WR_4B_DEBUG
    Serial.print("Read 0x");
    Serial.print(rdBuffer[i], HEX);
    Serial.print("@ Addr 0x");
    Serial.println( (buffer[0] << 8) + buffer[1] + i, HEX);
#endif
    i++;
  }
  data[0] = (rdBuffer[0] << 24) + (rdBuffer[1] << 16) + (rdBuffer[2] << 8) + rdBuffer[3];
#ifdef PRINT_RD_WR_4B_DEBUG
  Serial.print("4 byte read final result: 0x");
  Serial.print(data[0], HEX);
  Serial.print("@ Addr 0x");
  Serial.println( RegisterAddr, HEX);
#endif
  return 1;
}

