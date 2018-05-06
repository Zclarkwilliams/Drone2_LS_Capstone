#include <Wire.h>

#define I2C_ADDRESS 0x29


volatile uint8_t data_array[46];
uint8_t index = 0;
volatile bool lock = false;

int16_t join_bytes(uint8_t byte1, uint8_t byte2, double divisor){
  return ((((int16_t)byte2<<8)|(int16_t)byte1)/divisor);
}

void setup() {
  index = 0;
  Wire.begin(I2C_ADDRESS);      // join i2c bus with address I2C_ADDRESS
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(115200);           // start serial for output
  Serial.println("Ready.... Go!");
}

void loop() {
  while(lock) ; //Spin here until unlocked
    
  lock = true;
  //Serial.print("Accelerometer X Dps:");Serial.print((double)join_bytes(data_array[0], data_array[1],100.0));
  //Serial.print(", Accelerometer Y Dps:");Serial.print((double)join_bytes(data_array[2], data_array[3],100.0));
  //Serial.print(", Accelerometer Z Dps:");Serial.print((double)join_bytes(data_array[4], data_array[5],100.0));
  //Serial.print(", Magnetometer X uT:");Serial.print((double)join_bytes(data_array[6], data_array[7],16.0));
  //Serial.print(", Magnetometer Y:uT");Serial.print((double)join_bytes(data_array[8], data_array[9],16.0));
  //Serial.print(", Magnetometer Z uT:");Serial.print((double)join_bytes(data_array[10], data_array[11],16.0));
  //Serial.print(", Gyroscope X Dps:");Serial.print((double)join_bytes(data_array[12], data_array[13],16.0));
  //Serial.print(", Gyroscope Y Dps:");Serial.print((double)join_bytes(data_array[14], data_array[15],16.0));
  //Serial.print(", Gyroscope Z Dps:");Serial.print((double)join_bytes(data_array[16], data_array[17],16.0));
//  **** NOTE ****
//  These 3 sets of registers do not follow the same sequence as other registers, instead of X, Y, Z, these are sequenced, Z, X, Y
  Serial.print("Euler Angle Roll° (X-Axis):");Serial.print((double)join_bytes(data_array[20], data_array[21],16.0));
  Serial.print(", Euler Angle Pitch° (Y-Axis):");Serial.print((double)join_bytes(data_array[22], data_array[23],16.0));
  Serial.print(", Euler Angle Yaw° (Z-Axis):");Serial.print((double)join_bytes(data_array[18], data_array[19],16.0));
//  **** /END NOTE ****
  //Serial.print(", Quaternion W:");Serial.print((double)join_bytes(data_array[24], data_array[25],(2^14.0)));
  //Serial.print(", Quaternion X:");Serial.print((double)join_bytes(data_array[26], data_array[27],(2^14.0)));
  //Serial.print(", Quaternion Y:");Serial.print((double)join_bytes(data_array[28], data_array[29],(2^14.0)));
  //Serial.print(", Quaternion Z:");Serial.print((double)join_bytes(data_array[30], data_array[31],(2^14.0)));
  Serial.print(", Linear Accel X:");Serial.print((double)join_bytes(data_array[32], data_array[33],100.0));
  Serial.print(", Linear Accel Y:");Serial.print((double)join_bytes(data_array[34], data_array[35],100.0));
  Serial.print(", Linear Accel Z:");Serial.print((double)join_bytes(data_array[36], data_array[37],100.0));
  //Serial.print(", Gravity Accel X:");Serial.print((double)join_bytes(data_array[38], data_array[39],100.0));
  //Serial.print(", Gravity Accel Y:");Serial.print((double)join_bytes(data_array[40], data_array[41],100.0));
  //Serial.print(", Gravity Accel Z:");Serial.print((double)join_bytes(data_array[42], data_array[43],100.0));
  //Serial.print(", Temperature (C°):");Serial.print(data_array[44]);
  Serial.print(", Calibration Status:");Serial.print(data_array[45]);
  Serial.println("");
  lock = false;
  delay(100); //Delay 1s, then print again
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  uint8_t x;
  while (Wire.available()) {
    lock = true;
    index = Wire.read();
    x = Wire.read();
    data_array[index] = x;
    lock = false;
  }
}
