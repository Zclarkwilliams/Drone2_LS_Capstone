#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

//Set the delay between fresh samples
#define SAMPLERATE_DELAY_MS (200)
#define SAMPLERATE_DELAY (SAMPLERATE_DELAY_MS/1000)
#define ACCEL_ZERO_LIMIT 0.5

Adafruit_BNO055 bno = Adafruit_BNO055();
float X_Velocity;
float Y_Velocity;
float Z_Velocity;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);
  
/*

   System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms

     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! 

   System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error 

*/

// Expect 0x5, 0xF, 0x0
 
  uint8_t system_status;
  uint8_t self_test_result;
  uint8_t system_error;
  bno.getSystemStatus(&system_status, &self_test_result, &system_error);
  Serial.print(" System Status: ");
  Serial.print(system_status, HEX);
  Serial.print(", Self Test Status");
  Serial.print(self_test_result, HEX);
  Serial.print(", System Error");
  Serial.print(system_error, HEX);
  Serial.println(" ");

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // Display the floating point accelerations
  Serial.print("Accelerometer m/s^2: ");
  Serial.print("X: ");
  Serial.print(accel.x(), DEC);
  Serial.print(" Y: ");
  Serial.print(accel.y(), DEC);
  Serial.print(" Z: ");
  Serial.print(accel.z(), DEC);

//Store initial sensor offset values prior to calibration
  delay(1000);
  adafruit_bno055_offsets_t SensorOffsets;
  bno.getSensorOffsets(SensorOffsets);
  uint16_t init_accel_x_offset = SensorOffsets.accel_offset_x;
  uint16_t init_accel_y_offset = SensorOffsets.accel_offset_y;
  uint16_t init_accel_z_offset = SensorOffsets.accel_offset_z;

  
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  uint8_t system, gyro, accelerometer, mag = 0;
  delay(1000);

  Serial.println("Start sensor calibration");
  bno.getCalibration(&system, &gyro, &accelerometer, &mag);

  while(!bno.isFullyCalibrated()) {
    bno.getCalibration(&system, &gyro, &accelerometer, &mag);
    Serial.print("    CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accelerometer, DEC);
    Serial.print(" Mag=");
    Serial.print(mag, DEC);
  
    //Display the floating point angles
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("    Euler Angles: ");
    Serial.print("X: ");
    Serial.print(euler.x());
    Serial.print(" Y: ");
    Serial.print(euler.y());
    Serial.print(" Z: ");
    Serial.print(euler.z());
    Serial.println("    ");
  
    delay(SAMPLERATE_DELAY_MS);
  }
  

  delay(1000);
  Serial.println("Sensor calibration completed");
  delay(1000);

  bno.getSensorOffsets(SensorOffsets);

  Serial.print("Initial Sensor Offsets:");
  Serial.print(" Accel X=");
  Serial.print(init_accel_x_offset, DEC);
  Serial.print(" Accel Y=");
  Serial.print(init_accel_y_offset, DEC);
  Serial.print(" Accel Z=");
  Serial.println(init_accel_z_offset, DEC);

  Serial.print("Final Sensor Offsets:");
  Serial.print(" Accel X=");
  Serial.print(SensorOffsets.accel_offset_x, DEC);
  Serial.print(" Accel Y=");
  Serial.print(SensorOffsets.accel_offset_y, DEC);
  Serial.print(" Accel Z=");
  Serial.println(SensorOffsets.accel_offset_z, DEC);

  delay(1000);
  
  Serial.println("Place sensor flat on surface and stop movement");
  delay(1000);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float accelX = accel.x();
  float accelY = accel.y();
  float accelZ = accel.z();
  // Display the floating point accelerations
  Serial.print("Accelerometer m/s^2: ");
  Serial.print("X: ");
  Serial.print(accel.x(), DEC);
  Serial.print(" Y: ");
  Serial.print(accel.y(), DEC);
  Serial.print(" Z: ");
  Serial.println(accel.z(), DEC);

  delay(1000);
  
  while( ((abs(accelX) > ACCEL_ZERO_LIMIT) == true) || ((abs(accelY) > ACCEL_ZERO_LIMIT) == true) || ((abs(accelZ) > ACCEL_ZERO_LIMIT) == true) ) {
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    accelX = accel.x();
    accelY = accel.y();
    accelZ = accel.z();
    Serial.print("Still moving...");
    Serial.print("Accelerometer m/s^2: ");
    Serial.print("X: ");
    Serial.print(accelX, DEC);
    Serial.print(" Y: ");
    Serial.print(accelY, DEC);
    Serial.print(" Z: ");
    Serial.print(accelZ, DEC);
    Serial.println(" ");
    delay(1000);    
  }

  X_Velocity = 0;
  Y_Velocity = 0;
  Z_Velocity = 0;

  delay(1000);
  
  Serial.println("Set 0 Velocity");

  accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // Display the floating point accelerations
  Serial.print("Accelerometer m/s^2: ");
  Serial.print("X: ");
  Serial.print(accel.x(), DEC);
  Serial.print(" Y: ");
  Serial.print(accel.y(), DEC);
  Serial.print(" Z: ");
  Serial.print(accel.z(), DEC);

}

void updateVelocity(float * xVel, float * yVel, float * zVel, imu::Vector<3> accel) {
  #define ACCEL_ZERO_LIMIT 0.5
  float xAccel = 0;
  float yAccel = 0;
  float zAccel = 0;
  if ( abs( accel.x() ) > ACCEL_ZERO_LIMIT ) xAccel = accel.x();
  if ( abs( accel.y() ) > ACCEL_ZERO_LIMIT ) yAccel = accel.y();
  if ( abs( accel.z() ) > ACCEL_ZERO_LIMIT ) zAccel = accel.z();
  *xVel = *xVel + xAccel*float(SAMPLERATE_DELAY);
  *yVel = *yVel + yAccel*float(SAMPLERATE_DELAY);
  *zVel = *zVel + zAccel*float(SAMPLERATE_DELAY);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
/*
  //Display the floating point angles
  Serial.print("Euler Angles: ");
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("    ");


  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("    ");
  
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // Display the floating point accelerations
  Serial.print("Accelerometer m/s^2: ");
  Serial.print("X: ");
  Serial.print(accel.x(), DEC);
  Serial.print(" Y: ");
  Serial.print(accel.y(), DEC);
  Serial.print(" Z: ");
  Serial.print(accel.z(), DEC);

  //Display the floating point velocities
  float xVel = 0;
  float yVel = 0;
  float zVel = 0;
  updateVelocity(&xVel, &yVel, &zVel, accel);  


  Serial.print("    Velocity m/s: ");
  Serial.print("X: ");
  Serial.print(xVel);
  Serial.print(" Y: ");
  Serial.print(yVel);
  Serial.print(" Z: ");
  Serial.print(zVel);
  //Display calibration status for each sensor.

  uint8_t system, gyro, accelerometer, mag = 0;
  bno.getCalibration(&system, &gyro, &accelerometer, &mag);
  Serial.print("    CALIBRATION: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accelerometer);
  Serial.print(" Mag=");
  Serial.print(mag);

  Serial.println("");
*/
  delay(SAMPLERATE_DELAY_MS);
}
