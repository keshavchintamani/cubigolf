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

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (20)

Adafruit_BNO055 bno = Adafruit_BNO055();
  float latitude = 50.8503463;
  float longitude = 4.351721;
  float heading = 0;
  int8_t fix = 4;
  float speed = 0;
  float wheelSpeed = 0;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(38400);
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
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
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
  imu::Vector<3> data = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("IMU;");
  Serial.print(data.x(), 4);
  Serial.print(";");
  Serial.print(data.y(), 4);
  Serial.print(";");
  Serial.print(data.z(), 4);
  
  
  data = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);  
  
  Serial.print(";");
  Serial.print(data.x(), 4);
  Serial.print(";");
  Serial.print(data.y(), 4);
  Serial.print(";");
  Serial.print(data.z(), 4);
  
  
  data = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);  
  
  Serial.print(";");
  Serial.print(data.x(), 4);
  Serial.print(";");
  Serial.print(data.y(), 4);
  Serial.print(";");
  Serial.print(data.z(), 4);
 
  data = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);  
  
  Serial.print(";");
  Serial.print(data.x(), 4);
  Serial.print(";");
  Serial.print(data.y(), 4);
  Serial.print(";");
  Serial.println(data.z(), 4);
  

  // Quaternion data
/*  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.println(quat.z(), 4);*/
  

  /* Display calibration status for each sensor. */
/*  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);*/
  
  //GPS and odometry
  latitude += (float)random(100)/100000;
  longitude += (float)random(100)/100000;
  heading += (float)random(10)/100;
  speed += (float)random(10)/100;
  wheelSpeed += (float)random(10)/100;
  
  if (latitude>90) latitude -= 45;
  if (longitude>300) longitude -= 250;
  if (heading>300) heading -= 250;
  if (speed>15) speed -= 15;
  if (wheelSpeed>15) wheelSpeed -= 15;
  
    /* Display the floating point data */
  Serial.print("GPS;");
  Serial.print(latitude, 8);
  Serial.print(";");
  Serial.print(longitude, 8);
  Serial.print(";");
  Serial.print(heading, 4);
  Serial.print(";");
  Serial.print(fix);
  Serial.print(";");
  Serial.println(speed, 4);
  
      /* Display the floating point data */
  Serial.print("ODO;");
  Serial.println(wheelSpeed, 4);


  delay(BNO055_SAMPLERATE_DELAY_MS);
}