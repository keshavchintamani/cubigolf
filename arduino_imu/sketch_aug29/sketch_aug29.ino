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
#define ST 0xaa
#define ET 0xbb

Adafruit_BNO055 bno = Adafruit_BNO055();
uint32_t gps_x_m = 51279400;
uint32_t gps_y_m = 567056600;

float heading = 0;
int8_t fix = 4;
float speed = 0;
float wheelSpeed = 0;
double timeSendGps = 0;
float potSteerAngle_deg = 0;

typedef union
{
 float value;
 uint8_t bytes[4];
} FLOATUNION_t;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
//  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
/*  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");*/

  bno.setExtCrystalUse(true);
  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

void SendValue(uint16_t id, float value)
{
  byte data[10]; 
  byte data2[20];
  byte msg[30];
  int m = 0;
  FLOATUNION_t dataFloat;
  dataFloat.value = value;
  //data.length = sizeof(payload);
  
  data[m++] = (byte) id;
  data[m++] = (byte) id >> 8;
  //int32_t val = (int32_t) (value * pow(10,scale));
  data[m++] = dataFloat.bytes[0];
  data[m++] = dataFloat.bytes[1];
  data[m++] = dataFloat.bytes[2];
  data[m++] = dataFloat.bytes[3];
  //data[m++] = scale;
  
  //compose the final msg
  int n = 0;
  int chSum = 0;
  for (byte i=0;i<m;i++)
  {
    data2[n++] = data[i];
    chSum ^= data2[n-1];
  }
  data2[n++] = (byte)chSum;
  data2[n++] = (byte)chSum >> 8;

 //Check for ST & ET within the data and checksum and mark them as data using ST
 int j = 0;
 msg[j++] = ST; //add start transmission
 for (byte i=0;i<n;i++)
  {
    msg[j++] = data2[i];
    if (data2[i] == ST or data2[i] == ET)
    {//mark as transparent
      msg[j++] = ST;
    }
  }
  msg[j++] = ST;
  msg[j++] = ET;
  
  //print
  Serial.write(msg,j);
}

void SendValueCoordinates(uint16_t id, uint32_t value)
{
  byte data[10]; 
  byte data2[20];
  byte msg[30];
  int m = 0;

  data[m++] = (byte) id;
  data[m++] = (byte) id >> 8;
  //int32_t val = (int32_t) (value * pow(10,scale));
  data[m++] =  (byte) value;
  data[m++] =  (byte) (value >> 8);
  data[m++] =  (byte) (value >> 16);
  data[m++] =  (byte) (value >> 24);
  //data[m++] = scale;
  
  //compose the final msg with checkSum
  int n = 0;
  int chSum = 0;
  for (byte i=0;i<m;i++)
  {
    data2[n++] = data[i];
    chSum ^= data2[n-1];
  }
  data2[n++] = (byte)chSum;
  data2[n++] = (byte)chSum >> 8;

 //Check for ST & ET within the data and checksum and mark them as data using ST
 int j = 0;
 msg[j++] = ST; //add start transmission
 for (byte i=0;i<n;i++)
  {
    msg[j++] = data2[i];
    if (data2[i] == ST or data2[i] == ET)
    {//mark as transparent
      msg[j++] = ST;
    }
  }
  msg[j++] = ST;
  msg[j++] = ET;
  
  //print
  Serial.write(msg,j);
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
  SendValue(90, data.x()); //Roll ?
  SendValue(91, data.y()); //Pitch ?
  SendValue(92, data.z()); //Yaw ?
  
  data = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);  
  SendValue(95, data.x());
  SendValue(96, data.y());
  SendValue(97, data.z());
  
  data = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);  
  SendValue(98, data.x());
  SendValue(99, data.y());
  SendValue(100, data.z());
 
  data = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);  
  SendValue(101, data.x());
  SendValue(102, data.y());
  SendValue(103, data.z());
  
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  SendValue(104, quat.x());
  SendValue(105, quat.y());
  SendValue(106, quat.z());
  SendValue(107, quat.w());
  

  /* Display calibration status for each sensor. */
 // uint8_t system, gyro, accel, mag = 0;
/*  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);*/
  
  
  if (millis() - timeSendGps > 10) //10Hz
  {
    timeSendGps = millis();
    //GPS and odometry
    gps_x_m += random(10);
    gps_y_m += random(10);
    heading += (float)random(10)/100;
    speed += (float)random(10)/100;
    wheelSpeed += (float)random(10)/100;
    potSteerAngle_deg = (float)random(10)/10;
  

    if (heading>300) heading -= 250;
    if (speed>15) speed -= 15;
    if (wheelSpeed>15) wheelSpeed -= 15;
    
    /* Display the floating point data */
    SendValueCoordinates(140, gps_x_m);
    SendValueCoordinates(141, gps_y_m);
    SendValue(74, heading);
    SendValue(78, speed);
    SendValue(112, potSteerAngle_deg);
  
  
    /* Display the floating point data */
    SendValue(111, wheelSpeed);
  }


  delay(BNO055_SAMPLERATE_DELAY_MS);
}