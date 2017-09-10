#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <utility/vector.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define GPSECHO false
#define ADAFRUIT_GPS true
#define ECHO true //Setting true writes to serial monitor instead
#define GPS_DT 10 //Sets the time period for GPS sampling

#ifdef ADAFRUIT_GPS
    SoftwareSerial mySerial(3, 2);
    Adafruit_GPS GPS(&mySerial);
    uint32_t timer = millis();
    boolean usingInterrupt = false;
    void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
    #define GPS_DT 1000 //Sets the time period for GPS sampling
#endif
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
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  //If we're using an ADAFRUIT_GPS, setup a 1Hz timer interrup;
  #ifdef ADAFRUIT_GPS
    //adafruit_timer_setup();
    setup_adafruit_gps();
  #endif
  delay(1000);
   Serial.print("Starting sketch ");
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
  #if ECHO == false
    Serial.write(msg,j);
  #endif
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
  #if ECHO == false
    Serial.write(msg,j);
  #endif
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
  //Serial.println("Hello world");
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

  if (millis() - timeSendGps > GPS_DT) //10Hz
  {

    timeSendGps = millis();
    //GPS and odometry
    #if ADAFRUIT_GPS == true
        float * gps_vals;
        read_adafruit_gps();

    #else
     //TODO Jacob's RTK should get published here
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
    #endif

}


  delay(BNO055_SAMPLERATE_DELAY_MS);
}

#ifdef ADAFRUIT_GPS

void setup_adafruit_gps()
{
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);
    useInterrupt(true);
}

void read_adafruit_gps()
{
    // in case you are not using the interrupt above, you'll
    // need to 'hand query' the GPS, not suggested :(
    if (! usingInterrupt) {
      // read data from the GPS in the 'main loop'
      char c = GPS.read();
      // if you want to debug, this is a good time to do it!
      if (GPSECHO)
        if (c) Serial.print(c);
    }

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences!
      // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
      //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    /* if (GPS.lat =="N")
      { ret[0]= gps_lat_deg;}
      else
      { ret[0]=-gps_lat_deg;}
      if (GPS.lon =="E")
      { ret[1]= gps_lon_deg;}
      else
      { ret[1]=-gps_lon_deg;}
      ret[2]=GPS.speed;
      ret[3]=GPS.angle;
      ret[4]=GPS.altitude;
      ret[5]=GPS.fix;
      ret[6]=GPS.fixquality;
      return(ret);*/

      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    float gps_lat_deg = convertNMEAtoWGS84(GPS.latitude);
    float gps_lon_deg = convertNMEAtoWGS84(GPS.longitude);

    #if ECHO == true //Write a sample IMU and GPS value to console
        Serial.print(gps_lat_deg,4);
        Serial.print(" ");
        Serial.println(gps_lon_deg,4);
        Serial.print(GPS.latitude);
        Serial.print(" ");
        Serial.println(GPS.longitude);
        Serial.print(GPS.lat);
        Serial.print(" ");
        Serial.println(GPS.lon);
    #else
        SendValue(301,gps_lat_deg); //lat
        SendValue(302,gps_lon_deg); //long
        SendValue(78, GPS.speed); //Speed
        SendValue(74, GPS.angle); //Angle
        SendValue(73, GPS.altitude); //Altitude
        SendValue(76, GPS.fix); //fix
        SendValue(77, GPS.fixquality); //fix quali
    #endif



}

float convertNMEAtoWGS84(float value)
{
     //sign = int(value/ abs(value))
     int DD = int(float(abs(value))) / 100;
     float SS = float(abs(value)) - DD * 100;
     float res = (DD + SS / 60);
     return res;
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif
