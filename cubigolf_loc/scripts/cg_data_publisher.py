#!/usr/bin/env python

import time
import rospy
import tf.transformations
import serial
from sensor_msgs.msg import Imu, MagneticField, NavSatFix

D2R = 0.0174533

class SerialReader():
    def __init__(self, devid):

        self.devid = devid
        self._startserial()

    def _startserial(self):

        print("Trying to open Serial arduino: %s ", self.devid)
        #try:
        self.Serial = serial.Serial(self.devid, 38400, timeout=1)
        print("Success: %s ", self.Serial)
        time.sleep(1)
        pass
        #except (IOError, ValueError):
        #    print("Cannot open serial device: %s", self.devid)
        #    raise IOError

    def readserial(self):

        try:
            line = self._readline()
            return (line)

        except ValueError:
            print("Value error exception parsing serial data")
            pass

    def _readline(self):
        eol = '\r\n'
        leneol = len(eol)
        line = bytearray()
        while True:
            c = self.Serial.read(1)
            if c:
                line += c
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line[:-leneol])

    def writeserial(self, message):
        try:
            self.Serial.write(message);
            return (True);
        except (IOError, ValueError):
            print("Cannot open serial device: %s", self.devid)
            raise IOError
        return (False);

def splitString(str):

    split = str.split(';')
    print split
    vals = []

    for i in range(1, len(split)):
            try:
                vals.append(float(split[i]))
            except ValueError:
                pass
    header = split[0]
    return (header, vals)

def Start():

    rospy.init_node('cubigolf_localization', anonymous=True)

    #Setup the publishers
    pose_pub = rospy.Publisher("/cubigolf/imu", Imu, queue_size= 10)
    navsat_pub = rospy.Publisher("/cubigolf/gps", NavSatFix, queue_size=10)
    magneto_pub = rospy.Publisher("/cubigolf/magneticfield", MagneticField, queue_size=10)

    r = rospy.Rate(10) # 10hz
    gyro = Imu()
    rtk = NavSatFix()
    magneto = MagneticField()

    #Setup Serial interface
    Serial = SerialReader('/dev/ttyUSB0')
    while not rospy.is_shutdown():

        received = Serial.readserial();
        (header, vals) = splitString(received)

        if header == "IMU":
            gyro.header.stamp = rospy.Time.now()
            #Populate the pose message

            roll = vals[0]
            pitch = vals[1]
            yaw = vals[2]
            #convert euler into quaternion
            quat = tf.transformations.quaternion_from_euler(pitch*D2R,roll * D2R, yaw * D2R)
            gyro.orientation.x = quat[0]
            gyro.orientation.y = quat[1]
            gyro.orientation.z = quat[2]
            gyro.orientation.w = quat[3]

            gyro.angular_velocity.x = vals[9]
            gyro.angular_velocity.y = vals[10]
            gyro.angular_velocity.z = vals[11]

            gyro.linear_acceleration.x = vals[3]
            gyro.linear_acceleration.y = vals[4]
            gyro.linear_acceleration.z = vals[5]
            pose_pub.publish(gyro)
        #elif header == "COMPASS":
        #    #Populate the magneto message
        #    magneto.header.stamp = rospy.Time.now()
        #    magneto.magnetic_field.x = 0
        #    magneto.magnetic_field.y = 0
        #    magneto.magnetic_field.z = 0
        #    magneto_pub.publish(magneto)
        elif header == "GPS":
            #Populate navfix message
            rtk.header.stamp = rospy.Time.now()
            rtk.latitude = vals[0]
            rtk.longitude = vals[1]
            if vals[2] > 4:
                rtk.status.status = 0
            else:
                rtk.status.status = 1
            navsat_pub.publish(rtk)
        elif header =="ODO":
            vel = vals[0]#Do somethins
        else:
            rospy.logerr("Invalid serial header detected: %s %s", header, vals)
        r.sleep()

if __name__ == '__main__':
    Start()

