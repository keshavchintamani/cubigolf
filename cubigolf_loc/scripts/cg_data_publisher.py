#!/usr/bin/env python

"""
cg_data_publisher reads a serial interface, captures and publishes IMU, GPS, wheel speed and steering angle topics
It provides a naive example of a odometry topic for a differential wheel robot.
This needs to be transformed into an ackermann platform's at some point
"""

import time
import math
import serial
import rospy
import tf.transformations
import random as Rand
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

__author__ = "Keshav Chintamani"
__copyright__ = "Copyright 2017, The Cubigolf Project"
__credits__ = ["Keshav Chintamani", "Jacob Carballido"]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Keshav Chintamani"
__email__ = "keshav.chintamani@gmail.com"
__status__ = "Devel"


D2R = 0.0174533
Vehicle_diameter = 1

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
            self.Serial.write(message)
            return (True);
        except (IOError, ValueError):
            print("Cannot open serial device: %s", self.devid)
            raise IOError
        return (False);

def splitString(str):

    split = str.split(';')
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

    odom_pub = rospy.Publisher("/cubigolf/odom", Odometry, queue_size=10)

    freq = 30
    r = rospy.Rate(freq) # 10hz
    dt =0
    gyro = Imu()
    rtk = NavSatFix()
    odom = Odometry()
    magneto = MagneticField()
    wheelspeed = Float32()
    steeringangle = Float32()
    vel_oldx = vel_oldy = angle_old = 0
    #Setup Serial interface
    Serial = SerialReader('/dev/ttyUSB0')
    while not rospy.is_shutdown():

        received = Serial.readserial()
        (header, vals) = splitString(received)

        if header == "IMU":

            gyro.header.stamp = rospy.Time.now()
            #Populate the pose message
            roll = vals[0]
            pitch = vals[1]
            yaw = vals[2]
            #convert euler into quaternion
            quat = tf.transformations.quaternion_from_euler(roll*D2R,pitch * D2R, yaw * D2R)
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
            Rand.seed()
            vel = Rand.uniform(0,10)
            angle = -30*D2R
            time_now = time.time()
            odom.twist.twist.linear.x = math.cos(angle) * vel
            odom.twist.twist.linear.y = math.sin(angle) * vel
            odom.pose.pose.position.x +=  (odom.twist.twist.linear.x) * dt
            odom.pose.pose.position.y +=  (odom.twist.twist.linear.y) * dt

            quat = tf.transformations.quaternion_from_euler(0, 0, angle)
            odom.pose.pose.orientation.x = quat[0]
            odom.pose.pose.orientation.y = quat[1]
            odom.pose.pose.orientation.z = quat[2]
            odom.pose.pose.orientation.w = quat[3]

            if not dt == 0:
                odom.twist.twist.angular.z = (angle) / dt

            angle_old = angle
            time_then = time.time()
            dt = time_now - time_then
            odom_pub.publish(odom)
        else:
            rospy.logerr("Invalid serial header detected: %s %s", header, vals)
        r.sleep()

if __name__ == '__main__':
    Start()

