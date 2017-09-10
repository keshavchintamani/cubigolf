#!/usr/bin/env python

"""
cg_data_publisher reads a serial interface, captures and publishes IMU, GPS, wheel speed and steering angle topics
It provides a naive example of a odometry topic for a differential wheel robot.
This needs to be transformed into an ackermann platform's at some point
"""

import time
import math
import serial
import threading
import rospy
import tf.transformations
import random as Rand
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from cg_serial_reader import UartPLCDeserializer, messageLib

__author__ = ["Keshav Chintamani", "Jacob Carballido"]
__copyright__ = "Copyright 2017, The Cubigolf Project"
__credits__ = ["Keshav Chintamani", "Jacob Carballido"]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Keshav Chintamani"
__email__ = "keshav.chintamani@gmail.com"
__status__ = "Devel"


def NMEA2WGS84(value):
    try:
        sign = int(value/ abs(value))
        DD = int(float(abs(value))) / 100
        SS = float(abs(value)) - DD * 100
        return float((DD + SS / 60)*sign)
    except ZeroDivisionError:
        print "Could not convert NMEA to DMS... value is {}".format(value)


def Start():

    rospy.init_node('cubigolf_localization', anonymous=True)

    #Setup the publishers
    imu_pub = rospy.Publisher("/cubigolf/imu", Imu, queue_size= 10)
    navsat_pub = rospy.Publisher("/cubigolf/gps", NavSatFix, queue_size=10)
    magneto_pub = rospy.Publisher("/cubigolf/magneticfield", MagneticField, queue_size=10)
    odom_pub = rospy.Publisher("/cubigolf/odom", Odometry, queue_size=10)

    freq = 100
    r = rospy.Rate(freq) # 10hz
    imu = Imu()
    gps = NavSatFix()
    odom = Odometry()
    magneto = MagneticField()
    wheelspeed = Float32()
    steeringangle = Float32()
    vel_oldx = vel_oldy = angle_old = 0

    #Start deserializing the code coming in on the serial port
    myDeco = UartPLCDeserializer('/dev/ttyUSB0')
    deserializerThread = threading.Thread(target=myDeco.Main, args=())
    deserializerThread.start()

    while not rospy.is_shutdown():

        #Populate IMU
        imu.header.stamp = rospy.Time.now()
        imu.orientation.x = myDeco.variables.imu_quatX['value']
        imu.orientation.y = myDeco.variables.imu_quatY['value']
        imu.orientation.z = myDeco.variables.imu_quatZ['value']
        imu.orientation.w = myDeco.variables.imu_quatW['value']

        imu.angular_velocity.x = myDeco.variables.imu_gyroX_rads['value']
        imu.angular_velocity.y = myDeco.variables.imu_gyroY_rads['value']
        imu.angular_velocity.z = myDeco.variables.imu_gyroZ_rads['value']

        imu.linear_acceleration.x = myDeco.variables.imu_accX_ms2['value']
        imu.linear_acceleration.y = myDeco.variables.imu_accY_ms2['value']
        imu.linear_acceleration.z = myDeco.variables.imu_accZ_ms2['value']
        imu_pub.publish(imu)

        #Populate the magneto message
        magneto.header.stamp = rospy.Time.now()
        magneto.magnetic_field.x = myDeco.variables.imu_magX_ut['value']
        magneto.magnetic_field.y = myDeco.variables.imu_magY_ut['value']
        magneto.magnetic_field.z = myDeco.variables.imu_magZ_ut['value']
        magneto_pub.publish(magneto)
        #Populate navfix message

        gps.header.stamp = rospy.Time.now()
        gps.latitude = myDeco.variables.gps_lat_dms['value']
        gps.longitude = myDeco.variables.gps_long_dms['value']
        gps.altitude = myDeco.variables.gps_altitude_m['value']
        print  gps.latitude,  gps.longitude, gps.altitude
        #NMEA  0 = Invalid;1 = GPS fix, 2 = DGPS fix
        #ROS NavSatStatus
        #int8 STATUS_NO_FIX =  -1        # unable to fix position
        #int8 STATUS_FIX =      0        # unaugmented fix
        #int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
        #int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation
        if myDeco.variables.gps_fix['value'] == 0:
            gps.status.status = NavSatStatus.STATUS_NO_FIX
        elif myDeco.variables.gps_fix['value'] == 1:
            gps.status.status = NavSatStatus.STATUS_FIX
        elif myDeco.variables.gps_fix['value'] == 2:
            gps.status.status = NavSatStatus.STATUS_SBAS_FIX # TODO Check FIX assumptions
        navsat_pub.publish(gps)

        #TODO Define an odometry frame for an ackermann platfrom
        odom.child_frame_id="base_link"
        odom.twist.twist.linear.x = 0
        odom.twist.twist.linear.y = 0
        odom.child_frame_id = "odom"
        odom.pose.pose.position.x = 0
        odom.pose.pose.position.y = 0
        odom.pose.pose.orientation.x = 0
        odom.pose.pose.orientation.y = 0
        odom.pose.pose.orientation.z = 0
        odom.pose.pose.orientation.w = 1
        odom_pub.publish(odom)
        r.sleep()

if __name__ == '__main__':
    global deserializerThread
    Start()


