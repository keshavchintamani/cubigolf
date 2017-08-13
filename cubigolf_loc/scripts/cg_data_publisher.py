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
        # line = self.Serial.readline()

        # res = re.findall("[-+]?\d+[\.]?\d*", line)
        try:
            line = self._readline()
            return (line)

        except ValueError:
            print("Value error exception parsing serial data")
            pass

    def _readline(self):
        eol = b'\r'
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
        return bytes(line)

    def writeserial(self, message):
        try:
            self.Serial.write(message);
            return (True);
        except (IOError, ValueError):
            print("Cannot open serial device: %s", self.devid)
            raise IOError
        return (False);


def Start():

    rospy.init_node('cubigolf_localization', anonymous=True)

    #Setup the publishers
    pose_pub = rospy.Publisher("/cubigolf/imu", Imu, queue_size= 10)
    navsat_pub = rospy.Publisher("/cubigolf/gps", NavSatFix, queue_size=10)
    magneto_pub = rospy.Publisher("/cubigolf/magneticfield", MagneticField, queue_size=10)

    r = rospy.Rate(50) # 10hz
    gyro = Imu()
    rtk = NavSatFix()
    magneto = MagneticField()

    #Setup Serial interface
    Serial = SerialReader('/dev/ttyUSB0')
    while not rospy.is_shutdown():

        #TODO Read Serial Interface
        gotthis = Serial.readserial();
        print "got this: {}".format(gotthis)

        gyro.header.stamp = rospy.Time.now()
        #Populate the pose message
        gyro.orientation.x = 0
        gyro.orientation.y = 0
        gyro.orientation.z = 0
        gyro.orientation.w = 0

        #convert euler into quaternion
        #quat = tf.transformations.quaternion_from_euler(sense.get_orientation_degrees()['pitch']*D2R, \
        #                                        sense.get_orientation_degrees()['roll'] * D2R, \
        #                                        sense.get_orientation_degrees()['yaw'] * D2R)
        gyro.angular_velocity.x = 0
        gyro.angular_velocity.y = 0
        gyro.angular_velocity.z = 0

        gyro.linear_acceleration.x = 0
        gyro.linear_acceleration.y = 0
        gyro.linear_acceleration.z = 0

        #Populate the magneto message
        magneto.header.stamp = rospy.Time.now()
        magneto.magnetic_field.x = 0
        magneto.magnetic_field.y = 0
        magneto.magnetic_field.z = 0

        #Populate navfix message
        rtk.header.stamp = rospy.Time.now()
        rtk.latitude = 0
        rtk.longitude = 0
        rtk.altitude = 0

        #Publish the messages
        pose_pub.publish(gyro)
        magneto_pub.publish(magneto)
        navsat_pub.publish(rtk)

        r.sleep()

if __name__ == '__main__':
    Start()

