#!/usr/bin/env python
import thread
import time
import Imu_reader
import threading

myDeco = Imu_reader.UartPLCDeserializer('/dev/ttyUSB0')

deserializerThread = threading.Thread(target=myDeco.Main, args=())
deserializerThread.start()

while True:
	if myDeco.isUpdated:
		print myDeco.variables.pos_gps_x_m['value'], myDeco.variables.pos_gps_y_m['value'], myDeco.variables.odo_potSteerAngle_deg['value']
		myDeco.isUpdated = False
		time.sleep(0.1)

