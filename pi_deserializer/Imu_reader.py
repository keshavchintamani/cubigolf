#!/usr/bin/env python

import serial
import struct

class messageLib:
	gps_time_s = dict(mid=70, value =0)
	gps_lat_deg= dict(mid=71, value =0)	
	gps_long_deg= dict(mid=72, value =0)	
	gps_altitude_m= dict(mid=73, value =0)	
	gps_heading_deg= dict(mid=74, value =0)	
	gps_fix= dict(mid=76, value =0)	
	gps_HDOP= dict(mid=77, value =0)	
	gps_speed_kph= dict(mid=78, value =0)	

	imu_angleX_deg	= dict(mid=90, value=0)
	imu_angleY_deg = dict(mid=91, value=0)
	imu_angleZ_deg 	= dict(mid=92, value=0)
	imu_WzSeerAngle_deg	= dict(mid=95, value=0)
	imu_accX_ms2 	= dict(mid=95, value=0)
	imu_accY_ms2 	= dict(mid=96, value=0)
	imu_accZ_ms2 	= dict(mid=97, value=0)
	imu_magX_ut 	= dict(mid=98, value=0)
	imu_magY_ut 	= dict(mid=99, value=0)
	imu_magZ_ut 	= dict(mid=100, value=0)
	imu_gyroX_rads	= dict(mid=101, value=0)
	imu_gyroY_rads	= dict(mid=102, value=0)
	imu_gyroZ_rads	= dict(mid=103, value=0)
	imu_quatX 	= dict(mid=104, value=0)
	imu_quatY 	= dict(mid=105, value=0)
	imu_quatZ 	= dict(mid=106, value=0)
	imu_quatW 	= dict(mid=107, value=0)
	
	pos_gps_x_m	= dict(mid=140, value=0)
	pos_gps_y_m	= dict(mid=141, value=0)


	odo_pulses	= dict(mid=110, value=0)
	odo_speed_kph	= dict(mid=111, value=0)
	odo_potSteerAngle_deg = dict(mid=112, value=0)

	

class UartPLCDeserializer:
	def __init__(self, usbDevice):	
		self.ser = serial.Serial(usbDevice, 115200, timeout=1)
		self.bytesIn = []
		self.cleanBytes = []
		self.mainSwitch = 0
		self.byteIn = 0
		self.ST = 0xaa;
		self.ET = 0xbb;
		self.STx = '\xaa'
		self.ETx = '\xbb'
		self.previousByte = 0
		self.errors = 0
		self.variables = messageLib()
		self.isUpdated = False

	def ProcessMsg(self):			
		b = ''.join(map(chr, self.cleanBytes[0:2]))
		msgId = struct.unpack('H', b)[0]		
		
		if msgId == 71 or msgId == 72 or msgId == 140 or msgId == 141:
			b = ''.join(map(chr, self.cleanBytes[2:6]))
			msgValue = float(struct.unpack('L', b)[0])/100
			
		else:		
			b = ''.join(map(chr, self.cleanBytes[2:6]))
			msgValue = struct.unpack('<f', b)[0]
		
		for a in dir(self.variables):
			if not a.startswith('__'):
				var = getattr(self.variables, a)
				if var['mid'] == msgId:
					var['value'] = msgValue
					
		self.isUpdated = True

	def ProcessRawBytes(self):
		#clean markers
		self.cleanBytes = []
		skip = False
		checkS = 0
		for i in range(0, len(self.bytesIn)):
			if skip:
				skip = False
				continue
			if self.bytesIn[i] == self.ST or self.bytesIn[i] == self.ET:
				self.cleanBytes.append(self.bytesIn[i])
				self.skip = True
			else:
				self.cleanBytes.append(self.bytesIn[i])
			if len(self.cleanBytes) < 7:
				checkS ^= self.bytesIn[i]
		#checkSum
		checkSint = self.cleanBytes[6] | self.cleanBytes[7] << 8
		if checkS == checkSint:
			self.ProcessMsg()
	
	def ProcessByteIn(self):
		if self.byteIn == self.STx and self.mainSwitch == 0:
			self.mainSwitch = 1
			self.bytesIn = []
			return			
		if self.byteIn == self.ETx and self.mainSwitch == 1 and self.previousByte == self.STx:
			self.mainSwitch = 0
			self.bytesIn = bytearray(self.bytesIn)
			try:
				self.ProcessRawBytes()
			except:
				print 'error'
				self.errors += 1
		
		if len(self.bytesIn) > 20:
			self.errors += 1
			print 'error'
			self.mainSwitch = 0
			self.bytesIn = []
			return		
		else:	
			self.bytesIn.append(self.byteIn)
			
	def Main(self):
		while True:
			while self.ser.inWaiting() > 0:
				self.byteIn = self.ser.read(1)
				self.ProcessByteIn()
				self.previousByte = self.byteIn
			pass
		


 