import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import time
import math
import numpy as np

class IMU():
	def __init__(self):
		SETTINGS_FILE = "RTIMULib"

		#  computeHeight() - the conversion uses the formula:
		#
		#  h = (T0 / L0) * ((p / P0)**(-(R* * L0) / (g0 * M)) - 1)
		#
		#  where:
		#  h  = height above sea level
		#  T0 = standard temperature at sea level = 288.15
		#  L0 = standard temperatur elapse rate = -0.0065
		#  p  = measured pressure
		#  P0 = static pressure = 1013.25
		#  g0 = gravitational acceleration = 9.80665
		#  M  = mloecular mass of earth's air = 0.0289644
		#  R* = universal gas constant = 8.31432
		#
		#  Given the constants, this works out to:
		#
		#  h = 44330.8 * (1 - (p / P0)**0.190263)
			
		print("Using settings file " + SETTINGS_FILE + ".ini")
		if not os.path.exists(SETTINGS_FILE + ".ini"):
		  print("Settings file does not exist, will be created")

		self.s = RTIMU.Settings(SETTINGS_FILE)
		self.imu = RTIMU.RTIMU(self.s)

		print("IMU Name: " + self.imu.IMUName())
	
		if (not self.imu.IMUInit()):
			print("IMU Init Failed")
			sys.exit(1)
		else:
			print("IMU Init Succeeded");

		# this is a good time to set any fusion parameters

		self.imu.setSlerpPower(0.02)
		self.imu.setGyroEnable(True)
		self.imu.setAccelEnable(True)
		self.imu.setCompassEnable(True)

		self.poll_interval = self.imu.IMUGetPollInterval()
		print("Recommended Poll Interval: %dmS\n" % self.poll_interval)

		'''
		while True:
		  if imu.IMURead():
			# x, y, z = imu.getFusionData()
			# print("%f %f %f" % (x,y,z))
			data = imu.getIMUData()
			(data["pressureValid"], data["pressure"], data["temperatureValid"], data["temperature"]) = pressure.pressureRead()
			fusionPose = data["fusionPose"]
			print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), 
				math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
			if (data["pressureValid"]):
				print("Pressure: %f, height above sea level: %f" % (data["pressure"], computeHeight(data["pressure"])))
			if (data["temperatureValid"]):
				print("Temperature: %f" % (data["temperature"]))
			time.sleep(poll_interval*1.0/1000.0)
		'''
	def getFirstYaw(self,iteraciones):
		i = 0
		fusionPose = np.array([0,0,0])
		while(i < 2*iteraciones):
			if self.imu.IMURead():
				data = self.imu.getIMUData()
				auxFusionPose = data["fusionPose"]
				print(i)
				#print(auxFusionPose)
				print("r: %f p: %f y: %f" % (math.degrees(auxFusionPose[0]),math.degrees(auxFusionPose[1]), math.degrees(auxFusionPose[2])))
				if (i >= iteraciones):
					auxFusionPose = np.array(auxFusionPose)
					fusionPose = fusionPose + auxFusionPose
					#print(fusionPose)
				i += 1
				time.sleep(self.poll_interval*1.0/1000.0)
		fusionPose /= iteraciones
		print(math.degrees(fusionPose[2]))
		#fusionPose[2] = math.degrees(fusionPose[2])
		return fusionPose[2]
			
	def getYaw(self):
		if self.imu.IMURead():
			data = self.imu.getIMUData()
			fusionPose = data["fusionPose"]
			#print(auxFusionPose)
			print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]),math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
		return fusionPose[2]
