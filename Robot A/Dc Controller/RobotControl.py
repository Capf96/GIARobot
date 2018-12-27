from MotorControl import Motor
import pigpio
from time import sleep
from math import log

#encoder provides a resolution of 48 counts per revolution




class Robot(object):
	
	def __init__(self, mLeft, mRight):
		
		self.mLeft = mLeft
		self.mRight = mRight
	
	def foward(self, ticks, speed):
		
		kp = 0.6 #Buen valor 0.75
		ki = 0.0
		kd = 0
		dt = 0.01
		pdif = 0
		psum = 0
		while self.mLeft.getTicks() < ticks and  self.mRight.getTicks() < ticks:
	
			ticksLeft = self.mLeft.getTicks()
			ticksRight = self.mRight.getTicks()

			dif= ticksRight - ticksLeft
			
			psum = psum + dif

			delta = dif * kp + (dif - pdif / dt) *kd + ki * psum

			self.mLeft.run(speed + delta) # 

			self.mRight.run(speed) 

			print( str(self.mRight.getTicks()) + " " + str(self.mLeft.getTicks()) +" " +str(dif))

			pdif= dif

			sleep(dt)
		
		self.mLeft.stop()
		self.mRight.stop()
	def fowardB(self,distance, pwm): #distance tiene que estar en centimetros
		kp = 0.35 #Buen valor 0.75
		ki = 0
		kd = 0
		pdif = 0
		psum = 0
		
		distDelta  = lambda x: 0.14692097 * (x**(0.93840107)) 
		
		distance = distance #+ distDelta(distance) + distDelta(distDelta(distance)) 
		
		r = 4.08 #cm
		dt = 0.07 #s
		
		
		pTicksR = 0
		pTicksL = 0
		
		nTicksR = 0
		nTicksL = 0
		
		distanceR = 0
		pi = 3.141592654 #RAD
		
		while distanceR < distance :
			nTicksR = self.mRight.getTicks()
			nTicksL = self.mLeft.getTicks()
			
			dif= nTicksR - nTicksL
			
			psum = psum + dif

			delta = dif * kp + (dif - pdif / dt) *kd + ki * psum

			self.mLeft.run(pwm + delta)  

			self.mRight.run(pwm) 


			pdif = dif

			wr =  pi * (nTicksR - pTicksR) / (dt * 10 * 180) # convertimos los ticks/s en rad/segfundos 3600 ticks = 2pi
			
			wl = pi * (nTicksL - pTicksL) / (dt * 10 * 180)
			
			v = r * (wr + wl) / 2 # cm/s
			
			distanceR = v * dt + distanceR
			
			pTicksR = nTicksR
			pTicksL = nTicksL
		
			
			
			#print("""str(v) + " " + str(wl) + " " + str(wr) + " " +""" str(distanceR))
			
			print(str(int(distanceR)))
			sleep(dt)
		
		self.mLeft.stop()
		self.mRight.stop()	

	def turnRight(self, degrees, pwm):
		kp = 0.1 #Buen valor 0.75
		ki = 0.00
		kd = 0
		pdif = 0
		psum = 0
		
		correction = lambda x: 1.86586641*(x**0.32895066) - 3
		
		degrees = degrees - correction(degrees)
		
		r = 4.08 #cm
		L = 29.9 - 3.4 #cm Distancia entre las ruedas
		dt = 0.07 #s
		
		
		pTicksR = 0
		pTicksL = 0
		
		nTicksR = 0
		nTicksL = 0
		
		degreesT = 0
		
		while degreesT < degrees :
			
			nTicksR = self.mRight.getTicks()
			nTicksL = self.mLeft.getTicks()
			
			dif = abs(nTicksR) - abs(nTicksL)
			
			psum = psum + dif

			delta = dif * kp + (dif - pdif / dt) *kd + ki * psum

			self.mLeft.run(-pwm - dif)  

			self.mRight.run(pwm) 


			pdif = dif

			wr =(nTicksR - pTicksR) / (dt * 10) # convertimos los ticks/s en grados/segfundos 3600 ticks = 360 grados
			
			wl =(nTicksL - pTicksL) / (dt * 10)
			
			w = r * (wr - wl) / L # grados/s
			
			degreesT = w * dt + degreesT
			
			pTicksR = nTicksR
			pTicksL = nTicksL
			
			print(str(degreesT) + " ")
			
			sleep(dt)
		
		self.mLeft.stop()
		self.mRight.stop()
	def turn(self, degrees, vel):	
		wheelR = 4.08 #"cm"
		robotL = 30.2 - 3.4 #"cm"
		#"pI = 3.14"
		
		step = 50
		dt = 0.001
	
		ticks = round( 3600 * (robotL * degrees) / (wheelR * 360) )#"48 * (robotL * degrees * (pI/180)) / (wheelR * 2 * pI)"
		
		while self.mRight.getTicks() > -5700 and self.mLeft.getTicks() < 5700:
			
			
			self.mLeft.turnTicks(step, -vel)
			self.mRight.turnTicks(-step, vel)
			 
			
			print(str(self.mRight.getTicks()) + " " + str(self.mLeft.getTicks()))
			sleep(dt)
			
		while self.mRight.getTicks() > -5700:
			
			self.mRight.turnTicks(-step, vel)
			print(str(self.mRight.getTicks()) + " " + str(self.mLeft.getTicks()))
			sleep(dt)
		
		
		while self.mLeft.getTicks() < 5700:
			
			self.mLeft.turnTicks(step, -vel)
			print(str(self.mRight.getTicks()) + " " + str(self.mLeft.getTicks()))
			sleep(dt)
			
			
			
		
		self.mLeft.stop()
		self.mRight.stop()

	def turnPID(self, degrees, vel):
		wheelR = 4.08#"cm"
		robotL = 30.2 - 3.4 #"cm"
		#"pI = 3.14"
		
		kp = 0.025
		ki = 0
		kd= 0
		pdif = 0
		dt = 0.05
		
		ticks = round( 3600 * ( (robotL / 2) * degrees) / (wheelR * 360) )#"48 * (robotL * degrees * (pI/180)) / (wheelR * 2 * pI)"
		
		ticks = 5700
		while self.mRight.getTicks() > -ticks and self.mLeft.getTicks() < ticks:
			
			ticksLeft = self.mLeft.getTicks()
			ticksRight = self.mRight.getTicks()
			
			dif = ticksRight - ticksLeft
	
			delta = dif * kp + kd * (dif - pdif) / dt
			
			self.mLeft.run(vel)
			self.mRight.run(-vel ) # +delta)
			
			#pdif = dif
			
			sleep(dt)
		self.mLeft.stop()
		self.mRight.stop()
		while self.mRight.getTicks() > -ticks:
			
			self.mRight.run(-vel)
			print(str(self.mRight.getTicks()) + " " + str(self.mLeft.getTicks()))
			sleep(dt)
		
		self.mLeft.stop()
		self.mRight.stop()
		while self.mLeft.getTicks() < ticks:
			
			self.mLeft.run(vel)
			print(str(self.mRight.getTicks()) + " " + str(self.mLeft.getTicks()))
			sleep(dt)
		
		self.mLeft.stop()
		self.mRight.stop()
		
	def rightPrll (self, ticks, vel):
		counter = 0
		step = 1000
		dt = 0.07
		while counter < ticks:
			self.mLeft.turnTicks(step, vel)
			sleep(dt)
			self.mRight.turnTicks(step, vel)
			sleep(dt)
			
			counter = counter + abs(self.mRight.getTicks())
			
			self.mLeft.turnTicks(-step, -vel)
			sleep(dt)
			self.mRight.turnTicks(-step, -vel)
			sleep(dt)
			
			counter = counter + abs(self.mRight.getTicks())
			print(str(counter))
		self.mLeft.stop()
		self.mRight.stop()
		
	def leftPrll (self, ticks, vel):
		counter = 0
		step = 1000
		dt = 0.1
		while counter < ticks:
			self.mRight.turnTicks(step, vel)
			sleep(dt)
			self.mLeft.turnTicks(step, vel)
			sleep(dt)
			
			counter = counter + abs(self.mLeft.getTicks())
			
			self.mRight.turnTicks(-step, -vel)
			sleep(dt)
			self.mLeft.turnTicks(-step, -vel)
			sleep(dt)
			
			counter = counter + abs(self.mLeft.getTicks())
			print(str(counter))
		self.mLeft.stop()
		self.mRight.stop()
	
		
