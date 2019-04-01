from MotorControl import Motor
import pigpio
from time import sleep, time
from electroIman import Magnet

from Sensores import QTR, Sensor
import serial

ser = serial.Serial('/dev/ttyUSB0', 57600)

#encoder provides a resolution of 3600 counts per revolution
ser.flushInput()



class Robot(object):
	
	def __init__(self, mLeft, mRight, magnet, qtr):
		self.mLeft = mLeft
		self.mRight = mRight
		self.magnet = magnet
		self.qtr = qtr
		º	
		
	
	
	def fowardC(self,distance, pwm): #distance tiene que estar en centimetros
		
		kp = 0.0875 #Buen valor 0.75
		ki = 0
		kd = 0
		
		pdif = 0 #previous diference
		psum = 0 #previous sum
		
		distDelta  = lambda x: 0.14692097 * (x**(0.93840107)) 
		
		#distance = distance #+ distDelta(distance) + distDelta(distDelta(distance)) 
		
		r = 4.08 #cm
		dt = 0.07 #s before 0.07
		
		
		pTicksR = 0
		pTicksL = 0
		
		nTicksR = 0
		nTicksL = 0
		
		distanceR = 0
		pi = 3.141592654 #RAD
		
		while distanceR < distance  :
			
			nTicksR = self.mRight.getTicks()
			nTicksL = self.mLeft.getTicks()
			
			dif = nTicksR - nTicksL
			
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
		
			
			sleep(dt)
		
	
	def backwardC(self,distance, pwm): #distance tiene que estar en centimetros
		
		kp = 0.0875 #Buen valor 0.75
		ki = 0
		kd = 0
		
		pdif = 0 #previous diference
		psum = 0 #previous sum
		
		distDelta  = lambda x: 0.14692097 * (x**(0.93840107)) 
		
		#distance = distance #+ distDelta(distance) + distDelta(distDelta(distance)) 
		
		distance = -distance
		
		r = 4.08 #cm
		dt = 0.000104 #s
		
		
		pTicksR = 0
		pTicksL = 0
		
		nTicksR = 0
		nTicksL = 0
		
		distanceR = 0
		pi = 3.141592654 #RAD
		
		while distanceR > distance :
			
			nTicksR = self.mRight.getTicks()
			nTicksL = self.mLeft.getTicks()
			
			dif = nTicksR - nTicksL
			
			psum = psum + dif

			delta = dif * kp + (dif - pdif / dt) *kd + ki * psum

			self.mLeft.run(-pwm + delta)  

			self.mRight.run(-pwm) 


			pdif = dif

			wr =  pi * (nTicksR - pTicksR) / (dt * 10 * 180) # convertimos los ticks/s en rad/segfundos 3600 ticks = 2pi
			
			wl = pi * (nTicksL - pTicksL) / (dt * 10 * 180)
			
			v = r * (wr + wl) / 2 # cm/s
			
			distanceR = v * dt + distanceR
			
			pTicksR = nTicksR
			pTicksL = nTicksL
		
			
			
			sleep(dt)
	
	
	def fowardB(self,distance, pwm): #distance tiene que estar en centimetros
		kp = 0.75 #Buen valor 0.75
		ki = 0
		kd = 0
		pdif = 0
		psum = 0
		
		distDelta  = lambda x: 0.14692097 * (x**(0.93840107)) 
		
		#distance = distance #+ distDelta(distance) + distDelta(distDelta(distance)) 
		
		r = 4.08 #cm
		dt = 0.07 #s before 0.07
		
		
		pTicksR = 0
		pTicksL = 0
		
		nTicksR = 0
		nTicksL = 0
		
		distanceR = 0
		pi = 3.141592654 #RAD
		
		while distanceR < distance :
			nTicksR = self.mRight.getTicks()
			nTicksL = self.mLeft.getTicks()
			
			dif = nTicksR - nTicksL
			
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
		
			
			
			print( str(distanceR))
			
			
			sleep(dt)
		
		self.mLeft.stop()
		self.mRight.stop()	
	
	def getToLineB(self, pwm): #va recto hacia atras hasta una linea y se para en la linea
		
		ds = 1 #cm
		white = True
		
		negro = 700
		
		while white:
				
			linea = self.qtr.getValues()
			
			for i in range(len(linea)):
				if int(linea[i]) >= negro:
					white = False
					break
					
			self.backwardC(ds, pwm)
			
		
		self.mLeft.stop()
		self.mRight.stop()
		
	def getToLineF(self, pwm): #va recto hasta una linea y se para en loa linea
		
		white = True
		
		negro = 1000
		
		timenow = time()
		timepast = time()
		
		### Variables  del pid 
		kp = 0.047#Buen valor 0.75
		ki = 0
		kd = 0
		epsilon = 0.5
		
		pdif = 0 #previous diference
		psum = 0 #previous sum
		
		r = 4.08 #cm
		
		pTicksR = 0
		pTicksL = 0
		
		nTicksR = 0
		nTicksL = 0
		
		distanceR = 0
		pi = 3.141592654 #RAD
		delta = 0
		###
		n = 0
		while white:
			n = n + 1
			print('n =' +str(n))
			timenow  = time()	
			linea = self.qtr.getValues()
			print('lei')
			print(linea)

			if max(linea) > 1000:
				print(max(linea))
				self.mLeft.stop()
				self.mRight.stop()
					
				print('stop?!')
				print(linea )
				
				white = False
				break
					
					
			if (timenow - timepast >= epsilon) and white == True:
				print('in' + str(timenow - timepast))
			
				nTicksR = self.mRight.getTicks()
				nTicksL = self.mLeft.getTicks()
		
				dif = nTicksR - nTicksL
				
				psum = psum + dif

				delta = dif * kp   + (dif - pdif /(timenow - timepast)) *kd + ki * psum 
				print(dif)
				print(delta)

				pdif = dif

				
				pTicksR = nTicksR
				pTicksL = nTicksL
				
				timepast = timenow
			
				self.mLeft.run(pwm + delta)  
				self.mRight.run(pwm) 
				
			else:
				n = n + 1
				
	
		
		
		print(self.qtr.getValues())
		self.mLeft.stop()
		self.mRight.stop()
		
				
				
		
	def fowardL(self, pwm): #El robot avanza hasta encontrar una linea
		kp = 0.0875 #Buen valor 0.75
		ki = 0
		kd = 0
		pdif = 0
		psum = 0
		
		
		
		dt = 0.25 #s
		
		
		pTicksR = 0
		pTicksL = 0
		
		nTicksR = 0
		nTicksL = 0
		
		
		pi = 3.141592654 #RAD
		
		linea = self.qtr.getValues()
		
	
		while True:
			
			linea = self.qtr.getValues()
			
			for i in range(len(linea)):
				if int(linea[i]) >= 500:
					break
			
			if int(linea[i]) >= 500:
					break		
			
			nTicksR = self.mRight.getTicks()
			nTicksL = self.mLeft.getTicks()
			
			dif = nTicksR - nTicksL
			
			psum = psum + dif

			delta = dif * kp + (dif - pdif / dt) * kd + ki * psum

			self.mLeft.run(pwm + delta)  

			self.mRight.run(pwm) 


			pdif = dif
			"""
			wr =  pi * (nTicksR - pTicksR) / (dt * 10 * 180) # convertimos los ticks/s en rad/segfundos 3600 ticks = 2pi
			
			wl = pi * (nTicksL - pTicksL) / (dt * 10 * 180)
			
			v = r * (wr + wl) / 2 # cm/s
			
			distanceR = v * dt + distanceR
			"""
			pTicksR = nTicksR
			pTicksL = nTicksL
		
			
			
			
			#print(linea)
			#sleep(dt)
		
		self.mLeft.stop()
		self.mRight.stop()		
	
	def backwardB(self,distance, pwm): #distance tiene que estar en centimetros
		kp = 0.1 #Buen valor 0.75
		ki = 0
		kd = 0
		pdif = 0
		psum = 0
		
		distDelta  = lambda x: 0.14692097 * (x**(0.93840107)) 
		
		distance = -distance #+ distDelta(distance) + distDelta(distDelta(distance)) 
		
		r = 4.08 #cm
		dt = 0.07 #s
		
		
		pTicksR = 0
		pTicksL = 0
		
		nTicksR = 0
		nTicksL = 0
		
		distanceR = 0
		pi = 3.141592654 #RAD
		
		while distanceR > distance :
			
			nTicksR = self.mRight.getTicks()
			nTicksL = self.mLeft.getTicks()
			
			dif= nTicksR - nTicksL
			
			psum = psum + dif

			delta = dif * kp + (dif - pdif / dt) *kd + ki * psum

			self.mLeft.run(-pwm + delta)  

			self.mRight.run(-pwm) 


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
	
	def turnLeft(self, degrees, pwm): #El robot gira n sentido antihorario "degrees" grados con una potencia "pwm" 
		
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

			self.mLeft.run(-pwm - delta)  

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
	
	def turnRight(self, degrees, pwm): #El robot gira n sentido horario "degrees" grados con una potencia "pwm" 
		
		kp = 0.1 #Buen valor 0.75
		ki = 0.00
		kd = 0
		pdif = 0
		psum = 0
		
		#correction = lambda x: 1.86586641*(x**0.32895066) - 3
		
		#degrees = degrees - correction(degrees)
		
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

			self.mLeft.run(pwm + delta)  

			self.mRight.run(-pwm) 


			pdif = dif

			wr =(nTicksR - pTicksR) / (dt * 10) # convertimos los ticks/s en grados/segfundos 3600 ticks = 360 grados
			
			wl =(nTicksL - pTicksL) / (dt * 10)
			
			w = r * (wr - wl) / L # grados/s
			
			degreesT = -w * dt + degreesT
			
			pTicksR = nTicksR
			pTicksL = nTicksL
			
			print(str(degreesT))
			
			sleep(dt)
		
		self.mLeft.stop()
		self.mRight.stop()
	
	
	
	def rightPrll (self, ticks, vel): # El robot se mueve hacia la derecha (n ticks) sin cambiar su orientacion con velocidad vel
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
		
	
	def leftPrll (self, ticks, vel): # El robot se mueve hacia la izquierda (n ticks) sin cambiar su orientacion con velocidad vel
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
	
	
		


	
