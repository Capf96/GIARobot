import pigpio
import serial

from MotorControl import Motor
from time import sleep, time
from electroIman import Magnet
from Sensores import QTR, Sensor
from UltraSonic import UltrasonicS
from subprocess import call 
from multiprocessing import Process
ser = serial.Serial('/dev/ttyUSB0', 57600)
# Encoder provides a resolution of 3600 ticks per revolution.
# Un tick es la division minima del giro del moto que proporciona el encoder
ser.flushInput()


class Robot(object):	
		
	def forward(self, distance, pwm): 
		"""Mueve al robot en linea recta hacia adelante con una distancia y velocidad determinadas.
		Entradas
		distance: Distancia aproximada que se desea que el robot recorra.
		pwm: Potencia de los motores."""
		#Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
		
		#Parametros del PID#
		kp = 5 # Constante Proporcional
		ki = 0	  # Constante Integral
		kd = 0	  # Constante Diferencial
		
		#Vairables de tiempo
		dt = 0        # Diferencial de tiempo. *Es el tiempo que espera el sistema para aplciar de nuevo los calculos de ajuste del PID.*
		epsilon = 0.01
		timepast = 0 
		
		#Inizialicacion de las variables del PID#
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		pTicksR = 0 # Ticks derechos previos
		pTicksL = 0	# Ticks izquierdos previos
		
		nTicksR = 0 # Ticks derechos actuales
		nTicksL = 0 # Ticks izquierdos actuales

		
		#Radio de las ruedas del robot#
		r = 4.08 #cm
		

		#Inicializacion de a distancia recorrida
		distanceR = 0
		
		pi = 3.141592654 #Valor de pi
		
		while abs(distanceR) < abs(distance) :
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el teimpo actual y el pasado
			dt = timenow - timepast
			
			if dt >= epsilon:
				#Se lee el valor de los encoders#
				nTicksR = self.mRight.getTicks()
				nTicksL = self.mLeft.getTicks()
				
				#Error/diferencia entre la salida de los encoders de esta iteracion#
				dif = nTicksR - nTicksL
				
				#Suma de los errores(dif) anteriores#
				psum = psum + dif

				#Aplicacion de la funcion de PID#
				delta = dif * kp + (dif - pdif / dt) * kd + ki * psum

				#Se modifica la salida de los motores#
				self.mLeft.run(pwm + delta)  # *Se busca igualar la velocidad de uno de los motores con la del otro por lo tanto las modificaciones solo se relaizan a ese motor*
				self.mRight.run(pwm) 

				#La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
				pdif = dif

				#Se calcula la velocidad angular de cada rueda del robot y con esos valores se calcula la velocidad lineal del robot#
				wr = pi * (nTicksR - pTicksR) / (dt * 1800) # convertimos los ticks/s en rad/segfundos 3600 ticks = 2pi
				wl = pi * (nTicksL - pTicksL) / (dt * 1800)
				v = r * (wr + wl) / 2 						#Velocidad lineal del robot en cm/s
				
				#Integramos para hallar la distancia recorrida#
				distanceR = v * dt + distanceR
				
				#Los ticks de los sensores actuales 'nTicks' pasan a ser los ticks previos 'pTicks'#
				pTicksR = nTicksR
				pTicksL = nTicksL
			
				#el tiempo actual 'timenow' pasa a ser el tiempo previo 'timepast'#
				timenow = timepast
			
				
			
			
		#Se detiene el robot al terminar la ejecucion de la funcion.
		self.mLeft.stop()
		self.mRight.stop()
	
		#Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
			
	def backward(self,distance, pwm): 
		"""Mueve al robot en linea recta hacia atras a una distancia y velocidad determinadas.
		Entradas:
		distance: Distancia aproximada que se desea que el robot recorra. Tiene que estar en centimetros.
		pwm: Potencia de los motores"""
		
		#Aplicamos la misma funcion forward solo que la distancia y la potencia de los motores son negativas#
		self.forward(-distance, -pwm)
			
	def turnLeft(self, degrees, pwm): 
		"""Hace girar al robot en sentido antihorario, para luego apagar los motores.
		Entradas:
		degrees: Grados aproximado que se desea que el robot gire.
		pwm: Potencia de los motores."""

		#Parametros del PID#
		kp = 0.1 # Constante Proporcional
		ki = 0	  # Constante Integral
		kd = 0	  # Constante Diferencial

		#Funcion de correcion(experimental) para los grados que se tienen que recorrer
		correction = lambda x: 1.86586641*(x**0.32895066) - 3
		degrees = degrees - correction(degrees)
		
		#Parametros del robot#
		r = 4.08 #cm Radio de las ruedas
		L = 26.5 #cm Distancia entre las ruedas
		
		#Parametros del tiempo#
		dt = 0.07 #s
		
		
		#Inizialicacion de las variables del PID#
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		pTicksR = 0 # Ticks derechos previos
		pTicksL = 0	# Ticks izquierdos previos
		
		nTicksR = 0 # Ticks derechso actuales
		nTicksL = 0 # Ticks izquierdos actuales
		
		#Inicializacion de los grados girados#
		degreesT = 0
		aux = True
		
		while abs(degreesT) < abs(degrees) :
			#Se lee el valor de los encoders#
			
			nTicksR = self.mRight.getTicks()
			nTicksL = self.mLeft.getTicks()
			
			#Error/diferencia entre la salida de los encoders de esta iteracion#
			dif = abs(nTicksR) - abs(nTicksL)
			
			#Suma de los errores(dif) anteriores#
			psum = psum + dif

			#Aplicacion de la funcion de PID#
			delta = dif * kp + (dif - pdif / dt) *kd + ki * psum

			#Se modifica la salida de los motores#
			self.mLeft.run(-pwm - delta)  
			self.mRight.run(pwm) 

			#La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
			pdif = dif

			#Se calcula la velocidad angular de cada rueda del robot y con esos valores se calcula la velocidad angular del robot#
			wr =(nTicksR - pTicksR) / (dt * 10) # convertimos los ticks/s en grados/segfundos 3600 ticks = 360 grados
			wl =(nTicksL - pTicksL) / (dt * 10)
			w = r * (wr - wl) / L # grados/s
			
			#Integramos para hallar los grados recorridos#
			degreesT = w * dt + degreesT
			
			
			#Los ticks de los sensores actuales 'nTicks' pasan a ser los ticks previos 'pTicks'#
			pTicksR = nTicksR
			pTicksL = nTicksL
			
			sleep(dt)
		
		#Detenemos los motores
		self.mLeft.stop()
		self.mRight.stop()
		#Reinicio los ticks del encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0	
	
	def turnRight(self, degrees, pwm):  
		"""Hace girar al robot en sentido horario, para luego apagar los motores.
		Entradas:
		degrees: Grados aproximado que se desea que el robot gire.
		pwm: Potencia de los motores."""
				
		#Aplicamos la misma funcion turnLeft solo que la distancia y la potencia de los motores son negativas#

		#Parametros del PID#
		kp = 0.1 # Constante Proporcional
		ki = 0	  # Constante Integral
		kd = 0	  # Constante Diferencial
		
		#Funcion de correcion(experimental) para los grados que se tienen que recorrer
		correction = lambda x: 1.86586641*(x**0.32895066) - 3
		degrees = degrees - correction(degrees)
		
		#Parametros del robot#
		r = 4.08 #cm Radio de las ruedas
		L = 26.5 #cm Distancia entre las ruedas
		
		#Parametros del tiempo#
		dt = 0.07 #s
		
		#Inizialicacion de las variables del PID#
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		pTicksR = 0 # Ticks derechos previos
		pTicksL = 0	# Ticks izquierdos previos
		
		nTicksR = 0 # Ticks derechso actuales
		nTicksL = 0 # Ticks izquierdos actuales
		
		#Inicializacion de los grados girados#
		degreesT = 0
		
		while abs(degreesT) < abs(degrees) :
			#Se lee el valor de los encoders#
			nTicksR = self.mRight.getTicks()
			nTicksL = self.mLeft.getTicks()
			
			#Error/diferencia entre la salida de los encoders de esta iteracion#
			dif = abs(nTicksR) - abs(nTicksL)
			
			#Suma de los errores(dif) anteriores#
			psum = psum + dif

			#Aplicacion de la funcion de PID#
			delta = dif * kp + (dif - pdif / dt) *kd + ki * psum

			#Se modifica la salida de los motores#
			self.mLeft.run(pwm + delta)  
			self.mRight.run(-pwm) 

			#La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
			pdif = dif

			#Se calcula la velocidad angular de cada rueda del robot y con esos valores se calcula la velocidad angular del robot#
			wr =(nTicksR - pTicksR) / (dt * 10) # convertimos los ticks/s en grados/segfundos 3600 ticks = 360 grados
			wl =(nTicksL - pTicksL) / (dt * 10)
			w = r * (wr - wl) / L # grados/s
			
			#Integramos para hallar los grados recorridos#
			degreesT = w * dt + degreesT
			
			#Los ticks de los sensores actuales 'nTicks' pasan a ser los ticks previos 'pTicks'#
			pTicksR = nTicksR
			pTicksL = nTicksL
			
			sleep(dt)
	
		#Detenemos los motores
		self.mLeft.stop()
		self.mRight.stop()
		#Reinicio los ticks del encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0	
	
	def turnLeftNonStop(self, degrees, pwm): 
		"""Hace girar al robot en sentido antihorario.
		Entradas:
		degrees: Grados aproximado que se desea que el robot gire.
		pwm: Potencia de los motores."""
		
		#Parametros del PID#
		kp = 0.1 # Constante Proporcional
		ki = 0	  # Constante Integral
		kd = 0	  # Constante Diferencial
		
		#Funcion de correcion(experimental) para los grados que se tienen que recorrer
		correction = lambda x: 1.86586641*(x**0.32895066) - 3
		degrees = degrees - correction(degrees)
		
		#Parametros del robot#
		r = 4.08 #cm Radio de las ruedas
		L = 26.5 #cm Distancia entre las ruedas
		
		#Parametros del tiempo#
		dt = 0.07 #s
		
		
		#Inizialicacion de las variables del PID#
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		pTicksR = 0 # Ticks derechos previos
		pTicksL = 0	# Ticks izquierdos previos
		
		nTicksR = 0 # Ticks derechso actuales
		nTicksL = 0 # Ticks izquierdos actuales
		
		#Inicializacion de los grados girados#
		degreesT = 0
		aux = True
		
		while abs(degreesT) < abs(degrees) :
			#Se lee el valor de los encoders#
			
			nTicksR = self.mRight.getTicks()
			nTicksL = self.mLeft.getTicks()
			
			#Error/diferencia entre la salida de los encoders de esta iteracion#
			dif = abs(nTicksR) - abs(nTicksL)
			
			#Suma de los errores(dif) anteriores#
			psum = psum + dif

			#Aplicacion de la funcion de PID#
			delta = dif * kp + (dif - pdif / dt) *kd + ki * psum

			#Se modifica la salida de los motores#
			self.mLeft.run(-pwm - delta)  
			self.mRight.run(pwm) 

			#La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
			pdif = dif

			#Se calcula la velocidad angular de cada rueda del robot y con esos valores se calcula la velocidad angular del robot#
			wr =(nTicksR - pTicksR) / (dt * 10) # convertimos los ticks/s en grados/segfundos 3600 ticks = 360 grados
			wl =(nTicksL - pTicksL) / (dt * 10)
			w = r * (wr - wl) / L # grados/s
			
			#Integramos para hallar los grados recorridos#
			degreesT = w * dt + degreesT
			
			#Los ticks de los sensores actuales 'nTicks' pasan a ser los ticks previos 'pTicks'#
			pTicksR = nTicksR
			pTicksL = nTicksL
			
			sleep(dt)
		
		#Reinicio los ticks del encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
				
	def turnRightNonStop(self, degrees, pwm):  
		"""Hace girar al robot en sentido horario.
		Entradas:
		degrees: Grados aproximado que se desea que el robot gire.
		pwm: Potencia de los motores."""
				
		#Aplicamos la misma funcion turnLeft solo que la distancia y la potencia de los motores son negativas#

		#Parametros del PID#
		kp = 0.1 # Constante Proporcional
		ki = 0	  # Constante Integral
		kd = 0	  # Constante Diferencial

		#Funcion de correcion(experimental) para los grados que se tienen que recorrer
		correction = lambda x: 1.86586641*(x**0.32895066) - 3
		degrees = degrees - correction(degrees)
		
		#Parametros del robot#
		r = 4.08 #cm Radio de las ruedas
		L = 26.5 #cm Distancia entre las ruedas
		
		#Parametros del tiempo#
		dt = 0.07 #s
		
		#Inizialicacion de las variables del PID#
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		pTicksR = 0 # Ticks derechos previos
		pTicksL = 0	# Ticks izquierdos previos
		
		nTicksR = 0 # Ticks derechso actuales
		nTicksL = 0 # Ticks izquierdos actuales
		
		#Inicializacion de los grados girados#
		degreesT = 0
		
		while abs(degreesT) < abs(degrees) :
			#Se lee el valor de los encoders#
			nTicksR = self.mRight.getTicks()
			nTicksL = self.mLeft.getTicks()
			
			#Error/diferencia entre la salida de los encoders de esta iteracion#
			dif = abs(nTicksR) - abs(nTicksL)
			
			#Suma de los errores(dif) anteriores#
			psum = psum + dif

			#Aplicacion de la funcion de PID#
			delta = dif * kp + (dif - pdif / dt) *kd + ki * psum

			#Se modifica la salida de los motores#
			self.mLeft.run(pwm + delta)  
			self.mRight.run(-pwm) 

			#La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
			pdif = dif

			#Se calcula la velocidad angular de cada rueda del robot y con esos valores se calcula la velocidad angular del robot#
			wr =(nTicksR - pTicksR) / (dt * 10) # convertimos los ticks/s en grados/segfundos 3600 ticks = 360 grados
			wl =(nTicksL - pTicksL) / (dt * 10)
			w = r * (wr - wl) / L # grados/s
			
			#Integramos para hallar los grados recorridos#
			degreesT = w * dt + degreesT
			
			#Los ticks de los sensores actuales 'nTicks' pasan a ser los ticks previos 'pTicks'#
			pTicksR = nTicksR
			pTicksL = nTicksL
			
			sleep(dt)
	
		#Reinicio los ticks del encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
	
	def rightPrll (self, ticks, pwm): 
		"""Mueve el robot hacia la derecha sin cambiar su orientacion.
		Entradas:
		ticks: Ticks (1 Tick = 0.1 grados) aproximados que se desea que el robot gire en cada una de sus 
			ruedas durante el movimiento.
		pwm: Potencia de los motores. """

		#Parametros de la funcion#
		counter = 0 # Cuenta los ticks que se han medido#
		step = 1000 # EL giro en ticks minimo que hace cada rueda del robot
		dt = 0.07	# Tiempo de espera entre cada cambio de velocidad de los motores 
		
		while counter < ticks:

			self.mLeft.turnTicks(step, pwm)
			sleep(dt)
			self.mRight.turnTicks(step, pwm)
			sleep(dt)
			
			counter = counter + abs(self.mRight.getTicks())
			
			self.mLeft.turnTicks(-step, -pwm)
			sleep(dt)
			self.mRight.turnTicks(-step, -pwm)
			sleep(dt)
			
			counter = counter + abs(self.mRight.getTicks())
	
		#Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
			
	def leftPrll (self, ticks, pwm): 
		"""Funcion que mueve el robot hacia la izquierda sin cambiar su orientacion.
		Entradas:
		ticks: Ticks (1 Tick = 0.1 grados) aproximados que se desea que el robot gire en cada una de sus 
			ruedas durante el movimiento.
		pwm: Potencia de los motores. """

		#Parametros de la funcion#
		counter = 0 # Cuenta los ticks que se han medido#
		step = 1000 # EL giro en ticks minimo que hace cada rueda del robot
		dt = 0.07	# Tiempo de espera entre cada cambio de velocidad de los motores 
		
		while counter < ticks:
			self.mRight.turnTicks(step, pwm)
			sleep(dt)
			self.mLeft.turnTicks(step, pwm)
			sleep(dt)
			
			counter = counter + abs(self.mLeft.getTicks())
			
			self.mRight.turnTicks(-step, -pwm)
			sleep(dt)
			self.mLeft.turnTicks(-step, -pwm)
			sleep(dt)
			
			counter = counter + abs(self.mLeft.getTicks())
	
		#Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
	
	def getToLineF(self, pwm):
		"""Hace mover el robot hacia adelante hasta conseguir una linea.
		
		Entradas:
		pwm: Potencia de los motores."""

		white = True
		
		negro = 1000
		
		timenow = time()
		timepast  = 0
		timepastP = 0
		
		# Variables del pid 
		kp = 0.05 # Buen valor 0.75
		ki = 0
		kd = 0
		epsilon = 0.01
		
		pdif = 0 # Previous diference
		psum = 0 # Previous sum
		
		r = 4.08 # cm
		
		pTicksR = 0
		pTicksL = 0
		
		nTicksR = 0
		nTicksL = 0
		
		pi = 3.141592654 # RAD
		delta = 0
		n = 0
		
		while white:
		
			timenow  = time()	
			dt = timenow- timepast
			
			if dt >= epsilon :
				
				n = n + 1
				linea = self.qtr.getValues()

				if max(linea) > 1000:

					self.mLeft.stop()
					self.mRight.stop()
				
					white = False
					self.mLeft.stop()
					self.mRight.stop()
					break
			
			if dt >= epsilon and white == True:

					# Se lee el valor de los encoders#
					nTicksR = self.mRight.getTicks()
					nTicksL = self.mLeft.getTicks()
					
					# Error/diferencia entre la salida de los encoders de esta iteracion#
					dif = nTicksR - nTicksL
					
					# Suma de los errores(dif) anteriores#
					psum = psum + dif

					# Aplicacion de la funcion de PID#
					delta = dif * kp + (dif - pdif / dt) * kd + ki * psum

					# Se modifica la salida de los motores#
					self.mLeft.run(pwm + delta)  # *Se busca igualar la velocidad de uno de los motores con la del otro por lo tanto las modificaciones solo se relaizan a ese motor*
					self.mRight.run(pwm) 

					# La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
					pdif = dif
					
					
					# Los ticks de los sensores actuales 'nTicks' pasan a ser los ticks previos 'pTicks'#
					pTicksR = nTicksR
					pTicksL = nTicksL
					
					# El tiempo actual 'timenow' pasa a ser el tiempo previo 'timepast'#

			timepast = timenow
				
		# Se detiene el robot al terminar la ejecucion de la funcion.
		self.mLeft.stop()
		self.mRight.stop()
	
		# Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
				
	def backToLine(self, pwm):
		"""Hace retroceder al robot hasta una linea y se para en ella.
		
		Entradas:
		pwm: Potencia de los motores."""
		
		white = True
		
		negro = 1000
		
		timenow = time()
		timepast  = 0
		timepastP = 0
		
		# Variables  del pid 
		kp = 0.047 #Buen valor 0.75
		ki = 0
		kd = 0
		lowEpsilon = 0.01
		highEpsilon = 0.5
		
		pdif = 0 # Previous diference
		psum = 0 # Previous sum
		
		r = 4.08 # CM
		
		pTicksR = 0
		pTicksL = 0
		
		nTicksR = 0
		nTicksL = 0
		
	
		pi = 3.141592654 # RAD
		delta = 0

		n = 0
		while white:
		
			timenow  = time()	
		
			
			dt = timenow- timepast
			if dt >= lowEpsilon :
				n = n + 1
				# print('n =' +str(n))
				linea = self.qtr.getValues()
				# print('linea dt=' + str(dt))
				# print(linea)

				if max(linea) > 1000:
					# print(max(linea))
					self.mLeft.stop()
					self.mRight.stop()
					
					# print('stop?!')
					# print(linea)
				
					white = False
					self.mLeft.stop()
					self.mRight.stop()
					break
			
			if dt >= highEpsilon and white == True:
					print('PID dt='+ str(dt))
					# Se lee el valor de los encoders#
					nTicksR = self.mRight.getTicks()
					nTicksL = self.mLeft.getTicks()
					
					# Error/diferencia entre la salida de los encoders de esta iteracion#
					dif = nTicksR - nTicksL
					
					# Suma de los errores(dif) anteriores#
					psum = psum + dif

					# Aplicacion de la funcion de PID#
					delta = dif * kp + (dif - pdif / dt) * kd + ki * psum

					# Se modifica la salida de los motores#
					self.mLeft.run(-pwm - delta)  # *Se busca igualar la velocidad de uno de los motores con la del otro por lo tanto las modificaciones solo se relaizan a ese motor*
					self.mRight.run(-pwm) 

					# La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
					pdif = dif
					
					
					# Los ticks de los sensores actuales 'nTicks' pasan a ser los ticks previos 'pTicks'#
					pTicksR = nTicksR
					pTicksL = nTicksL
					
					# El tiempo actual 'timenow' pasa a ser el tiempo previo 'timepast'#
					
			
			
			
				 
				
			timepast = timenow
				
			
		# print(self.qtr.getValues())
		# Se detiene el robot al terminar la ejecucion de la funcion.
		self.mLeft.stop()
		self.mRight.stop()
	
		# Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
			
	def aling(self, pwm):
		"""Hace alinear al robot con respecto a una linea.
		
		Entradas:
		pwm: Potencia de los motores."""
		
		negro = 700 # Valor minimo que deben dar los sensores cuando detectan una linea negra
		ds = 0.1 # Distancia que avanzara el robot cuando lee completamente blanco
		white = True # Suponemos que varios sensores estan leyendo blanco al principio
		timenow = time() # Guardamos el tiempo actual
		timepast  = 0 # Variable que guardara el tiempo pasado
		dt = 0 # Variable que guardara el lapso de tiempo desde la ultima actualizacion de los sensores
		linea = self.qtr.getValues() # Guardamos el valor de los sensores
		alingned = False # Suponemos que el robot no esta alineado al principio
		dphi=1
		while alingned == False:
			
			# Mientras leas blanco en los sensores laterales
			while white:
				
				# Actualizar la linea
				linea = self.qtr.getValues()
				
				# Si los sensores laterales leen blanco, sigue hacia adelante
				if linea[0] < negro and linea[7] < negro:
					self.forward(ds, pwm)
					
				# Si no, salimos del while white
				else:
					white = False
					
			# Actualizar la linea
			linea = self.qtr.getValues()
			
			# Si la linea izquierda lee negro y la derecha lee blanco, gira a la derecha
			if linea[0] > negro and linea[7] < negro:
				self.turnRight(dphi, pwm)
			# Si la linea izquierda lee blanco y la derecha lee negro, gira a la izquierda
			elif linea[0] < negro and linea[7] > negro:
				self.turnLeft(dphi, pwm)
			# Si ambos sensores laterales leen negro, el robot esta alineado
			elif linea[0] > negro and linea[7] > negro:
				alingned = True
				
			# Si algun sensor lee blanco, repetimos toda el procedimiento
			if any(sensor < negro for sensor in linea) :		
				alingned = False
				white = True
			
		# Paramos los motores	
		self.stop()
						
	def alingBlock(self, pwm):
		"""Alineamos el robot con el bloque que tiene en frente.
		
		Entradas:
		pwm: Potencia de los motores."""
		
		# Rango permitido para lectura de los sensores
		firstLecLeft = 0
		firstLecRight = 0
		for i in range(3):
			firstLecLeft += self.ultSndL.getDistance()/3
			firstLecRight += self.ultSndR.getDistance()/3
		medio = min(firstLecRight, firstLecLeft)
		rango = 2.5
		
		inf = medio - rango
		sup = medio + rango
	
		# Leemos el valor de los ultrasonidos laterales
		sensorL = self.ultSndL.getDistance()
		sensorR = self.ultSndR.getDistance()
		
		# Indica a donde se movio
		izq = False

		# Mientras el sensor izquierdo lea bien pero el derecho no, se movera a la izquierda
		while (inf < sensorL) and (sensorL < sup) and ( (sensorR < inf) or (sensorR > sup) ):
			self.leftPrll(2000, 50)
			sensorL = self.ultSndL.getDistance()
			sensorR = self.ultSndR.getDistance()
			izq = True
			
		# Mientras el sensor derecho lea bien pero el izquierdo no, se movera a la derecha
		while (inf < sensorR) and (sensorR < sup) and ( (sensorL < inf) or (sensorL > sup) ):
			self.rightPrll(2000, 50)
			sensorL = self.ultSndL.getDistance()
			sensorR = self.ultSndR.getDistance()
			izq = False
			
		if izq:
			self.leftPrll(2000, 50)
		else:
			self.rightPrll(2000, 50)
			
		self.stop()
			
	def seguirLinea(self, pwm, kp, ki, kd, integral):
		"""Hace mover al robot a traves de una linea.
		
		Entrandas:
		pwm: Potencia de los motores.
		kp: Constante Proporcional.
		ki: Constante Integral.
		kd: Constante Diferencial.
		integral: Sirve para contrarrestar el error, en caso de ser muy grande."""

		position = self.qtr.average() 	# Revisamos en que sensor esta centrado el qtr ("Que sensor esta en negro")

		# Calculamos el valor proporcional, integarl y derivativo del PID basados en la poscicon del sensor
		proporcional = position - 2500
		integral=integral + self.proporcional_pasado;  
		derivativo = (proporcional - self.proporcional_pasado)

		#Se acota el valor del valor integral del PID
		if integral>1000: 
			integral=1000
		if integral<-1000: 
			integral=-1000
		#Calculamos la funcion PID
		delta_pwm = ( proporcional * kp ) + ( derivativo * kd ) + ( integral * ki )

	
		#Evaluamos casos para que el robot no se atrase
		if (delta_pwm <= 0):
			self.mLeft.run(pwm - delta_pwm)
			self.mRight.run(pwm)
		
		if (delta_pwm >0):
			
			self.mLeft.run(pwm)
			self.mRight.run(pwm + delta_pwm)

		self.proporcional_pasado = proporcional

	def detectarEsquina(self, c):
		"""Retorna True """
		sensors = self.qtr.getValues()
		if sensors[0]>c or sensors[7]>c: return True
		else:                            return False

	def seguirLineaEsq(self, der, bloque):
		#Funcion    :El robot se sigue la linea hasta detectar una esquina

		#Entradas#
		#"der"   :Distancia aproximada que se desea que el robot recorra
		#"bloque":Potencia de los motores 


		#Parametros del PID#
		kp = 0   # Constante Proporcional
		ki = 0	  # Constante Integral
		kd = 0	  # Constante Diferencial
		integral = 0
		
		#Parametros dl pwm
		pwm = 50
		
		#Vairables de tiempo
		dt = 0        # Diferencial de tiempo. *Es el tiempo que espera el sistema para aplciar de nuevo los calculos de ajuste del PID.*
		epsilon = 0.7
		timepast = 0 

		d = 20
		
		while not detectarEsquina(1000, 0.1, 0, 0):
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el teimpo actual y el pasado
			dt = timenow - timepast
			#Si se supera el epsilon se hace el calculo del PID
			if dt >= epsilon:
				self.seguirLinea(pwm, kp, ki, kd, integral)
				timenow = timepast
			
			
			
		if bloque: avanzar(d)
		else: girar(not der)
		
	def followLineCorner(self, pwm, der, bloque):		
		
		#Vairables de tiempo
		dt = 0        # Diferencial de tiempo. *Es el tiempo que espera el sistema para aplciar de nuevo los calculos de ajuste del PID.*
		integral=0
		proporcional_pasado = 0 
		timepast = 0
		
		while not all(k < 600 for k in self.qtr.getValues()):	
			proporcional_pasado, timepast = self.followLine(pwm, dt, integral, proporcional_pasado, timepast)

		#Se detiene el robot al terminar la ejecucion de la funcion.
		self.mLeft.stop()
		self.mRight.stop()
	
		#Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
		
		self.forward(9,pwm)
		self.stop()
		
	def turnUntilLine(self,pwm,derecha): #Si derecha es true gira a la derecha si no a la izquierda
		#Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
		
		db=1
		for i in range(15):
			
			linea = self.qtr.getValues()
	
		if derecha:
			
			while all(i < 1000 for i in linea):
				
				self.turnRightNonStop(db, pwm)
				linea = self.qtr.getValues()
				

		else:
			
			while all(i < 1000 for i in linea):
				
				self.turnLeftNonStop(db, pwm)
				linea = self.qtr.getValues()

			
		self.stop()
	
	def blockDetection(self):#,rightSide): #rightSide es un boleano que indic si el bloque esta en el lado derecho del robot
		minDistance = 15 #cm
		
		distance = self.ultSndLL.getDistance()
		if distance <= minDistance:

			return True
		else:

			return False
			
	def blockDetecUtilTrue(self):#,rightSide): #rightSide es un boleano que indic si el bloque esta en el lado derecho del robot
		minDistance = 15 #cm
		
		distance = self.ultSndLL.getDistance()
		while distance > minDistance:

			distance = self.ultSndLL.getDistance()
		

		return True
						
	def forwardUntilBlock(self, pwm):# blockRight): #blockRight es un booleano que indica si el bloque esta en el lado derecho del robot
			#Parametros del PID#
		kp = 0.05 # Constante Proporcional
		ki = 0	  # Constante Integral
		kd = 0	  # Constante Diferencial
		
		#Vairables de tiempo
		dt = 0        # Diferencial de tiempo. *Es el tiempo que espera el sistema para aplciar de nuevo los calculos de ajuste del PID.*
		epsilon = 0.01
		timepast = 0 
		
		#Inizialicacion de las variables del PID#
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		pTicksR = 0 # Ticks derechos previos
		pTicksL = 0	# Ticks izquierdos previos
		
		nTicksR = 0 # Ticks derechos actuales
		nTicksL = 0 # Ticks izquierdos actuales

		
		#Radio de las ruedas del robot#
		r = 4.08 #cm
		

		#Inicializacion de a distancia recorrida
		distanceR = 0
		
		pi = 3.141592654 #Valor de pi
		
		
		while self.blockDetection()==False:
		
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el teimpo actual y el pasado
			dt = timenow - timepast
			
			if dt >= epsilon:
				#Se lee el valor de los encoders#
				nTicksR = self.mRight.getTicks()
				nTicksL = self.mLeft.getTicks()
				
				#Error/diferencia entre la salida de los encoders de esta iteracion#
				dif = nTicksR - nTicksL
				
				#Suma de los errores(dif) anteriores#
				psum = psum + dif

				#Aplicacion de la funcion de PID#
				delta = dif * kp + (dif - pdif / dt) * kd + ki * psum

				#Se modifica la salida de los motores#
				self.mLeft.run(pwm + delta)  # *Se busca igualar la velocidad de uno de los motores con la del otro por lo tanto las modificaciones solo se relaizan a ese motor*
				self.mRight.run(pwm) 

				#La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
				pdif = dif

				#Se calcula la velocidad angular de cada rueda del robot y con esos valores se calcula la velocidad lineal del robot#
				wr = pi * (nTicksR - pTicksR) / (dt * 1800) # convertimos los ticks/s en rad/segfundos 3600 ticks = 2pi
				wl = pi * (nTicksL - pTicksL) / (dt * 1800)
				v = r * (wr + wl) / 2 						#Velocidad lineal del robot en cm/s
				
				#Integramos para hallar la distancia recorrida#
				distanceR = v * dt + distanceR
				
				#Los ticks de los sensores actuales 'nTicks' pasan a ser los ticks previos 'pTicks'#
				pTicksR = nTicksR
				pTicksL = nTicksL
			
				#el tiempo actual 'timenow' pasa a ser el tiempo previo 'timepast'#
				timenow = timepast
			
				
			
			
		#Se detiene el robot al terminar la ejecucion de la funcion.
		self.mLeft.stop()
		self.mRight.stop()
	
		#Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
		
	def followLineUntilBlock(self, pwm):
		
		p1 = Process(target=self.followLineCorner, args=(pwm, False, False)) 
		p1.start()
		p2 = Process(target=self.blockDetecUtilTrue)
		p2.start()

		while p2.is_alive() and  p1.is_alive():
			pass
	
		if p2.is_alive()==True:
			p2.terminate()
		else:	
			p1.terminate()

		self.stop()
		
	def forwardNonStop(self, distance, pwm): 
		"""Mueve al robot en linea recta hacia adelante con una distancia y velocidad determinadas.
		Entradas
		distance: Distancia aproximada que se desea que el robot recorra.
		pwm: Potencia de los motores."""
		#Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
		
		#Parametros del PID#
		kp = 5 # Constante Proporcional
		ki = 0	  # Constante Integral
		kd = 0	  # Constante Diferencial
		
		#Vairables de tiempo
		dt = 0        # Diferencial de tiempo. *Es el tiempo que espera el sistema para aplciar de nuevo los calculos de ajuste del PID.*
		epsilon = 0.01
		timepast = 0 
		
		#Inizialicacion de las variables del PID#
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		pTicksR = 0 # Ticks derechos previos
		pTicksL = 0	# Ticks izquierdos previos
		
		nTicksR = 0 # Ticks derechos actuales
		nTicksL = 0 # Ticks izquierdos actuales

		
		#Radio de las ruedas del robot#
		r = 4.08 #cm
		

		#Inicializacion de a distancia recorrida
		distanceR = 0
		
		pi = 3.141592654 #Valor de pi
		
		while abs(distanceR) < abs(distance) :
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el teimpo actual y el pasado
			dt = timenow - timepast
			
			if dt >= epsilon:
				#Se lee el valor de los encoders#
				nTicksR = self.mRight.getTicks()
				nTicksL = self.mLeft.getTicks()
				
				#Error/diferencia entre la salida de los encoders de esta iteracion#
				dif = nTicksR - nTicksL
				
				#Suma de los errores(dif) anteriores#
				psum = psum + dif

				#Aplicacion de la funcion de PID#
				delta = dif * kp + (dif - pdif / dt) * kd + ki * psum

				#Se modifica la salida de los motores#
				self.mLeft.run(pwm + delta)  # *Se busca igualar la velocidad de uno de los motores con la del otro por lo tanto las modificaciones solo se relaizan a ese motor*
				self.mRight.run(pwm) 

				#La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
				pdif = dif

				#Se calcula la velocidad angular de cada rueda del robot y con esos valores se calcula la velocidad lineal del robot#
				wr = pi * (nTicksR - pTicksR) / (dt * 1800) # convertimos los ticks/s en rad/segfundos 3600 ticks = 2pi
				wl = pi * (nTicksL - pTicksL) / (dt * 1800)
				v = r * (wr + wl) / 2 						#Velocidad lineal del robot en cm/s
				
				#Integramos para hallar la distancia recorrida#
				distanceR = v * dt + distanceR
				
				#Los ticks de los sensores actuales 'nTicks' pasan a ser los ticks previos 'pTicks'#
				pTicksR = nTicksR
				pTicksL = nTicksL
			
				#el tiempo actual 'timenow' pasa a ser el tiempo previo 'timepast'#
				timenow = timepast
			
				
	
		#Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
		
	def forwardContinuo(self):
		pwm = 18
		#Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
		
		#Parametros del PID#
		kp = 5 # Constante Proporcional
		ki = 0	  # Constante Integral
		kd = 0	  # Constante Diferencial
		
		#Vairables de tiempo
		dt = 0        # Diferencial de tiempo. *Es el tiempo que espera el sistema para aplciar de nuevo los calculos de ajuste del PID.*
		epsilon = 0.01
		timepast = 0 
		
		#Inizialicacion de las variables del PID#
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		pTicksR = 0 # Ticks derechos previos
		pTicksL = 0	# Ticks izquierdos previos
		
		nTicksR = 0 # Ticks derechos actuales
		nTicksL = 0 # Ticks izquierdos actuales
		
		distanceR = 0

		
		#Radio de las ruedas del robot#
		r = 4.08 #cm
		
		pi = 3.141592654 #Valor de pi
		
		while True:
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el teimpo actual y el pasado
			dt = timenow - timepast
			
			if dt >= epsilon:
				#Se lee el valor de los encoders#
				nTicksR = self.mRight.getTicks()
				nTicksL = self.mLeft.getTicks()
				
				#Error/diferencia entre la salida de los encoders de esta iteracion#
				dif = nTicksR - nTicksL
				
				#Suma de los errores(dif) anteriores#
				psum = psum + dif

				#Aplicacion de la funcion de PID#
				delta = dif * kp + (dif - pdif / dt) * kd + ki * psum

				#Se modifica la salida de los motores#
				self.mLeft.run(pwm + delta)  # *Se busca igualar la velocidad de uno de los motores con la del otro por lo tanto las modificaciones solo se relaizan a ese motor*
				self.mRight.run(pwm) 

				#La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
				pdif = dif

				#Se calcula la velocidad angular de cada rueda del robot y con esos valores se calcula la velocidad lineal del robot#
				wr = pi * (nTicksR - pTicksR) / (dt * 1800) # convertimos los ticks/s en rad/segfundos 3600 ticks = 2pi
				wl = pi * (nTicksL - pTicksL) / (dt * 1800)
				v = r * (wr + wl) / 2 						#Velocidad lineal del robot en cm/s
				
				#Integramos para hallar la distancia recorrida#
				distanceR = v * dt + distanceR
				
				#Los ticks de los sensores actuales 'nTicks' pasan a ser los ticks previos 'pTicks'#
				pTicksR = nTicksR
				pTicksL = nTicksL
			
				#el tiempo actual 'timenow' pasa a ser el tiempo previo 'timepast'#
				timenow = timepast
	
	def getNearBlock(self, pwm):
		"""
		El robot se acercara al bloque hasta estar una distancia menor a limit del mismo.
		"""	
		limit = 5.5 #cm
				
		while self.ultSndR.getDistance() > limit:
			self.forward(1, pwm)

		self.stop()
		

	

		
			
				
	
	