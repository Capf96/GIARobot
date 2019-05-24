import pigpio
import serial
import RPi.GPIO as GPIO

from MotorControl import Motor
from time import sleep, time
from electroIman import Magnet
from Sensores import QTR, Sensor
from subprocess import call 



GPIO.setmode(GPIO.BCM) #Queremos usar la numeracion de la placa
ser = serial.Serial('/dev/ttyUSB0', 57600)

# Encoder provides a resolution of 3600 ticks per revolution.
# Un tick es la division minima del giro del moto que proporciona el encoder
ser.flushInput()



class Robot(object):
	
	def __init__(self, mLeft, mRight, magnet, qtr, ultSndL, ultSndR, ultSndIm):
		self.mLeft    = mLeft    # Motor Izquierdo
		self.mRight   = mRight   # Motor Derecho
		self.magnet   = magnet   # Iman
		self.qtr      = qtr 	    # Sensor de linea
		
		# Los sensores se representan con un arreglo [a, b], donde a es el Trigger y b es el Echo
		self.ultSndL  = ultSndL  # Sensor Ultrasonido Izquierdo
		self.ultSndR  = ultSndR  # Sensor Ultrasonido Derecho
		self.ultSndIm = ultSndI  # Sensor Ultrasonido del Iman
		
			
		
	
	
	def forward(self,distance, pwm): 

		#Funcion    :El robot se mueve en linea recta hacia adelante a una distancia y velocidad determinadas.

		#Entradas#
		#"distance" :Distancia aproximada que se desea que el robot recorra
		#"pwm"      :Potencia de los motores 


		#Parametros del PID#
		kp = 0.75 # Constante Proporcional
		ki = 0	  # Constante Integral
		kd = 0	  # Constante Diferencial
		
		#Vairables de tiempo
		dt = 0        # Diferencial de tiempo. *Es el tiempo que espera el sistema para aplciar de nuevo los calculos de ajuste del PID.*
		epsilon = 0.7
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
		#Funcion    :El robot se mueve en linea recta hacia atras a una distancia y velocidad determinadas.

		#Entradas#
		#"distance" :Distancia aproximada que se desea que el robot recorra. Tiene que estar en centimetros.
		#"pwm"      :Potencia de los motores 
		
		#Aplicamos la misma funcion forward solo que la distancia y la potencia de los motores son negativas#
		self.forward(-distance, -pwm)
		
		
	def turnLeft(self, degrees, pwm): 
		#Funcion    :El robot gira en sentido antihorario "degrees" grados con una potencia 'pwm'.

		#Entradas#
		#"degrees"  :Grados aproximada que se desea que el robot gire. Tirne que estar en grados.
		#"pwm"      :Potencia de los motores 
		

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

		#Funcion    :El robot gira n sentido horario "degrees" grados con una potencia 'pwm'.

		#Entradas#
		#"degrees"  :Grados aproximada que se desea que el robot gire. Tirne que estar en grados.
		#"pwm"      :Potencia de los motores 
				
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
	

	def rightPrll (self, ticks, pwm): 
		
		#Funcion    :# El robot se mueve hacia la derecha 'n ticks' sin cambiar su orientacion con velocidad 'pwm'

		#Entradas#
		#"ticks"    :Ticks (1 Tick = 0.1 grados) aproximados que se desea que el robot gire en cada una de sus ruedas durante el movimiento.
		#"pwm"      :Potencia de los motores 

		#Parametros de la funcion#
		counter = 0 # Cuenta los ticks que se han medido#
		step = 500 # EL giro en ticks minimo que hace cada rueda del robot
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
			
			print(str(counter))
		
		#Se detiene el robot al terminar la ejecucion de la funcion.
		self.mLeft.stop()
		self.mRight.stop()
	
		#Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
		
	
	def leftPrll (self, ticks, pwm): 
		#Funcion    :# El robot se mueve hacia la izquierda 'n ticks' sin cambiar su orientacion con velocidad 'pwm'

		#Entradas#
		#"ticks"    :Ticks (1 Tick = 0.1 grados) aproximados que se desea que el robot gire en cada una de sus ruedas durante el movimiento.
		#"pwm"      :Potencia de los motores 

		#Parametros de la funcion#
		counter = 0 # Cuenta los ticks que se han medido#
		step = 500 # EL giro en ticks minimo que hace cada rueda del robot
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
			print(str(counter))
		#Se detiene el robot al terminar la ejecucion de la funcion.
		self.mLeft.stop()
		self.mRight.stop()
	
		#Se reinician los ticks de los encoder en 0
		self.mLeft.encoder.ticks = 0 
		self.mRight.encoder.ticks = 0
	
	
	def getToLineF(self, pwm): 
		# Va recto hasta una linea y se para en ella
		white = True
		
		negro = 1000
		
		timenow = time()
		timepast  = 0
		timepastP = 0
		
		# Variables del pid 
		kp = 0.047 # Buen valor 0.75
		ki = 0
		kd = 0
		lowEpsilon = 0.01
		highEpsilon = 0.5
		
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
			if dt >= lowEpsilon :
				n = n + 1
				# print('n =' +str(n))
				linea = self.qtr.getValues()
				print('linea dt=' + str(dt))
				# print(linea)

				if max(linea) > 1000:
					print(max(linea))
					self.mLeft.stop()
					self.mRight.stop()
					
					print('stop?!')
					print(linea)
				
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
					self.mLeft.run(pwm + delta)  # *Se busca igualar la velocidad de uno de los motores con la del otro por lo tanto las modificaciones solo se relaizan a ese motor*
					self.mRight.run(pwm) 

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
		
		
	def backToLine(self, pwm):
		# Retrocede recto hasta una linea y se para en ella
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
		negro = 1500
		dphi = 4
		ds = 0.5
		dss = 1
		epsilon = 0.05
		white = True
		timenow = time()
		timepast  = 0
		dt = 0
		linea = self.qtr.getValues()
		alingned = False
		giro = False
		while alingned == False:
			while white:
				
				if dt > epsilon:
					linea = self.qtr.getValues()
					print(linea)
					timepast = timenow
					
				if linea[0] > negro and  linea[7] < negro and giro == False :
					dss = 4
					self.backward(dss,pwm)
					sleep(0.5)
					self.turnRight(dphi, pwm)
					giro = True
					print("giro")
				
				elif linea[0] < negro and  linea[7] > negro and giro == False :
					dss = 4
					self.backward(dss,pwm)
					sleep(0.5)
					self.turnLeft(dphi, pwm)
					giro = True
					print("giro*")
				
				elif (linea[0] > negro and  linea[7] > negro) or (linea[2] > negro and  linea[6] > negro): 
					white=False
					black=False
					alingned=True
					
					print("Me alinie??")
				
				elif linea[0] < negro and  linea[7] < negro : #and giro == False: 
					self.forward(ds,pwm)
					giro = False
				
				#elif linea[0] < negro and  linea[7] < negro and giro == True: 
					#giro = False
					#dss=1
					#self.backward(dss,pwm)
				
				dt  = timenow - timepast
				timenow = time()
			
			while black:
				if dt > epsilon:
					linea = self.qtr.getValues()
					print(linea)
					timepast = timenow
				
				if linea[0] > negro and  linea[7] > negro: 
					print("voy pa atras")
					dss = 1
					self.backward(dss,pwm)
				
				elif linea[0] < negro and  linea[7] < negro:
					print("me alinie")
					dss = 2
					self.forward(dss,pwm)
					alingned=True
					black = False
				
				elif (linea[0] > negro and  linea[7] < negro) or (linea[0] < negro and  linea[7] > negro):
					black = False
					white = True
					print("la cague")
					
				dt  = timenow - timepast
				timenow = time()
				
				
	def actUltSndL(self):
		# Medimos distancia con el ultra sonido izquierdo
		
		
		# Configuramos los pines del ultra sonido izquierdo
		Trig = self.ultSndL[0]
		Echo = self.ultSndL[1]
		
		GPIO.setup(Trig, GPIO.OUT)
		GPIO.setup(Echo, GPIO.IN)
		GPIO.output(Trig, False)

	
		medida = 0
		i = 0
		start = 0
		end = 0
		
		while i < 5:
			GPIO.output(Trig, False) # Apagamos el pin Trig
			time.sleep(2*10**-6) # Esperamos dos microsegundos
			GPIO.output(Trig, True) # Encendemos el pin Trig
			time.sleep(10*10**-6) # Esperamos diez microsegundos
			GPIO.output(Trig, False) # Y lo volvemos a apagar
		 
			# Empezaremos a contar el tiempo cuando el pin Echo se encienda
			while GPIO.input(Echo) == 0:
				start = time.time()

			while GPIO.input(Echo) == 1:
				end = time.time()
		 
			#La duracion del pulso del pin Echo sera la diferencia entre el tiempo de inicio y el final
			duracion = end-start
			duracion = duracion*10**6
			medida += duracion/58 
			i += 1
		 
			time.sleep(0.1)
			
		medida = medida/5
		
		return medida


	def actUltSndR(self):
		# Medimos distancia con el ultra sonido derecho
		
		
		# Configuramos los pines del ultra sonido derecho
		Trig = self.ultSndR[0]
		Echo = self.ultSndR[1]
		
		GPIO.setup(Trig, GPIO.OUT)
		GPIO.setup(Echo, GPIO.IN)
		GPIO.output(Trig, False)

	
		medida = 0
		i = 0
		start = 0
		end = 0
		
		while i < 5:
			GPIO.output(Trig, False) # Apagamos el pin Trig
			time.sleep(2*10**-6) # Esperamos dos microsegundos
			GPIO.output(Trig, True) # Encendemos el pin Trig
			time.sleep(10*10**-6) # Esperamos diez microsegundos
			GPIO.output(Trig, False) # Y lo volvemos a apagar
		 
			# Empezaremos a contar el tiempo cuando el pin Echo se encienda
			while GPIO.input(Echo) == 0:
				start = time.time()

			while GPIO.input(Echo) == 1:
				end = time.time()
		 
			#La duracion del pulso del pin Echo sera la diferencia entre el tiempo de inicio y el final
			duracion = end-start
			duracion = duracion*10**6
			medida += duracion/58 
			i += 1
		 
			time.sleep(0.1)
			
		medida = medida/5
		
		return medida
		
		
	def alingBlock(self, pwm):
		# Nos alineamos con el bloque
	
		# Leemos el valor de los ultrasonidos laterales
		sensorL = actUltSndL(self)
		sensorR = actUltSndR(self)

		# Mientras el sensor izquierdo lea bien pero el derecho no, se movera a la izquierda
		while (13.5 < sensorL) and (sensorL < 14.5) and ( (sensorR < 13.5) or (senserR > 14.5) ):
			leftPrll(500, 50)
			sensorL = actUltSndL(self)
			sensorR = actUltSndR(self)

			
		# Mientras el sensor derecho lea bien pero el izquierdo no, se movera a la derecha
		while (13.5 < sensorR) and (sensorR < 14.5) and ( (sensorL < 13.5) or (senserL > 14.5) ):
			rightPrll(500, 50)
			sensorL = actUltSndL(self)
			sensorR = actUltSndR(self)


				
	
	
	
		


	
