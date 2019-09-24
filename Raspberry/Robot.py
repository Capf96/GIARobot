from MotorControl import Motors
from ArduinoReader import Arduino
from StepperMotor import Grua
from time import sleep, time
from multiprocessing import Process
import pigpio
import serial

motores = Motors(20,26) # Pines GPIO17 y GPIO27
arduino = Arduino()
grua = Grua(24, 23, 22, 27)
ser = serial.Serial('/dev/ttyACM0', 57600)

class Robot(object):
	
	################################ FUNCIONES BASICAS #################################
	def angToTime(self, angle, clockwise):		# CHECK
		if clockwise:
			b0 = -14.9090909091        
			b1 = 152.272727273
			
		else:
			b0 = -10.4545454545        
			b1 = 133.727272727
		
		return (angle - b0)/b1

				
	def distToTime(self, dist, foward):			# CHECK
		if foward:
			b0 = -2.57272727273        
			b1 = 28.6818181818

		else:
			b0 = -3.09090909091        
			b1 = 27.5181818182

			
		return (dist - b0)/b1
	
		
	def difGrados(self, now, prev, foward):		# CHECH
		"""Obtiene la diferencia entre una posicion pasada y una actual dada en grados.
		
		ARGUMENTOS:
		prev: Posicion previa.
		now: Posicion actual. 
		foward: True si la rueda gira hacia adelante; False en caso contrario."""
		
		dif = ((-1) ** int(foward)) * (prev - now)
		
		if dif >= 250 or (-110 <= dif and dif <= 0):
			return 0
			
		
		if dif >= 0:
			return dif
		else:
			return dif + 360
			
	
	def stop(self):								# CHECK
		"""Detiene y apaga todos los componentes del robot."""
		
		# iman.stop()
		# grua.nivel(0)
		# grua.setStep(0, 0, 0, 0)
		motores.stop()
		
		
	def moveStraight(self, foward, dist = 0, slow = True):	# CHECK
		"""Mueve al robot en linea recta.
		
		ARGUMENTOS:
		pwm: Velocidad.
		foward: True si el robot avanza hacia adelante; False en caso contrario.
		dist: Indica la distancia que se va a mover el robot; 0 indica distancia indeterminada (Valor predeterminado: 0)."""
		
		# Definimos la velocidad de los motores.
		if slow:
			velLF, velRF, velLB, velRB = 20, 21, -25.05, -20
		else:
			velLF, velRF, velLB, velRB = 50, 70, -40.31, -38.55
			
			
			
		if foward:
			motores.setMotorL(velLF)  	# 	50.00	|	20.00
			motores.setMotorR(velRF)	# 	70.00	| 	21.00
			
			t = self.distToTime(dist - 0.5, foward)
			
		else:
			motores.setMotorL(velLB)	#	-40.31	|	-25.05
			motores.setMotorR(velRB)	#	-38.55	|	-20
			
			t = self.distToTime(dist, foward)

			
		while not bool(dist):
			pass
			
		sleep(t)

		motores.stop()
		
		
	def followLine(self, pwm, dummy = False):	# CHECK
		"""Hace al robot seguir una linea negra.
		
		ARGUMENTOS:
		pwm: Velocidad."""
		
		kp = 0.015
		ki = 0
		kd = 0.0005
		proporcional_pasado = 0
		integral = 0

		# Variables de tiempo
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar
						# de nuevo los calculos de ajuste del PID.*
		epsilon = 0
		timepast = 0 
		
		n = 15
		
		ser.reset_input_buffer()
		

		while True:
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el tiempo actual y el pasado
			dt = timenow - timepast

			if dt >= epsilon:

				position = float(arduino.getAverageQTR())

				# Calculamos el valor proporcional, integarl y derivativo del PID basados en la poscicon del sensor, centro 3500
				proporcional = position - 3500
				integral = integral + proporcional_pasado;  
				derivativo = (proporcional - proporcional_pasado)

				# Se acota el valor del valor integral del PID
				if integral > 1000: 
					integral = 1000
				if integral < -1000: 
					integral = -1000
					
				# Calculamos la funcion PID
				delta_pwm = ( proporcional * kp ) + ( derivativo * kd ) + ( integral * ki )
						
				# Evaluamos casos para que el robot no se atrase
				motores.setMotorL(pwm - delta_pwm)
				motores.setMotorR(pwm + delta_pwm)

				proporcional_pasado = proporcional
				
				print "Delta: ", delta_pwm, ". Posicion: ", position

				# El tiempo actual 'timepast' pasa a ser el tiempo previo 'timenow'
				timepast = timenow

			
	def detect(self, corner = False, blockL = False, blockR = False, dist = 0, line = False):	# CHECK
		"""Procedimiento que se mantiene activo mientras no se detecte el objeto indicado por argumento.

		ARGUMENTOS:
		corner: True si se quiere detectar una esquina, False en caso contrario (Valor predetermiando: False)
		blockL: True si se quiere detectar un bloque a la izquierda, False en caso contrario (Valor predetermiando: False)
		blockR: True si se quiere detectar un bloque a la derecha, False en caso contrario (Valor predetermiando: False)
		dist: Distancia maxima a la que se considera que se detecto un bloque
		line: True si se quiere detectar una linea, False en caso contrario (Valor predetermiando: False)
		"""
		
		ser.reset_input_buffer()
		
		if corner:
			sensors = arduino.getQTR()
			negro = 1500 # Valor minimo que se considera que los sensores estan leyendo negro
			
			while sensors[0] < negro and sensors[7] < negro:
				sensors = arduino.getQTR()

		elif blockL:
			block = arduino.getUltraL()

			while block > dist:
				block = arduino.getUltraL()

		elif blockR:
			block = arduino.getUltraR()

			while block > dist:
				block = arduino.getUltraR()

		elif line:
			sensors = arduino.getQTR()
			negro = 1500 # Valor minimo que se considera que los sensores estan leyendo negro
			
			while all((sensor<negro) for sensor in sensors):
				sensors = arduino.getQTR()

			
	def turn(self, clockwise, angle):			# CHECK
		"""Hace girar al robot.
		
		ARGUMENTOS:
		pwm: Velocidad.
		clockwise: True indica sentido anti-horario; False indica sentido horario"""

		neg = ((-1) ** int(clockwise))
		motores.setMotorL((-neg) * 20)  
		motores.setMotorR(neg * 20)
		t = self.angToTime(angle, clockwise)
		sleep(t)
		motores.stop()




	################################# FUNCIONES COMPUESTAS ###############################
	def movStrUntObj(self, foward, Slow = True, BlockL = False, BlockR = False, dist = 0, Line = False):		# CHECK
		"""Mueve el robot en linea recta hasta detectar el objeto indicado.

		ARGUMENTOS:
		foward: True indica que el robot se mueve hacia adelante; False indica el caso contrario
		Corner: True si se quiere detectar una esquina, False en caso contrario (Valor predetermiando: False)
		BlockL: True si se quiere detectar un bloque a la izquierda, False en caso contrario (Valor predetermiando: False)
		BlockR: True si se quiere detectar un bloque a la derecha, False en caso contrario (Valor predetermiando: False)
		dist: Distancia maxima a la que se considera que se detecto un bloque
		Line: True si se quiere detectar una linea, False en caso contrario (Valor predetermiando: False)
		"""

		p1 = Process(target = self.moveStraight, args = (foward, 0, Slow)) 
		p1.start()
		p2 = Process(target = self.detect, args = (False, BlockL, BlockR, dist, Line,))
		p2.start()
		

		while p2.is_alive():
			pass
	
		p1.terminate()
		motores.stop()


	def fllwLineUntObj(self, pwm, Corner = False, BlockL = False, BlockR = False, dist = 0, Bifur = False, Time = False, tm = 0):		# CHECK
		"""El robot sigue la linea negra hasta detectar el objeto indicado.

		ARGUMENTOS:
		pwm: Velocidad.
		Corner: True si se quiere detectar una esquina, False en caso contrario (Valor predetermiando: False)
		BlockL: True si se quiere detectar un bloque a la izquierda, False en caso contrario (Valor predetermiando: False)
		BlockR: True si se quiere detectar un bloque a la derecha, False en caso contrario (Valor predetermiando: False)
		dist: Distancia maxima a la que se considera que se detecto un bloque
		Line: True si se quiere detectar una linea, False en caso contrario (Valor predetermiando: False)
		"""		
		
		# velocidad 30 
		kp = 0.010
		ki = 0
		kd = 0.0005
		proporcional_pasado = 0
		integral = 0

		# Variables de tiempo
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar
						# de nuevo los calculos de ajuste del PID.*
		epsilon = 0
		timepast = 0
		lapse = 0
		
		ser.reset_input_buffer()
		
		ard = arduino.getAll()
		sensors = ard[0:8]
		dL = ard[9]
		dR = ard[11]

		# Expresiones booleanas para el while
		p = any(sensor > 1000 for sensor in sensors) and Corner
		q = (sensors[0] < 1000 and sensors[7] < 1000) and Bifur
		r = (dL > dist or dL == 0) and BlockL
		s = (dR > dist or dR == 0) and BlockR
		t = (lapse < tm) and Time
		
				
		while p or q or r or s or t:
			
			# Mide el tiempo actual
			timenow = time()	
			# Calcula diferencia entre el tiempo actual y el pasado
			dt = timenow - timepast

			if dt >= epsilon:

				position = float(ard[8])

				# Calculamos el valor proporcional, integarl y derivativo del PID basados en la poscicon del sensor, centro 3500
				proporcional = position - 3500
				integral = integral + proporcional_pasado;  
				derivativo = (proporcional - proporcional_pasado)

				# Se acota el valor del valor integral del PID
				if integral > 1000: 
					integral = 1000
				if integral < -1000: 
					integral = -1000
					
				# Calculamos la funcion PID
				delta_pwm = ( proporcional * kp ) + ( derivativo * kd ) + ( integral * ki )
						
				# Evaluamos casos para que el robot no se atrase
				motores.setMotorL(pwm - delta_pwm)
				motores.setMotorR(pwm + delta_pwm)

				proporcional_pasado = proporcional
				
				# El tiempo actual 'timepast' pasa a ser el tiempo previo 'timenow'
				timepast = timenow
				
			ard = arduino.getAll()
			sensors = ard[0:8]
			dL = ard[9]
			dR = ard[11]
			
			if dt < 2:
				lapse += dt
						
			# Expresiones booleanas para el while
			p = any(sensor > 1000 for sensor in sensors) and Corner
			q = (sensors[0] < 1000 and sensors[7] < 1000) and Bifur
			r = (dL > dist or dL == 0) and BlockL
			s = (dR > dist or dR == 0) and BlockR
			t = (lapse < tm) and Time
				
		motores.stop()
				
			
	def fllwLineUntCorner(self, pwm):				# CHECK
		"""Hace al robot seguir una linea negra hasta detectar una esquina (todos los sensores blancos)
		
		ARGUMENTOS:
		pwm: Velocidad.
		"""
		
		kp = 0.015
		ki = 0
		kd = 0.0005
		proporcional_pasado = 0
		integral = 0

		# Variables de tiempo
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar
						# de nuevo los calculos de ajuste del PID.*
		epsilon = 0
		timepast = 0 
		
		n = 15
		
		ser.reset_input_buffer()
		sensors = arduino.getQTR()
		

		while any(sensor>1000 for sensor in sensors):
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el tiempo actual y el pasado
			dt = timenow - timepast

			if dt >= epsilon:

				position = float(arduino.getAverageQTR())

				# Calculamos el valor proporcional, integarl y derivativo del PID basados en la poscicon del sensor, centro 3500
				proporcional = position - 3500
				integral = integral + proporcional_pasado;  
				derivativo = (proporcional - proporcional_pasado)

				# Se acota el valor del valor integral del PID
				if integral > 1000: 
					integral = 1000
				if integral < -1000: 
					integral = -1000
					
				# Calculamos la funcion PID
				delta_pwm = ( proporcional * kp ) + ( derivativo * kd ) + ( integral * ki )
						
				# Evaluamos casos para que el robot no se atrase
				motores.setMotorL(pwm - delta_pwm)
				motores.setMotorR(pwm + delta_pwm)

				proporcional_pasado = proporcional
				
				# El tiempo actual 'timepast' pasa a ser el tiempo previo 'timenow'
				timepast = timenow
				
			sensors = arduino.getQTR()
		motores.stop()


	def turnUntLine(self, clockwise):				# CHECK		
		"""El robot gira hasta conseguir una linea.

		ARGUMENTOS:
		pwm: Velocidad.
		Clockwise: True indica sentido horario; False indica sentido anti-horario.
		"""
		p2 = Process(target = self.detect, args = (False, False, False, 0, True))
		p2.start()
		
		p1 = Process(target = self.turn, args = (clockwise, 360)) 
		p1.start()

		while p2.is_alive():
			pass
	
		p1.terminate()
		motores.stop()


	def align(self, pwm, foward):					# CHECK
		"""El robot se mueve en lineaa recta y se alinea con una linea negra.

		ARGUMENTOS:
		pwm: Velocidad del giro.
		"""
		neg = -(-1)**int(foward)
		n = 20
		while n:
			sensors = arduino.getQTR()
			n -= 1
		sensors = arduino.getQTR()
		
		negro = False

		# El robot se mueve en linea recta hasta conseguir una linea
		while not negro:
			negro = any(sensor > 1000 for sensor in sensors)
			sensors = arduino.getQTR()
			motores.run(neg*pwm)
		
		# Verificamos los sensores QTR
		sensors = arduino.getQTR()
		motores.stop()

		# Verificamos si el primer sensor QTR que detecto la linea fue el izquierdo (True) o el derecho (False)
		izq = sum(sensors[i] for i in range(3)) < sum(sensors[i] for i in range(5, 8))
		

		# El valor de la variable es 1 si foward es True, o -1 en caso contrario
		negro = 1500
		
		if foward:
			kp = 1.25
		else:
			kp = 2.5


		if izq:
			while sensors[0] < negro:
				motores.setMotorR(neg*15)
				motores.setMotorL(-neg*15 / kp)
				sensors = arduino.getQTR()
		else:
			while sensors[7] < negro:
				motores.setMotorL(neg*15)
				motores.setMotorR(-neg*15 / kp)
				sensors = arduino.getQTR()
				
		if not all(sensor > 100 for sensor in sensors):
			motores.run(-10)
			sleep(1)
			self.foward(pwm, True)
				
		motores.stop()




	################################ TESTING #####################################
	def TmoveStraight(self, foward, dist = 0):
		"""Mueve al robot en linea recta.
		
		ARGUMENTOS:
		pwm: Velocidad.
		foward: True si el robot avanza hacia adelante; False en caso contrario.
		dist: Indica la distancia que se va a mover el robot; 0 indica distancia indeterminada (Valor predeterminado: 0)."""
		
		# Parametros del PID
		kp = 0.05		# Constante Proporcional
		ki = 0	  	# Constante Integral
		kd = 0 	# Constante Diferencial
		errL = 0	# Error del encoder izquierdo
		
		# Variables de tiempo
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar
						# de nuevo los calculos de ajuste del PID.*
		epsilon = 0.001
		timepast = 0 
		
		# Inizialicacion de las variables del PID
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		# Posicion previa de las ruedas 
		prePosL = arduino.getEncoderL()
		prePosR = arduino.getEncoderR()
		# Posicion actual de las ruedas
		nowPosR = prePosR
		nowPosL = prePosL
		
		# Distancia recorrida
		distance = 0

		# Radio de las ruedas del robot
		r = 2.8 # cm
		
		pi = 3.141592654 #Valor de pi

		# El valor de la variable es 1 si foward es True, o -1 en caso contrario
		neg = -((-1) ** int(foward))
		
		n = 10
		
		while n:
			# Posicion previa de las ruedas 
			prePosL = arduino.getEncoderL()
			prePosR = arduino.getEncoderR()
			# Posicion actual de las ruedas
			nowPosR = prePosR
			nowPosL = prePosL
			
			n = n-1
			
		# Para tener un contro del promedio de ticks que giraron ambas ruedas
		promL = 0
		promR = 0
		n = 0
			
		
		while not bool(dist) or distance < dist:
						
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el tiempo actual y el pasado
			dt = timenow - timepast
			
			if dt >= epsilon:
				# Se lee el valor de los encoders
				nowPosL = arduino.getEncoderL() + errL
				nowPosR = arduino.getEncoderR()
				
				
				# Diferencia entre los posiciones previas y actuales de los motores
				difL = self.difGrados(nowPosL, prePosL, foward)
				difR = self.difGrados(nowPosR, prePosR, foward)
				
				
				# Error/diferencia entre la salida de los encoders de esta iteracion
				dif = difR - difL
				
				# Suma de los errores(dif) anteriores
				psum = psum + dif

				# Aplicacion de la funcion de PID
				delta = dif * kp + (dif - pdif / dt) * kd + ki * psum

				
				# Se modifica la salida de los motores				
				motores.setMotorL(40.9 + delta)  	
				motores.setMotorR(99.55 - delta)


				# La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
				pdif = dif

				# Se calcula la velocidad angular de cada rueda del robot y con esos valores se calcula la velocidad lineal del robot#
				wr = pi * difR / (dt * 180) # Convertimos los ticks/s en rad/segfundos 3600 ticks = 2pi
				wl = pi * difL / (dt * 180)
				v = r * (wr + wl) / 2 						# Velocidad lineal del robot en cm/s
				
				# Integramos para hallar la distancia recorrida
				distance = v * dt + distance
				
				# Actualizamos la posicion de las ruedas
				prePosR = nowPosR
				prePosL = nowPosL
			
				# El tiempo previo 'timepast' pasa a ser el tiempo actual 'timenow'
				timepast = timenow
				
				promL += difL
				promR += difR
				n += 1

				print "Ticks Izquierdos: ", difL, ". Ticks Derechos: ", difR, ". Delta: ", delta , ". dist: ", distance

		print "Ticks Promedio Izquierdos: ", promL/n, ". Ticks Promedio Derechos: ", promR/n
		motores.stop()
	
	
	def TprobEncd(self, pwm):
		timepast = time()
		dt = 0

		# Posicion previa de las ruedas 
		prePosL = arduino.getEncoderL()
		prePosR = arduino.getEncoderR()
		# Posicion actual de las ruedas
		nowPosR = prePosR
		nowPosL = prePosL

		n = 10

		while n:
			# Posicion previa de las ruedas 
			prePosL = arduino.getEncoderL()
			prePosR = arduino.getEncoderR()
			# Posicion actual de las ruedas
			nowPosR = prePosR
			nowPosL = prePosL
			
			n = n-1
			
		# Para tener un contro del promedio de ticks que giraron ambas ruedas
		promL = 0
		promR = 0
		n = 0

		motores.setMotorL(pwm)  	
		motores.setMotorR(pwm)

		while True:
			# Se lee el valor de los encoders
			nowPosL = arduino.getEncoderL()
			nowPosR = arduino.getEncoderR()
			
			# Diferencia entre los posiciones previas y actuales de los motores
			difL = self.difGrados(nowPosL, prePosL, True)
			difR = self.difGrados(nowPosR, prePosR, True)
			
			prePosR = nowPosR
			prePosL = nowPosL
			
			promL += difL
			promR += difR
			n += 1
			
			timenow  = time()	
			dt = timenow - timepast
			
			print "Ticks Izquierdos: ", difL, ". Ticks Derechos: ", difR, "Pos Izquierda: ", nowPosL, ". Pos DerechA: ", nowPosR, "\n"
			
		print "Ticks Promedio Izquierdos: ", promL/n, ". Ticks Promedio Derechos: ", promR/n
			
		robot.stop()
	
	
	
	
