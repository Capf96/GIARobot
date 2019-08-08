from MotorControl import Motors
from ArduinoReader import Arduino
from StepperMotor import Grua
from time import sleep, time
from multiprocessing import Process
import pigpio
import serial

motores = Motors(17,27) # Pines GPIO17 y GPIO27
arduino = Arduino()
grua = Grua(24, 23, 22, 27)

class Robot(object):
	
	################################ FUNCIONES BASICAS #################################
	def difGrados(self, now, prev, foward):
		"""Obtiene la diferencia entre una posicion pasada y una actual dada en grados.
		
		ARGUMENTOS:
		prev: Posicion previa.
		now: Posicion actual. 
		foward: True si la rueda gira hacia adelante; False en caso contrario."""
		
		dif = ((-1) ** int(foward)) * (prev - now)
		
		if dif >= 0:
			return dif
		else:
			return dif + 360
			
	
	def stop(self):
		"""Detiene y apaga todos los componentes del robot."""
		
		motores.stop()
		# iman.stop()
		# grua.nivel(0)
		grua.setStep(0, 0, 0, 0)
		
		
	def moveStraight(self, pwm, foward, dist = 0):
		"""Mueve al robot en linea recta.
		
		ARGUMENTOS:
		pwm: Velocidad.
		foward: True si el robot avanza hacia adelante; False en caso contrario.
		dist: Indica la distancia que se va a mover el robot; 0 indica distancia indeterminada (Valor predeterminado: 0)."""
		
		# Parametros del PID
		kp = 5 		# Constante Proporcional
		ki = 0	  	# Constante Integral
		kd = 0	  	# Constante Diferencial
		
		# Variables de tiempo
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar
						# de nuevo los calculos de ajuste del PID.*
		epsilon = 0.01
		timepast = 0 
		
		# Inizialicacion de las variables del PID
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		# Posicion previa de las ruedas 
		prePosL = arduino.getEncoderL()
		prePosR = arduino.getEncoderR()
		# Posicion actual de las ruedas
		nowPosR = prePorR
		nowPosL = prePosL
		
		# Distancia recorrida
		distance = 0

		# Radio de las ruedas del robot
		r = 4.08 # cm
		
		pi = 3.141592654 #Valor de pi

		# El valor de la variable es 1 si foward es True, o -1 en caso contrario
		neg = -((-1) ** int(foward))
		
		while not bool(dist) or distance < dist:
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el tiempo actual y el pasado
			dt = timenow - timepast
			
			if dt >= epsilon:
				# Se lee el valor de los encoders
				nowPosL = arduino.getEncoderL()
				nowPosR = arduino.getEncoderR()
				
				# Diferencia entre los posiciones previas y actuales de los motores
				difL = difGrados(nowPosL, prePosL, foward)
				difR = difGrados(nowPosR, prePosR, foward)
				
				# Error/diferencia entre la salida de los encoders de esta iteracion
				dif = difR - difL
				
				# Suma de los errores(dif) anteriores
				psum = psum + dif

				# Aplicacion de la funcion de PID
				delta = dif * kp + (dif - pdif / dt) * kd + ki * psum

				# Se modifica la salida de los motores
				motores.setMotorL(neg * (pwm + delta))  	# Se busca igualar la velocidad de uno de los motores con 
													# la del otro por lo tanto las modificaciones solo se realizan a ese motor
				motores.setMotorR(neg * pwm) 

				# La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
				pdif = dif

				# Se calcula la velocidad angular de cada rueda del robot y con esos valores se calcula la velocidad lineal del robot#
				wr = pi * difR / (dt * 1800) # Convertimos los ticks/s en rad/segfundos 3600 ticks = 2pi
				wl = pi * difL / (dt * 1800)
				v = r * (wr + wl) / 2 						# Velocidad lineal del robot en cm/s
				
				# Integramos para hallar la distancia recorrida
				distance = v * dt + distance
				
				# Actualizamos la posicion de las ruedas
				prePosR = nowPosR
				prePosL = nowPosL
			
				# El tiempo previo 'timepast' pasa a ser el tiempo actual 'timenow'
				timepast = timenow

		motores.stop()
		
		
	def followLine(self, pwm):
		"""Hace al robot seguir una linea negra.
		
		ARGUMENTOS:
		pwm: Velocidad."""
		
		kp = 0.014 	# 50/3500 
		ki = 0.0
		kd = 0.0
		proporcional_pasado = 0
		integral = 0

		# Variables de tiempo
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar
						# de nuevo los calculos de ajuste del PID.*
		epsilon = 0.01
		timepast = 0 

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
				if (delta_pwm <= 0):
					motores.setMotorL(pwm - delta_pwm)
					motores.setMotorR(pwm)
				if (delta_pwm > 0):
					motores.setMotorL(pwm)
					motores.setMotorR(pwm + delta_pwm)

				proporcional_pasado = proporcional

				# El tiempo actual 'timepast' pasa a ser el tiempo previo 'timenow'
				timepast = timenow

			
	def detect(self, corner = False, blockL = False, blockR = False, line = False):
		"""Procedimiento que se mantiene activo mientras no se detecte el objeto indicado por argumento.

		ARGUMENTOS:
		corner: True si se quiere detectar una esquina, False en caso contrario (Valor predetermiando: False)
		blockL: True si se quiere detectar un bloque a la izquierda, False en caso contrario (Valor predetermiando: False)
		blockR: True si se quiere detectar un bloque a la derecha, False en caso contrario (Valor predetermiando: False)
		line: True si se quiere detectar una linea, False en caso contrario (Valor predetermiando: False)
		"""
		
		if corner:
			sensors = arduino.getQTR()
			negro = 1000 # Valor minimo que se considera que los sensores estan leyendo negro
			
			while sensors[0] < negro and sensors[7] < negro:
				sensors = arduino.getQTR()

		elif blockL:
			block = arduino.getUltraL()
			distMax = 23	# Distancia maxima a la que se considera que se detecto un bloque

			while block > distMax:
				block = arduino.getUltraL()

		elif blockR:
			block = arduino.getUltraR()
			distMax = 23	# Distancia maxima a la que se considera que se detecto un bloque

			while block > distMax:
				block = arduino.getUltraR()

		elif line:
			sensors = arduino.getQTR()
			negro = 1000 # Valor minimo que se considera que los sensores estan leyendo negro

			while all((sensor<negro) for sensor in sensors):
				sensors = arduino.getQTR()

			
	def turn(self, pwm, ctClockwise):
		"""Hace girar al robot.
		
		ARGUMENTOS:
		pwm: Velocidad.
		ctClockwise: True indica sentido anti-horario; False indica sentido horario"""

		# Parametros del PID
		kp = 0.1 	# Constante Proporcional
		ki = 0	  	# Constante Integral
		kd = 0	  	# Constante Diferencial
		
		# Parametros del robot 
		r = 4.08 	# cm Radio de las ruedas
		L = 26.5 	# cm Distancia entre las ruedas
		
		
		# Variables de tiempo
		dt = 0      	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar
						# de nuevo los calculos de ajuste del PID
		epsilon = 0.01
		timepast = 0 

		
		#Inizialicacion de las variables del PID#
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		# Posicion previa de las ruedas 
		prePosL = arduino.getEncoderL()
		prePosR = arduino.getEncoderR()
		# Posicion actual de las ruedas
		nowPosR = prePorR
		nowPosL = prePosL
		
		# Inicializacion de los grados girados
		degreesT = 0
		
		# Variable igual a -1 si ctClockwise es True; o 1 en caso contrario
		neg = ((-1) ** int(ctClockwise))
		
		while True:
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el tiempo actual y el pasado
			dt = timenow - timepast
			
			if dt >= epsilon:
				# Se lee el valor de los encoders
				nowPosL = arduino.getEncoderL()
				nowPosR = arduino.getEncoderR()
				
				# Diferencia entre los posiciones previas y actuales de los motores
				difL = difGrados(nowPosL, prePosL, not ctClockwise)
				difR = difGrados(nowPosR, prePosR, ctClockwise)
				
				# Error/diferencia entre la salida de los encoders de esta iteracion
				dif = difR - difL
				
				# Suma de los errores(dif) anteriores#
				psum = psum + dif

				# Aplicacion de la funcion de PID#
				delta = dif * kp + (dif - pdif / dt) *kd + ki * psum

				# Se modifica la salida de los motores#
				motores.setMotorL(neg * (pwm + delta))  
				motores.setMotorR(-(neg * pwm)) 

				# La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
				pdif = dif

				# Se calcula la velocidad angular de cada rueda del robot y con esos valores se calcula la velocidad angular del robot#
				wr = difR / (dt * 10) # convertimos los ticks/s en grados/segfundos 3600 ticks = 360 grados
				wl = difL / (dt * 10)
				w = r * (wr - wl) / L # grados/s
				
				# Integramos para hallar los grados recorridos#
				degreesT = w * dt + degreesT
				
				# Actualizamos la posicion de las ruedas
				prePosR = nowPosR
				prePosL = nowPosL
			
				# El tiempo previo 'timepast' pasa a ser el tiempo actual 'timenow'
				timepast = timenow


	################################# FUNCIONES COMPUESTAS ###############################
	def movStrUntObj(self, pwm, foward, Corner = False, BlockL = False, BlockR = False, Line = False):
		"""Mueve el robot en linea recta hasta detectar el objeto indicado.

		ARGUMENTOS:
		pwm: Velocidad.
		foward: True indica que el robot se mueve hacia adelante; False indica el caso contrario
		Corner: True si se quiere detectar una esquina, False en caso contrario (Valor predetermiando: False)
		BlockL: True si se quiere detectar un bloque a la izquierda, False en caso contrario (Valor predetermiando: False)
		BlockR: True si se quiere detectar un bloque a la derecha, False en caso contrario (Valor predetermiando: False)
		Line: True si se quiere detectar una linea, False en caso contrario (Valor predetermiando: False)
		"""

		p1 = Process(target = self.moveStraight, args = (pwm, foward)) 
		p1.start()
		p2 = Process(target = self.detect, args = (corner = Corner, blockL = BlockL, blockR = BlockR, line = Line))
		p2.start()

		while p2.is_alive():
			pass
	
		p1.terminate()
		motores.stop()


	def fllwLineUntObj(self, pwm, Corner = False, BlockL = False, BlockR = False, Line = False):
		"""El robot sigue la linea negra hasta detectar el objeto indicado.

		ARGUMENTOS:
		pwm: Velocidad.
		Corner: True si se quiere detectar una esquina, False en caso contrario (Valor predetermiando: False)
		BlockL: True si se quiere detectar un bloque a la izquierda, False en caso contrario (Valor predetermiando: False)
		BlockR: True si se quiere detectar un bloque a la derecha, False en caso contrario (Valor predetermiando: False)
		Line: True si se quiere detectar una linea, False en caso contrario (Valor predetermiando: False)
		"""

		p1 = Process(target = self.followLine, args = (pwm)) 
		p1.start()
		p2 = Process(target = self.detect, args = (corner = Corner, blockL = BlockL, blockR = BlockR, line = Line))
		p2.start()

		while p2.is_alive():
			pass
	
		p1.terminate()
		motores.stop()


	def turnUntLine(self, pwm, ctClockwise):
		"""El robot gira hasta conseguir una linea.

		ARGUMENTOS:
		pwm: Velocidad.
		ctClockwise: True indica sentido anti-horario; False indica sentido horario.
		"""

		p1 = Process(target = self.turn, args = (pwm, ctClockwise)) 
		p1.start()
		p2 = Process(target = self.detect, args = (line = True))
		p2.start()

		while p2.is_alive():
			pass
	
		p1.terminate()
		motores.stop()


	def align(self, pwm, foward):
		"""El robot se alinea con una linea negra.

		ARGUMENTOS:
		pwm: Velocidad.
		foward: True indica que el robot iba avanzando antes de llegar a la linea; False indica que iba retrocediendo.
		"""

		# El robot se mueve en linea recta hasta conseguir una linea
		movStrUntObj(pwm, foward, Line = True)

		# Verificamos los sensores QTR
		sensors = arduino.getQTR()

		# Verificamos si el primer sensor QTR que detecto la linea fue el izquierdo (True) o el derecho (False)
		izq = sensors[0] > sensors[7]

		# El valor de la variable es 1 si foward es True, o -1 en caso contrario
		negro = 1000

		# El valor de la variable es 1 si foward es True, o -1 en caso contrario
		neg = -((-1) ** int(foward))

		if izq:
			while sensors[7] < 1000:
				motores.setMotorR(neg * pwm)
				sensors = arduino.getQTR()
				sleep(0.01)
		else:
			while sensors[0] < 1000:
				motores.setMotorL(neg * pwm)
				sensors = arduino.getQTR()
				sleep(0.01)

