"""Aqui se encuentra la clase Robot que contiene los principales procedimientos para controlar los movimientos del robot."""

from MotorControl import Motors
from ArduinoReader import Arduino
from StepperMotor import Grua
from Electroiman import Magnet
from Encoder import Encoder
from Estado import Estado
from Switch import Switch

from pdb import set_trace
from time import sleep, time
from multiprocessing import Process


import pigpio
import serial


motores = Motors(26,20) # Pines GPIO17 y GPIO27
arduino = Arduino()
grua = Grua(19, 16, 6, 12)
magnet = Magnet(23)
ser = serial.Serial('/dev/ttyACM0', 57600)
encoderR = Encoder(21)
encoderL = Encoder(5) 
suicheD = Switch(9,10) #Suiche izquierdo	
suicheI = Switch(24,25)	#Suiche Derecho
pi = 3.141592654 #Valor de pi


class PID(object):
	
	################################ FUNCIONES BASICAS #################################
	def difAngle(self, ang1, ang2):
		"""Obtiene la distancia entre dos angulos.
		
		ARGUMENTOS:
		ang1: Primer Angulo.
		ang2: Segundo Angulo.
		"""
		
		# Si ambos tienen el mismo signo, entonces la distancia entre ambos angulos sera su diferencia.
		if (ang1 > 0 and ang2 > 0) or (ang1 < 0 and ang2 < 0):
			return abs(ang1 - ang2)
			
		# Si ambos tienen distinto signo, entonces la distancia entre ambos angulos sera la menor entre la suma de 
		#sus diferencias con 0  y  360 menos la distancia anterior.
		else:
			return min(abs(ang1) + abs(ang2), 360 - abs(ang1) - abs(ang2))
			 
		
	
	def angDifClockWise(self, ang_o, ang_f):		# CHECK

		if ang_f <0:
			r = ang_o - ang_f
			if r > 0:
				return r
			else:
				return 360 - r 
		elif ang_f > 0:
			r = ang_o - ang_f
			if r > 0:
				return  r 
			else:
				return 360 + r 
			

		
	def stop(self):								# CHECK
		"""Detiene y apaga todos los componentes del robot."""
		
		magnet.off()
		# grua.nivel(0)
		# grua.setStep(0, 0, 0, 0)
		motores.stop()

		
		
	def followLine(self, pwm, dummy = False):	# CHECK
		"""Hace al robot seguir una linea negra.
		
		ARGUMENTOS:
		pwm: Velocidad.
		dummy: Variable sin ningun uso que permite el uso de procesos."""
		
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
				

				# El tiempo actual 'timepast' pasa a ser el tiempo previo 'timenow'
				timepast = timenow


			
	def detect(self, corner = False, blockL = False, blockR = False, blockC= False, dist = 0, line = False):	# CHECK
		"""Procedimiento que se mantiene activo mientras no se detecte el objeto indicado por argumento.

		ARGUMENTOS:
		corner: True si se quiere detectar una esquina, False en caso contrario (Valor predetermiando: False)
		blockL: True si se quiere detectar un bloque a la izquierda, False en caso contrario (Valor predetermiando: False)
		blockR: True si se quiere detectar un bloque a la derecha, False en caso contrario (Valor predetermiando: False)
		blockC: True si se quiere detectar un bloque central, False en caso contrario (Valor predetermiando: False)
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
				
		elif blockC:
			block = arduino.getUltraC()

			while block > dist and block < 35:
				block = arduino.getUltraC()
				
				if block <= dist or block >= 35:
					for i in range(3):
						block += arduino.getUltraC() / 3
				
				

		elif line:
			sensors = arduino.getQTR()
			negro = 1800 # Valor minimo que se considera que los sensores estan leyendo negro
			
			while all((sensor<negro) for sensor in sensors):
				sensors = arduino.getQTR()

		
			
	def turn(self, clockwise, angle):			# CHECK
		"""Hace girar al robot.
		
		ARGUMENTOS:
		clockwise: True indica sentido horario; False indica sentido anti-horario.
		angle: Angulo aproximado que se desea girar.
		"""

		neg = ((-1) ** int(clockwise))
		motores.setMotorL((-neg) * 20)  
		motores.setMotorR(neg * 20)
		t = self.angToTime(angle, clockwise)
		sleep(t)
		motores.stop()



	################################# FUNCIONES COMPUESTAS ###############################
	def movStrUntObj(self, foward, Slow = True, BlockL = False, BlockR = False, BlockC = False, dist = 0, Line = False, pwm=50):		# CHECK
		"""Mueve el robot en linea recta hasta detectar el objeto indicado.

		ARGUMENTOS:
		foward: True indica que el robot se mueve hacia adelante; False indica el caso contrario
		Corner: True si se quiere detectar una esquina, False en caso contrario (Valor predetermiando: False)
		BlockL: True si se quiere detectar un bloque a la izquierda, False en caso contrario (Valor predetermiando: False)
		BlockR: True si se quiere detectar un bloque a la derecha, False en caso contrario (Valor predetermiando: False)
		blockC: True si se quiere detectar un bloque central, False en caso contrario (Valor predetermiando: False)
		dist: Distancia maxima a la que se considera que se detecto un bloque
		Line: True si se quiere detectar una linea, False en caso contrario (Valor predetermiando: False)
		"""

		p1 = Process(target = self.moveStraight, args = (pwm, dist)) 
		p1.start()
		p2 = Process(target = self.detect, args = (False, BlockL, BlockR, BlockC, dist, Line))
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
		Bifur: True si se quiere detectar una bifurcacion (Valor predeterminado: False)
		Line: True si se quiere detectar una linea, False en caso contrario (Valor predetermiando: False)
		Time: True indica que se seguira la linea durante cierto tiempo (Valor predeterminado: False)
		tm: Tiempo que se desea esperar mientras sigue la linea.
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
		Clockwise: True indica sentido horario; False indica sentido anti-horario.
		"""
		p2 = Process(target = self.detect, args = (False, False, False, False, 0, True))
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
		foward: Variable booleana que indica si va hacia adelante.
		"""
		p1 = Process(target = self.moveStraight, args = (foward, 0, True)) 
		p1.start()
		p2 = Process(target = self.detect, args = (False, False, False, False, 0, True))
		p2.start()
		
		while p2.is_alive():
			pass
	
		p1.terminate()
		motores.stop()
		
		sensors = arduino.getQTR()
		neg = -((-1)**int(foward))

		# Verificamos si el primer sensor QTR que detecto la linea fue el izquierdo (True) o el derecho (False)
		izq = sum(sensors[i] for i in range(3)) < sum(sensors[i] for i in range(5, 8))
		
		# El valor de la variable es 1 si foward es True, o -1 en caso contrario
		negro = 1500
		
		if foward:
			kp = 1.25
		else:
			kp = 3.5


		if izq:
			while sensors[0] < negro:
				motores.setMotorR(neg*pwm)
				motores.setMotorL(-neg*pwm / kp)
				sensors = arduino.getQTR()
		else:
			while sensors[7] < negro:
				motores.setMotorL(neg*pwm)
				motores.setMotorR(-neg*pwm / kp)
				sensors = arduino.getQTR()
				
		if not all(sensor > 100 for sensor in sensors):
			motores.run(-pwm)
			sleep(1)
			self.align(pwm, True)
				
		motores.stop()


	
	def adjtNearAngle(self, pwm, ang_ob):      
		"""Hace que el robot se ajuste a un angulo cercano dado.
		
		ARGUMENTOS
		pwm: Potencia de los motores.
		ang_obj: Es mi angulo objetivo.
		"""
		
		print("Me estoy acercando...")	#BORRAR
		
		
		alpha_o = arduino.gyro()
		timepast = time()
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar				
		epsilon  = 0.01   # 0.001 muy bajo buen valor 0.06
		t = None

		
		while alpha_o > ang_ob + 3 or alpha_o < ang_ob - 3 :
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el tiempo actual y el pasado
			dt = timenow - timepast
			n = 1
			if dt >= epsilon:
				
				alpha_o =  arduino.gyro()
					
		
				
				d = ang_ob - alpha_o
				if d > 0:
					motores.setMotorL(+pwm )  	
					motores.setMotorR(-pwm)
					if t=="cklock":
						n = n+1
						pwm = pwm-n
					
					t ="ccklock"
				
				else:
					motores.setMotorL(-pwm )  	
					motores.setMotorR(+pwm)
					if t=="ccklock":
						n = n+1
						pwm = pwm-n
					t =" cklock"
					
					
				
				# El tiempo previo 'timepast' pasa a ser el tiempo actual 'timenow'
				timepast = timenow
		motores.stop()		
		sleep(0.1)
		alpha_o =  arduino.gyro()
		if	not(alpha_o > ang_ob + 3 or alpha_o < ang_ob - 3 ):
			self.adjtNearAngle(pwm, ang_ob)
		motores.stop()
	
	
		
	def adjtAngle(self, pwm, clockwise, state, magnet):      
		"""Hace que el robot gire hacia uno de los 4 puntos cardinales vecinos.
		
		ARGUMENTOS:
		pwm: Potencia de los motores.
		clockwise: Sentido del giro.
		state: Estado del robot.
		magnet: Indica si el iman esta prendido o no
		"""
		
		# Limpiamos el buffer
		ser.reset_input_buffer()
		
		# Guardamos la distancia entre la posicion actual y los 4 puntos cardinales.
		# Verificamos cual es la menor de las 4 distancias anteriores, la menor sera la posicion actual
		
		neg = -(-1)**int(clockwise)
		direc = arduino.gyro()


		if magnet:
			difDir = self.difAngle(direc, state.direcsI[0])	# Esta variable indica cual es el punto cardinal mas cercana.
		else:
			difDir = self.difAngle(direc, state.direcs[0])	# Esta variable indica cual es el punto cardinal mas cercana.
			
			
		ind = 0		# Indice que indica el punto cardinal mas cercano.
		for i in range(1, 4):
			
			if magnet:
				angle = self.difAngle(direc, state.direcsI[i])
			else:
				angle = self.difAngle(direc, state.direcs[i])
				
				
			if angle < difDir:
				difDir = angle
				ind = i
			
		# BORRAR
		if ind == 0:
			print("ESTOY EN EL NORTE")
		elif ind == 1:
			print("ESTOY EN EL ESTE")
		elif ind == 2:
			print("ESTOY EN EL SUR")
		elif ind == 3:
			print("ESTOY EN EL OESTE")
		########
		
		if magnet:
			targetDir = state.direcsI[(ind + neg)%4]
		else:
			targetDir = state.direcs[(ind + neg)%4]
			
			
		pos = arduino.gyro()
		
		
		# Definimos nuesta condicion para mantener al robot dentro del while
		if (targetDir < -170):
			dif = self.difAngle(-180, targetDir)
			cond = (pos > targetDir + 10) and (pos < 170 + dif) 
		elif (targetDir > 170):
			dif = self.difAngle(180, targetDir)
			cond = (pos > -170 - dif) and (pos < targetDir - 10) 
		else:
			cond = (pos > targetDir + 10) or (pos < targetDir - 10)
			
			
		timepast = time()
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar				
		epsilon  = 0.01   # 0.001 muy bajo buen valor 0.06
		
		while cond:
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el tiempo actual y el pasado.
			dt = timenow - timepast
			
			if dt >= epsilon:
				pos =  arduino.gyro()
				
				motores.setMotorL(neg*pwm )  	
				motores.setMotorR(-neg*pwm)				
				
				# El tiempo previo 'timepast' pasa a ser el tiempo actual 'timenow'.
				timepast = timenow
				
				
				if (targetDir < -170):
					cond = (pos > targetDir + 10) and (pos < 170 + dif) 
				elif (targetDir > 170):
					cond = (pos > -170 - dif) and (pos < targetDir - 10) 
				else:
					cond = (pos > targetDir + 10) or (pos < targetDir - 10)
					
					
				
		# Despues de que nos acercamos lo suficiente a la posicion, usamos la funcion para acercarse a un angulo cercano.
		self.adjtNearAngle(pwm, targetDir)
		
		
		print("LLEGUE!") #BORRAR	
		
		
							
	def rotateClockwisePID(self, pwm, degF = 0):      
		"""
		Hace girar al robot cierto numero de grados respecto a su eje central
		
		ARGUMENTOS:
		pwm  : Velocidad.
		degF : Indica la los grados que se desean que el robot gire

		"""
		if degF > 360:
			return "chupalo lorenzo"
		# Parametros del PID

		kp = 0	    # Constante Proporcional
		ki = 0	  	# Constante Integral
		kd = 0 	        # Constante Diferencial

		
		# Variables de tiempo
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar
						
		epsilon  = 0.01   # 0.001 muy bajo buen valor 0.06
		timepast = time() # No se puede inicializar en 0 si no calcula mla la distncia 
		
		# Inizialicacion de las variables del PID
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		
		

		# Radio de las ruedas del robot
		r = 2.8 # cm
		
		#Longitud de la rueda al centro de girop dl robot
		rr =  7.9 # cm
		alpha_o = arduino.gyro()
		"""
		print(alpha_o)
		
		alpha_f = alpha_o - degF
		
		if alpha_f < -180:
			alpha_f += 360
		"""
		d = 0
		
		while d < degF or d > degF  + 10  :
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el tiempo actual y el pasado
			dt = timenow - timepast
			
			if dt >= epsilon:

				alpha_f =  arduino.gyro()		
				
				motores.setMotorL(-pwm )  	
				motores.setMotorR(+pwm)
				
				d = self.angDifClockWise(alpha_o,alpha_f)
				print(str(d)+"  "+str(alpha_f)+ "  "+str(alpha_o))
				# El tiempo previo 'timepast' pasa a ser el tiempo actual 'timenow'
				timepast = timenow
				
			
		motores.stop()
		

		
	def forwardPIDGyro(self, pwm):     
	
		# Parametros del PID

		kp = 0.1		    # Constante Proporcional
		ki = 0	  	    # Constante Integral
		kd = 0 #0.00375       # Constante Diferencial

		
		# Variables de tiempo
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar
						
		epsilon = 0.01  #0.001 muy bajo buen valor 0.06
		timepast = time() #No se puede inicializar en 0 si no calcula mla la distncia 
		
		# Inizialicacion de las variables del PID
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
	
		
		# Distancia recorrida
		distance = 0

		# Radio de las ruedas del robot
		r = 2.8 # cm
		
		alpha_o = arduino.gyro()
		
		sensors = arduino.getQTR()
		negro = 1800 
		
		while all((sensor<negro) for sensor in sensors):
									
			
			# Mide el tiempo actual
			timenow  = time()	
			sensors = arduino.getQTR()
			alpha_f = arduino.gyro()
			# Calcula diferencia entre el tiempo actual y el pasado
			dt = timenow - timepast
			
				
			
			if dt >= epsilon:		
				
				dif = alpha_f - alpha_o
					
				# Suma de los errores(dif) anteriores
				psum = psum + dif

				# Aplicacion de la funcion de PID
				delta = dif * kp + (dif - pdif / dt) * kd + ki * psum
				
				# Se modifica la salida de los motores				
				motores.setMotorL(pwm + delta)  	
				motores.setMotorR(pwm)
			
				# La Diferencia Actual 'dif' pasa a ser la diferencia previa 'pdif' #
				pdif = dif

				# El tiempo previo 'timepast' pasa a ser el tiempo actual 'timenow'
				timepast = time()

				alpha_o  = alpha_f
				
				
		motores.stop()
		
		
		
	def getPoles(self):     
		
		print("iniciando Calibracion")
		
		print("Coloque el robot mirando hacia el norte")
		print("N")
		input("[Presione enter para continuar]")
		k = 1
		while k :
			ser.reset_input_buffer()
			print("Sampleando...")
			for i in range(0, 10):
				print(arduino.gyro())
			k = input("[Presione enter para continuar o [1] y enter para volver a samplear]")
			if k == None:
				k = 0
		norte = arduino.gyro()

		print("Coloque el robot mirando hacia el este")
		print("E")
		input("[Presione enter para continuar]")
		k = 1
		while k :
			ser.reset_input_buffer()
			print("Sampleando...")
			for i in range(0, 10):
				print(arduino.gyro())
			k = input("[Presione enter para continuar o [1] y enter para volver a samplear]")
			if k == None:
				k = 0
		este = arduino.gyro()

		print("Coloque el robot mirando hacia el sur")
		print("S")
		input("[Presione enter para continuar]")
		k = 1
		while k :
			ser.reset_input_buffer()
			print("Sampleando...")
			for i in range(0, 10):
				print(arduino.gyro())
			k = input("[Presione enter para continuar o [1] y enter para volver a samplear]")
			if k == None:
				k = 0
		sur = arduino.gyro()

		print("Coloque el robot mirando hacia el oeste")
		print("O")
		input("[Presione enter para continuar]")
		k = 1
		while k :
			ser.reset_input_buffer()
			print("Sampleando...")
			for i in range(0, 10):
				print(arduino.gyro())
			k = input("[Presione enter para continuar o [1] y enter para volver a samplear]")
			if k == None:
				k = 0
		oeste = arduino.gyro()
		
		
		print([norte, este, sur, oeste])
		
		
		print("El electroiman sera encendido a continuacion coloquele un bloque de metal")
		input("[Presione enter para continuar]")
		magnet.on()
		print("Coloque el robot mirando hacia el norte")
		print("N")
		input("[Presione enter para continuar]")
		k = 1
		while k :
			ser.reset_input_buffer()
			print("Sampleando...")
			for i in range(0, 10):
				print(arduino.gyro())
			k = input("[Presione enter para continuar o [1] y enter para volver a samplear]")
			if k == None:
				k = 0
		norte_m = arduino.gyro()

		print("Coloque el robot mirando hacia el este")
		print("E")
		input("[Presione enter para continuar]")
		k = 1
		while k :
			ser.reset_input_buffer()
			print("Sampleando...")
			for i in range(0, 10):
				print(arduino.gyro())
			k = input("[Presione enter para continuar o [1] y enter para volver a samplear]")
			if k == None:
				k = 0
		este_m = arduino.gyro()

		print("Coloque el robot mirando hacia el sur")
		print("S")
		input("[Presione enter para continuar]")
		k = 1
		while k :
			ser.reset_input_buffer()
			print("Sampleando...")
			for i in range(0, 10):
				print(arduino.gyro())
			k = input("[Presione enter para continuar o [1] y enter para volver a samplear]")
			if k == None:
				k = 0
		sur_m = arduino.gyro()

		print("Coloque el robot mirando hacia el oeste")
		print("O")
		input("[Presione enter para continuar]")
		k = 1
		while k :
			ser.reset_input_buffer()
			print("Sampleando...")
			for i in range(0, 10):
				print(arduino.gyro())
			k = input("[Presione enter para continuar o [1] y enter para volver a samplear]")
			if k == None:
				k = 0
		oeste_m = arduino.gyro()
		
		
		print("El electroimn sera apado a continuacion retirele el bloque de metal")
		input("[Presione enter para continuar]")
		magnet.off()
		
		return [norte, este, sur, oeste], [norte_m, este_m, sur_m, oeste_m ] 	
			
	
	
if __name__ == "__main__":
	# NO BORRAR NI COMENTAR
	robot = Robot(); state = Estado()
	##########
	
	state.direcs, state.direcsI = robot.getPoles()
	
	while True:
		
		
		clock = bool(int(input("Girara a la derecha? [1 | 0]: ")))
		magnet = bool(int(input("Encendera el electroiman? [1 | 0]: ")))
		
		if magnet:
			magnet.on()
			
		robot.adjtAngle(0, clock, state, magnet)
		
		magnet.off()
	
		
	
	# NO BORRAR NI COMENTAR
	robot.stop()
	##########
	
	


