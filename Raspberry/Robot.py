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
from math import pi

import pigpio
import serial

# Componentes del Robot
motores = Motors(26,20) # Pines GPIO17 y GPIO27
grua = Grua(19, 16, 18, 17)
magnet = Magnet(23,22)
suicheD = Switch(9,10) #Suiche izquierdo	
suicheI = Switch(24,25)	#Suiche Derecho

ser = serial.Serial('/dev/ttyACM0', 57600)
arduino = Arduino()

class Robot(object):
	
	################################ FUNCIONES BASICAS #################################
	def timeLapse(self, lapse, dummy=0):
		"""Funcion que termina cuando pase el tiempo indicado. 
		
		ARGUMENTOS:
		lapse: Lapso de tiempo.
		"""
		
		past = time()
		now = past
		
		while now - past < lapse:
			now = time()
			sleep(0.05)
	
	
	def difAngle(self, ang1, ang2):				# CHECk
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
	
			
	def moveStraight(self, pwm, dummy=0):  	
		"""Hace que el robot se mueva hacia adelante en linea recta.
		
		ARGUMENTOS:
		pwm: Potencia de los motores.
		"""   
	
		# Parametros del PID
		kp = 0.4		    # Constante Proporcional
		ki = 0	  	    # Constante Integral
		kd = 0 #0.00375       # Constante Diferencial

		
		# Variables de tiempo
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar		
		epsilon = 0.01  #0.001 muy bajo buen valor 0.06
		timepast = time() #No se puede inicializar en 0 si no calcula mla la distncia 
		
		
		# Inizialicacion de las variables del PID
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
	

		# Radio de las ruedas del robot
		r = 2.8 # cm
		
		
		alpha_o = arduino.gyro()

		
		while True:							
			
			# Mide el tiempo actual
			timenow  = time()	
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


	def stop(self):								# CHECK
		"""Detiene y apaga todos los componentes del robot."""
		
		magnet.off()
		motores.stop()
	
				
	def followLine(self, pwm, dummy=0):			# CHECK
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

					
	def detect(self, corner = False, blockL = False, blockR = False, blockC= False, dist = 0, no = False, line = False):	# CHECK
		"""Procedimiento que se mantiene activo mientras no se detecte el objeto indicado por argumento.

		ARGUMENTOS:
		corner: True si se quiere detectar una esquina, False en caso contrario (Valor predetermiando: False)
		blockL: True si se quiere detectar un bloque a la izquierda, False en caso contrario (Valor predetermiando: False)
		blockR: True si se quiere detectar un bloque a la derecha, False en caso contrario (Valor predetermiando: False)
		blockC: True si se quiere detectar un bloque central, False en caso contrario (Valor predetermiando: False)
		dist: Distancia maxima a la que se considera que se detecto un bloque
		no: Indica si se busca detectar un bloque, o dejar de detectar un bloque
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
			
			while block > dist and not no:
				block = arduino.getUltraL()
				
			while block < dist and no:
				block = arduino.getUltraL()


		elif blockR:
			block = arduino.getUltraR()

			while block > dist and not no:
				block = arduino.getUltraR()
			
			while block < dist and no:
				block = arduino.getUltraR()
				
				
		elif blockC:
			while (suicheD.getVal() == 0) and (suicheI.getVal() == 0):
				pass	


		elif line:
			sensors = arduino.getQTR()
			negro = 1800 # Valor minimo que se considera que los sensores estan leyendo negro
			
			while all((sensor<negro) for sensor in sensors):
				sensors = arduino.getQTR()

			
	def turn(self, clockwise, dummy=0):			# CHECK
		"""Hace girar al robot.
		
		ARGUMENTOS:
		clockwise: True indica sentido horario; False indica sentido anti-horario.
		"""

		neg = ((-1) ** int(clockwise))
		motores.setMotorL((-neg) * 30)  
		motores.setMotorR(neg * 30)


	def getPoles(self):							# CHECK    
		
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


	def adjtNearAngle(self, pwm, ang_ob, rec):      
		"""Hace que el robot se ajuste a un angulo cercano dado.
		
		ARGUMENTOS
		pwm: Potencia de los motores.
		ang_obj: Es mi angulo objetivo.
		"""
		recursion = rec + 1
		PWM = pwm
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
					if t=="cklock" and (pwm-n-1 > 20):
						n = n+1
						pwm = pwm-n
					
					t ="ccklock"
				
				else:
					motores.setMotorL(-pwm )  	
					motores.setMotorR(+pwm)
					if t=="ccklock" and (pwm-n-1 > 20):
						n = n+1
						pwm = pwm-n
					t =" cklock"
					
					
				
				# El tiempo previo 'timepast' pasa a ser el tiempo actual 'timenow'
				timepast = timenow
		motores.stop()		
		sleep(0.1)
		alpha_o =  arduino.gyro()
		if not(alpha_o > ang_ob + 3 or alpha_o < ang_ob - 3 ) and recursion < 15:
			self.adjtNearAngle(PWM, ang_ob, recursion)
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
		
		while cond and not magnet:
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
					
		if not magnet:
			# Despues de que nos acercamos lo suficiente a la posicion, usamos la funcion para acercarse a un angulo cercano.
			self.adjtNearAngle(pwm, targetDir, 0)
			
		timepast = time()
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar				
		epsilon  = 0.005   # 0.001 muy bajo buen valor 0.06
		
		
		if magnet:
			motores.setMotorL(neg*pwm )  	
			motores.setMotorR(-neg*pwm)	
			sleep(3)
			motores.stop()
			
		while (pos > targetDir + 1) or (pos < targetDir - 1) and magnet:
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
		
		
		print("LLEGUE!") #BORRAR	
		
		
	
		
	################################# FUNCIONES COMPUESTAS ###############################
	def movStrUntTime(self, pwm, lapse):		# CHECK
		"""Mueve el robot en linea recta hasta que pase un lapso de tiempo indicado.

		ARGUMENTOS:
		pwm: Potencia de los motores.
		lapse: Lapso de tiempo.
		"""

		p1 = Process(target = self.moveStraight, args = (pwm, 0))
		p1.start()
		p2 = Process(target = self.timeLapse, args = (lapse, 0))
		p2.start()
		
		while p2.is_alive():
			pass
	
		p1.terminate()
		motores.stop()


	def moveStrUntLine(self, pwm):  				# CHECK   
	
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


	def movStrUntBlock(self, pwm):
		
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
		
		while (suicheD.getVal() == 0) and (suicheI.getVal() == 0):
									
			
			# Mide el tiempo actual
			timenow  = time()	
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


	def fllwLineUntObj(self, pwm, Corner = False, BlockL = False, BlockR = False, dist = 0, No = False, Bifur = False, Time = False, tm = 0):		# CHECK
		"""El robot sigue la linea negra hasta detectar el objeto indicado.

		ARGUMENTOS:
		pwm: Velocidad.
		Corner: True si se quiere detectar una esquina, False en caso contrario (Valor predetermiando: False)
		BlockL: True si se quiere detectar un bloque a la izquierda, False en caso contrario (Valor predetermiando: False)
		BlockR: True si se quiere detectar un bloque a la derecha, False en caso contrario (Valor predetermiando: False)
		dist: Distancia maxima a la que se considera que se detecto un bloque
		No: Indica si se busca detectar un bloque, o dejar de detectar un bloque
		Bifur: True si se quiere detectar una bifurcacion (Valor predeterminado: False)
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
		r = (dL > dist or dL == 0) and BlockL and not No
		s = (dR > dist or dR == 0) and BlockR and not No
		t = (lapse < tm) and Time
		u = (dL < dist) and BlockL and No
		v = (dR < dist) and BlockR and No
		
				
		while p or q or r or s or t or u or v:
			
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
						
			p = any(sensor > 1000 for sensor in sensors) and Corner
			q = (sensors[0] < 1000 and sensors[7] < 1000) and Bifur
			r = (dL > dist or dL == 0) and BlockL and not No
			s = (dR > dist or dR == 0) and BlockR and not No
			t = (lapse < tm) and Time
			u = (dL < dist) and BlockL and No
			v = (dR < dist) and BlockR and No
				
		motores.stop()


	def turnUntLine(self, clockwise):				# CHECK		
		"""El robot gira hasta conseguir una linea.

		ARGUMENTOS:
		Clockwise: True indica sentido horario; False indica sentido anti-horario.
		"""
		p2 = Process(target = self.detect, args = (False, False, False, False, 0, False, True))
		p2.start()
		
		p1 = Process(target = self.turn, args = (clockwise, 0)) 
		p1.start()

		while p2.is_alive():
			pass
	
		p1.terminate()
		motores.stop()


	def align(self, pwm):					# CHECK
		"""El robot se mueve en linea recta hacia adelante y se alinea con una linea negra.

		ARGUMENTOS:
		pwm: Velocidad del giro.
		"""
	
		self.moveStrUntLine(pwm)
		
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



	################################ TESTING #####################################
	def TprobEncd(self, pwm):
		timepast = time()
		dt = 0
		epsilon = 0.001
		# Posicion previa de las ruedas 
		prePosL = arduino.getEncoderL()
		prePosR = arduino.getEncoderR()
		# Posicion actual de las ruedas
		nowPosR = prePosR
		nowPosL = prePosL
			
		# Para tener un contro del promedio de ticks que giraron ambas ruedas
		promL = 0
		promR = 0
		n = 0

		motores.setMotorL(pwm)  	
		motores.setMotorR(pwm)

		while True:
			dt = time() - timepast
			if  dt > epsilon:
				# Se lee el valor de los encoders
				ard = arduino.getAll()
				nowPosL = ard[12]
				nowPosR = ard[13]
				
				# Diferencia entre los posiciones previas y actuales de los motores
				difL = self.difGrados(nowPosL, prePosL, True)
				difR = self.difGrados(nowPosR, prePosR, False)
				
				#print( difL, " ", difR )
				
				prePosR = nowPosR
				prePosL = nowPosL
				
				promL += difL
				promR += difR
				n += 1
				timepast = time() 
			
		robot.stop()
	

	def Talign(self, pwm, foward):
		"""El robot se mueve en lineaa recta y se alinea con una linea negra.

		ARGUMENTOS:
		pwm: Velocidad del giro.
		foward: Variable booleana que indica si va hacia adelante.
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
	

	def TmoveStraight(self, pwm, dist = 0):     
		"""Mueve al robot en linea recta.
		
		ARGUMENTOS:
		pwm: Velocidad.
		foward: True si el robot avanza hacia adelante; False en caso contrario.
		dist: Indica la distancia que se va a mover el robot en mm; 0 indica distancia indeterminada (Valor predeterminado: 0)."""
		
		# Definir direccion
		forward = pwm>0
		
		# Convertir dist a cm
		dist *=2.5
		
		# Parametros del PID
		if forward:
			kp = 2		# Constante Proporcional
			ki = 0.01	  	# Constante Integral
			kd = 0 	# Constante Diferencial
		else:
			kp = 0.001		# Constante Proporcional
			ki = 0	  	# Constante Integral
			kd = 0 	# Constante Diferencial
		# Variables de tiempo
		dt = 0        	# Diferencial de tiempo. Es el tiempo que espera el sistema para aplicar
						# de nuevo los calculos de ajuste del PID.*
		epsilon = 0.06 #0.001 muy bajo buen valor 0.01
		timepast = 0 
		
		# Inizialicacion de las variables del PID
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		n = 10
		while n:
			arduino.getAll()
			n -= 1
		
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
		
		# Para tener un contro del promedio de ticks que giraron ambas ruedas
		promL = 0
		promR = 0	

		ser.reset_input_buffer()
				
		while not dist or distance < dist:
									
			# Mide el tiempo actual
			timenow  = time()	
			# Calcula diferencia entre el tiempo actual y el pasado
			dt = timenow - timepast
			
			if dt >= epsilon:		
			
				# Se lee el valor de los encoders
				ard = arduino.getAll()
				nowPosL = ard[13]
				nowPosR = ard[12]
				
				# Diferencia entre los posiciones previas y actuales de los motores
				if forward:
					difL = self.difGrados(nowPosL, prePosL, False)
					difR = self.difGrados(nowPosR, prePosR, True)
				else:
					difL = self.difGrados(nowPosL, prePosL, True)
					difR = self.difGrados(nowPosR, prePosR, False)
				
				if difL > 300 or difR > 300:
					difL = difR
				
				# Error/diferencia entre la salida de los encoders de esta iteracion
				dif = difR - difL
								
				# Suma de los errores(dif) anteriores
				psum = psum + dif

				# Aplicacion de la funcion de PID
				delta = dif * kp + (dif - pdif / dt) * kd + ki * psum
				
				# Se modifica la salida de los motores				
				motores.setMotorL(pwm + delta)  	# *((-1)**(not forward))
				motores.setMotorR(pwm)

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
				
				# PRINT PAR HACER TEST
				print( "difR: " +  str(difR) + ". difL: " + str(difL) + ". Dif: " + str(dif) + ". promL: " + str(promL) + ". promR: " + str(promR))
				

		motores.stop()
	
	
	def T2moveStraight(self, foward, dist = 0, slow = True):	
		"""Mueve al robot en linea recta.
		
		ARGUMENTOS:
		pwm: Velocidad.
		foward: True si el robot avanza hacia adelante; False en caso contrario.
		dist: Indica la distancia que se va a mover el robot; 0 indica distancia indeterminada (Valor predeterminado: 0).
		slow: True indica que se movera lentamente, False indica rapidamente (Valor predetermiando: True). """
		
		# Definimos la velocidad de los motores.
		if slow:
			velLF, velRF, velLB, velRB = 19, 22, -29.05, -24
		else:
			velLF, velRF, velLB, velRB = 60, 45, -40.31, -38.55
			
		if foward:
			motores.setMotorL(velLF)  	# 	40.00	|	20.00
			motores.setMotorR(velRF)	# 	45.00	| 	21.00
			
			t = self.distToTime(dist - 0.5, foward)
			
		else:
			motores.setMotorL(velLB)	#	-40.31	|	-25.05
			motores.setMotorR(velRB)	#	-38.55	|	-20
			
			t = self.distToTime(dist, foward)

			
		while not bool(dist):
			pass
			
		sleep(t)

		motores.stop()
	
	
	def turnMagnet(self,pwm, ang = 90, clockwise = True, R = 16):
		ser.reset_input_buffer()	
		r = 2.8 
		
		
		if clockwise == True:
			nEc = 11 #Encoder Izquierdo
			motores.setMotorL(pwm)
		else:
			nEc = 12  #Encoder Derecho
			motores.setMotorR(pwm)
		
		print(arduino.getAll()[nEc])
		
		alphao = arduino.getAll()[nEc]
		alpha = R/r * ang + alphao
	
		n = int(alpha/360)
		alpha = alpha%360
		
		
		print("Alpha: ", alpha)
		
		angR = 0;
		epsilon = 0.1
		tpast = time()
		while n:
			dt = time() - tpast 
			if dt > epsilon:
				ser.reset_input_buffer()
				alphar = arduino.getAll()[nEc]
				print("Dentro de n. AlphaR:  ", alphar, "Alpha0: ", alphao)
				if alphar < alphao and abs(alphar - alphao) > 100:
					n=n-1
				alphao = alphar
				tpast = time()  
		
		while (alphao < alpha):
			dt = time() - tpast 
			if dt > epsilon:
				ser.reset_input_buffer()
				alphar = arduino.getAll()[nEc]
				print("Fuera de n. AlphaR:  ", alphar, "Alpha0: ", alphao)
				if alphar < alphao and abs(alphar - alphao) > 100:
					break
				alphao = alphar
				tpast = time() 
				
			
			
	
	
if __name__ == "__main__":
	# NO BORRAR NI COMENTAR
	robot = Robot(); state = Estado()
	##########
	
	magnet.off()
	while True:
		sleep(0.5)
		ser.reset_input_buffer()
		print("Encoder D: ", arduino.getAll()[12])
	

	
	
		
	
	# NO BORRAR NI COMENTAR
	robot.stop()
	##########

