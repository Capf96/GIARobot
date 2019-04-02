from MotorControl import Motor
import pigpio
from time import sleep, time
from electroIman import Magnet

from Sensores import QTR, Sensor
import serial

ser = serial.Serial('/dev/ttyUSB0', 57600)

# Encoder provides a resolution of 3600 ticks per revolution.
# Un tick es la division minima del giro del moto que proporciona el encoder
ser.flushInput()



class Robot(object):
	
	def __init__(self, mLeft, mRight, magnet, qtr):
		self.mLeft  = mLeft  # Motor Izquierdo
		self.mRight = mRight # Motor Derecho
		self.magnet = magnet # Iman
		self.qtr    = qtr 	 # Sensor de linea
			
		
	
	
	def forward(self,distance, pwm): 

		#Funcion    :El robot se mueve en línea recta hacia adelante a una distancia y velocidad determinadas.

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
		
		#Inizialicación de las variables del PID#
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		pTicksR = 0 # Ticks derechos previos
		pTicksL = 0	# Ticks izquierdos previos
		
		nTicksR = 0 # Ticks derechos actuales
		nTicksL = 0 # Ticks izquierdos actuales

		
		#Radio de las ruedas del robot#
		r = 4.08 #cm
		

		#Inicialización de a distancia recorrida
		distanceR = 0
		
		pi = 3.141592654 #Valor de pi
		
		while distanceR < distance :
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

				#Aplicación de la función de PID#
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
				print(str(distanceR))
				
			
			
		#Se detiene el robot al terminar la ejecución de la función.
		self.mLeft.stop()
		self.mRight.stop()	
	
	def backward(self,distance, pwm): 
		#Funcion    :El robot se mueve en línea recta hacia atrás a una distancia y velocidad determinadas.

		#Entradas#
		#"distance" :Distancia aproximada que se desea que el robot recorra. Tirne que estar en centimetros.
		#"pwm"      :Potencia de los motores 
		
		#Aplicamos la misma función forward solo que la distancia y la potencia de los motores son negativas#
		self.forward(-distance, -pwm)
		
	def turnLeft(self, degrees, pwm): 
		#Funcion    :El robot gira en sentido antihorario "degrees" grados con una potencia 'pwm'.

		#Entradas#
		#"degrees"  :Grados aproximada que se desea que el robot gire. Tirne que estar en grados.
		#"pwm"      :Potencia de los motores 
		
		#Aplicamos la misma función forward solo que la distancia y la potencia de los motores son negativas#
		

		#Parametros del PID#
		kp = 0.1 # Constante Proporcional
		ki = 0	  # Constante Integral
		kd = 0	  # Constante Diferencial
		
		

		#Función de correción(experimental) para los grados que se tienen que recorrer
		correction = lambda x: 1.86586641*(x**0.32895066) - 3
		degrees = degrees - correction(degrees)
		
		#Parametros del robot#
		r = 4.08 #cm Radio de las ruedas
		L = 26.5 #cm Distancia entre las ruedas
		
		#Parametros del tiempo#
		dt = 0.07 #s
		
		
		#Inizialicación de las variables del PID#
		pdif = 0    # Diferencia/error previo
		psum = 0    # Suma de las diferencias/errores previas
		
		pTicksR = 0 # Ticks derechos previos
		pTicksL = 0	# Ticks izquierdos previos
		
		nTicksR = 0 # Ticks derechso actuales
		nTicksL = 0 # Ticks izquierdos actuales
		
		#Inicialización de los grados girados#
		degreesT = 0
		
		while degreesT < degrees :
			#Se lee el valor de los encoders#
			nTicksR = self.mRight.getTicks()
			nTicksL = self.mLeft.getTicks()
			
			#Error/diferencia entre la salida de los encoders de esta iteracion#
			dif = abs(nTicksR) - abs(nTicksL)
			
			#Suma de los errores(dif) anteriores#
			psum = psum + dif

			#Aplicación de la función de PID#
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
			
			print(str(degreesT))
			
			sleep(dt)
		
		self.mLeft.stop()
		self.mRight.stop()
	
	def turnRight(self, degrees, pwm):  

		#Funcion    :El robot gira n sentido horario "degrees" grados con una potencia 'pwm'.

		#Entradas#
		#"degrees"  :Grados aproximada que se desea que el robot gire. Tirne que estar en grados.
		#"pwm"      :Potencia de los motores 
				
		#Aplicamos la misma función forward solo que la distancia y la potencia de los motores son negativas#
		
		self.turnLeft(-degrees, -pwm)


	def rightPrll (self, ticks, pwm): 
		
		#Funcion    :# El robot se mueve hacia la derecha 'n ticks' sin cambiar su orientacion con velocidad 'pwm'

		#Entradas#
		#"ticks"    :Ticks (1 Tick = 0.1 grados) aproximados que se desea que el robot gire en cada una de sus ruedas durante el movimiento.
		#"pwm"      :Potencia de los motores 

		#Parametros de la función#
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
			
			print(str(counter))
		
		self.mLeft.stop()
		self.mRight.stop()
		
	
	def leftPrll (self, ticks, pwm): 
		#Funcion    :# El robot se mueve hacia la izquierda 'n ticks' sin cambiar su orientacion con velocidad 'pwm'

		#Entradas#
		#"ticks"    :Ticks (1 Tick = 0.1 grados) aproximados que se desea que el robot gire en cada una de sus ruedas durante el movimiento.
		#"pwm"      :Potencia de los motores 

		#Parametros de la función#
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
			print(str(counter))
		self.mLeft.stop()
		self.mRight.stop()
	
	
	def getToLineF(self, pwm): #va recto hasta una linea y se para en loa linea
		
		white = True
		
		negro = 1000
		
		timenow = time()
		timepast = time()
		
		### Variables  del pid 
		kp = 0.047 #Buen valor 0.75
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
			
					
					
			if (timenow - timepast >= epsilon) and white == True:
				
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
		
				
	
	
	
		


	
