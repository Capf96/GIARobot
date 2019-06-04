import time
from subprocess import call 
import RPi.GPIO as GPIO
import RobotControl

GPIO.setmode(GPIO.BCM) #Queremos usar la numeracion de la placa
 
 
# Definimos los dos pines para el Trigger y los Echos
Trig = 21
EchoI = 25
EchoD = 24


# Configuramos los pines anteriores
GPIO.setup(Trig, GPIO.OUT)
GPIO.setup(EchoI, GPIO.IN)
GPIO.setup(EchoD, GPIO.IN)
GPIO.output(Trig, False)


def sensorI():
	# Detecta un obstaculo en el sensor Izquierdo
	
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
		while GPIO.input(EchoI) == 0:
			start = time.time()

		while GPIO.input(EchoI) == 1:
			end = time.time()
	 
		#La duracion del pulso del pin Echo sera la diferencia entre el tiempo de inicio y el final
		duracion = end-start
		duracion = duracion*10**6
		medida += duracion/58 
		i += 1
	 
		time.sleep(0.1)
		
	medida = medida/5
	
	return medida
	
def sensorD():
	# Detecta un obstaculo en el sensor Derecho
	
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
		while GPIO.input(EchoD) == 0:
			start = time.time()

		while GPIO.input(EchoD) == 1:
			end = time.time()
	 
		# La duracion del pulso del pin Echo sera la diferencia entre el tiempo de inicio y el final
		duracion = end-start
		duracion = duracion*10**6
		medida += duracion/58 
		i += 1
	 
		time.sleep(0.1)
		
	medida = medida/5
	
	return medida




def alinearBloque():
	# Nos alineamos con el bloque
	
	# Leemos el valor de los ultrasonidos laterales
	sensorIzq = sensorI()
	sensorDer = sensorD()

	# Mientras el sensor izquierdo lea bien pero el derecho no, se movera a la izquierda
	while (13.5 < sensorIzq) and (sensorIzq < 14.5) and ( (sensorDer < 13.5) or (senserDer > 14.5) ):
		leftPrll(500, 50)
		sensorIzq = sensorI()
		sensorDer = sensorD()
		
	# Mientras el sensor derecho lea bien pero el izquierdo no, se movera a la derecha
	while (13.5 < sensorDer) and (sensorDer < 14.5) and ( (sensorIzq < 13.5) or (senserIzq > 14.5) ):
		rightPrll(500, 50)
		sensorIzq = sensorI()
		sensorDer = sensorD()
		
		
		
		
		
		
		
