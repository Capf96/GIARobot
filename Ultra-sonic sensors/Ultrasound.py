import time
from subprocess import call 
import RPi.GPIO as GPIO

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
		while GPIO.input(EchoD) == 0:
			start = time.time()

		while GPIO.input(EchoD) == 1:
			end = time.time()
	 
		#La duracion del pulso del pin Echo sera la diferencia entre el tiempo de inicio y el final
		duracion = end-start
		duracion = duracion*10**6
		medida += duracion/58 
		i += 1
	 
		time.sleep(0.1)
		
	medida = medida/5
	
	return medida
	
while True:
	print("Sensor derecho: " + str(sensorD()) + ". Sensor izquierdo: " + str(sensorI()))