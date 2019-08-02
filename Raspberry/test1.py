from MotorControl import Motors
from arduino_reader import Arduino

motores = Motors(17,27) # Pines GPIO17 y GPIO27
arduino = Arduino()

motores.stop()

while True:
	print arduino.getQTR()
