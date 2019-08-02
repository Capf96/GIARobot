from MotorControl import Motors

motores = Motors(17,27) # Pines GPIO17 y GPIO27

motores.stop()

while True:
	v = float(input())
	motores.setMotorL(v)
