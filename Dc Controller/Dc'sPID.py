from time import sleep
import pigpio
from RPi import GPIO
from RotaryEncoder import Encoder


pi = pigpio.pi()
EncoderRight = Encoder(pi, 6, 12)
EncoderLeft = Encoder(pi, 19, 16)


pi.set_mode(18, pigpio.OUTPUT)
pi.set_mode(17, pigpio.OUTPUT)

pi.set_mode(23, pigpio.OUTPUT)
pi.set_mode(22, pigpio.OUTPUT)


def pid(speed, movement, tiempo_final):

	dt = 0.05
	kp = 0.4
	kd = 0

	initial_left = EncoderLeft.getTicks()
	initial_right = EncoderRight.getTicks()

	if movement == "F":
		speed_left = speed
		speed_right = speed
		kp = kp
		kd = kd
	elif movement == "R":
		speed_left = speed
		speed_right = -speed
		kp = kp
		kd = kd
	elif movement == "L":
		speed_left = -speed
		speed_right = speed
		kp = kp
		kd = kd
	elif movement == "B":
		speed_left = -speed
		speed_right = -speed
		kp = kp
		kd = kd
	else:
		return

	tiempo = 0
	actual_right = 0
	actual_left = 0
	while tiempo < tiempo_final:
		tiempo = tiempo + dt

		actual_left = EncoderLeft.getTicks() - actual_left  # Deberiamos leer los encoder aqui... no se como#
		actual_right = EncoderRight.getTicks() - actual_right  # Deberiamos leer los encoder aqui... no se como#
		print(str(actual_right))
		print(str(actual_left))

		proportional_right = actual_right - initial_right
		proportional_left = actual_left - initial_left
		
		# proportion = actual_right - actual_left

		derivative_right = proportional_right/dt
		derivative_left = proportional_left/dt

		output_right = speed_right + derivative_right * kd - proportional_right * kp
		# + (derivative_left * kd + proportional_left  * kp)
		output_left = speed_left + derivative_left * kd + proportional_left * kp
		# -  (derivative_right * kd + proportional_right  * kp)

		if output_right > 255:
			output_right = 255
		elif output_right < -255:
			output_right = -255

		if output_left > 255:
			output_left = 255
		elif output_left < -255:
			output_left = -255

		print("Right "+str(output_right))
		print("Left "+str(output_left))

		if output_right <= 0:
			if output_left <= 0:
				pi.set_PWM_dutycycle(18, 0)
				pi.set_PWM_dutycycle(17, -output_right)

				pi.set_PWM_dutycycle(23, -output_left)
				pi.set_PWM_dutycycle(22, 0)

			else:
				pi.set_PWM_dutycycle(17, -output_right)
				pi.set_PWM_dutycycle(18, 0)

				pi.set_PWM_dutycycle(23, 0)
				pi.set_PWM_dutycycle(22, output_left)

		elif output_right > 0:
			if output_left <= 0:
				pi.set_PWM_dutycycle(18, output_right)
				pi.set_PWM_dutycycle(17, 0)

				pi.set_PWM_dutycycle(23, -output_left)
				pi.set_PWM_dutycycle(22, 0)

			else:
				pi.set_PWM_dutycycle(18, output_right)
				pi.set_PWM_dutycycle(17, 0)

				pi.set_PWM_dutycycle(23, 0)
				pi.set_PWM_dutycycle(22, output_left)

		initial_left = actual_left
		initial_right = actual_right
		sleep(dt)

	pi.set_PWM_dutycycle(18, 0)
	pi.set_PWM_dutycycle(17, 0)
	pi.set_PWM_dutycycle(23, 0)
	pi.set_PWM_dutycycle(22, 0)
	pi.stop()


pid(190, "F", 1)
