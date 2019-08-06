from MotorControl import Motors
from arduinoReader import Arduino

motores = Motors(17,27) # Pines GPIO17 y GPIO27
arduino = Arduino()

class Robot(object):
	
	def followLine(self, pwm):
		"""Utiliza un PID con calculando el error con respecto a la linea 
		usando el QTR para seguir la linea ."""
		
		kp = float( 0.014 ) # 50/3500 
		ki = 0.0
		kd = 0.0
		proporcional_pasado = 0
		integral = 0

		while True:
			position = float( arduino.getAverageQTR() )

			# Calculamos el valor proporcional, integarl y derivativo del PID basados en la poscicon del sensor, centro 3500
			proporcional = position - 3500
			integral = integral + proporcional_pasado;  
			derivativo = (proporcional - proporcional_pasado)

			#Se acota el valor del valor integral del PID
			if integral>1000: 
				integral=1000
			if integral<-1000: 
				integral=-1000
				
			#Calculamos la funcion PID
			delta_pwm = ( proporcional * kp ) + ( derivativo * kd ) + ( integral * ki )
					
			#Evaluamos casos para que el robot no se atrase
			if (delta_pwm <= 0):
				motores.setMotorL(pwm - delta_pwm)
				motores.setMotorR(pwm)
			if (delta_pwm >0):
				motores.setMotorL(pwm)
				motores.setMotorR(pwm + delta_pwm)

			proporcional_pasado = proporcional
		
		
	
