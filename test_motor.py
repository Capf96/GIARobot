import RPi.GPIO as GPIO
import pigpio

pi = pigpio.pi()
# Servo Parallax PWM:
SERVOr = 17  # Deben ser pines GPIO
SERVOl = 27
pi.set_mode(SERVOr, pigpio.OUTPUT)
pi.set_mode(SERVOl, pigpio.OUTPUT)
pi.set_PWM_frequency(SERVOl, 50)
pi.set_PWM_frequency(SERVOr, 50)

while True:
	v = float(input())
	pi.set_servo_pulsewidth(SERVOl, v)
	pi.set_servo_pulsewidth(SERVOr, v)

