import time
import pigpio
from RPi import GPIO
from RotaryEncoder import Encoder



pi = pigpio.pi()

pi.set_mode(18, pigpio.OUTPUT)
pi.set_mode(17, pigpio.OUTPUT)

pi.set_mode(23, pigpio.OUTPUT)
pi.set_mode(22, pigpio.OUTPUT)

EncoderRight= Encoder(pi, 6, 12)
EncoderLeft= Encoder(pi, 19, 16)

Left = EncoderLeft.getTicks() 
Right = EncoderRight.getTicks()
print Left, Right

pi.set_PWM_dutycycle(18, 255)
pi.set_PWM_dutycycle(17, 0)

time.sleep(3)

Left = EncoderLeft.getTicks() 
Right = EncoderRight.getTicks()
print Left, Right

pi.set_PWM_dutycycle(23, 0)
pi.set_PWM_dutycycle(22, 255)

time.sleep(3)

Left = EncoderLeft.getTicks() 
Right = EncoderRight.getTicks()
print Left, Right

time.sleep(3)

pi.set_PWM_dutycycle(18, 0)
pi.set_PWM_dutycycle(17, 0)

time.sleep(3)

Left = EncoderLeft.getTicks() 
Right = EncoderRight.getTicks()
print Left, Right

pi.set_PWM_dutycycle(23, 0)
pi.set_PWM_dutycycle(22, 0)

time.sleep(3)

Left = EncoderLeft.getTicks() 
Right = EncoderRight.getTicks()
print Left, Right

pi.stop()
