#!/usr/bin/python
import pigpio
import time
#
pi = pigpio.pi() 

pi.set_mode(17,pigpio.OUTPUT) #Motor 1 (left) pwm D1 gpio 9
pi.set_mode(18,pigpio.OUTPUT) #Motor 1 (left) pwm D2 gpio 25  

pi.set_mode(22,pigpio.OUTPUT) #Motor 2 (right) pwm D1 gpio 11
pi.set_mode(23,pigpio.OUTPUT) #Motor 2 (right) pwm D2 gpio 8   

pi.set_PWM_dutycycle(17,  0) # Motor 1 D1 gira 1/4
pi.set_PWM_dutycycle(18,  200)# Motor 1 D2 gira 1/4

pi.set_PWM_dutycycle(22,  200) # Motor 2 D1 gira 1/4
pi.set_PWM_dutycycle(23,  0) #  Motor 2 D2 gira 1/4 

time.sleep(5) # Espera 5 segundos 

pi.set_PWM_dutycycle(17,  0) # Motor 1 D1 para
pi.set_PWM_dutycycle(18,  0)# Motor 1 D2 para

pi.set_PWM_dutycycle(22,  0) # Motor 2 D1 para
pi.set_PWM_dutycycle(23,  0) #  Motor 2 D2 para

pi.stop()                    
