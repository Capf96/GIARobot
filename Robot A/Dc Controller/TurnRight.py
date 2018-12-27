#!/usr/bin/python
import pigpio
import time
#
pi = pigpio.pi()
pi.set_mode(18, pigpio.OUTPUT)
pi.set_mode(17, pigpio.OUTPUT)

pi.set_mode(23, pigpio.OUTPUT)
pi.set_mode(22, pigpio.OUTPUT)

pi.set_PWM_dutycycle(18, 0)
pi.set_PWM_dutycycle(17, 255)

pi.set_PWM_dutycycle(23, 0)
pi.set_PWM_dutycycle(22, 255)
time.sleep(3)

pi.set_PWM_dutycycle(18, 0)
pi.set_PWM_dutycycle(17, 0)
pi.set_PWM_dutycycle(23, 0)
pi.set_PWM_dutycycle(22, 0)
pi.stop()
