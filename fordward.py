#!/usr/bin/python
import pigpio
import time
#
pi = pigpio.pi()
pi.set_mode(18, pigpio.OUTPUT)
pi.set_mode(17, pigpio.OUTPUT)

pi.set_mode(23, pigpio.OUTPUT)
pi.set_mode(22, pigpio.OUTPUT)

pi.set_PWM_dutycycle(17, 50)
pi.set_PWM_dutycycle(18, 0)

pi.set_PWM_dutycycle(23, 0)
pi.set_PWM_dutycycle(22, 0)

time.sleep(10)

pi.set_PWM_dutycycle(18, 0)
pi.set_PWM_dutycycle(17, 0)
pi.set_PWM_dutycycle(23, 0)
pi.set_PWM_dutycycle(22, 0)
pi.stop()



