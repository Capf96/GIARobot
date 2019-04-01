import pigpio
from time import sleep
pi = pigpio.pi()

pi.set_mode(12, pigpio.INPUT)
pi.set_mode(6, pigpio.INPUT)

pi.set_mode(18, pigpio.OUTPUT)
pi.set_mode(17, pigpio.OUTPUT)

a=0

while a<10:
    pi.set_PWM_dutycycle(17, 177.5)
    pi.set_PWM_dutycycle(18, 0)

    a = pi.read(12)
    b = pi.read(6)
    print(str(a)+str(b))
    a=a+0.1
    sleep(0.1)
    
