import serial
from time import sleep
import pigpio
import time
#

pi = pigpio.pi()

ser = serial.Serial('/dev/ttyUSB0', 9600)
pi.set_mode(18, pigpio.OUTPUT)
pi.set_mode(17, pigpio.OUTPUT)

pi.set_mode(23, pigpio.OUTPUT)
pi.set_mode(22, pigpio.OUTPUT)
K = 0
s=[]
while K<10:
    data = ''.join(list(ser.readline().strip()))
    if data:
        s.append(data)
        print (s)
        s = []
    pi.set_PWM_dutycycle(17, 100)
    pi.set_PWM_dutycycle(18, 0)
    #print(ser.readline())

    pi.set_PWM_dutycycle(23, 100)
    pi.set_PWM_dutycycle(22, 0)
    #print(ser.readline())
    #time.sleep(10)
    K+=1
    

pi.set_PWM_dutycycle(18, 0)
pi.set_PWM_dutycycle(17, 0)
pi.set_PWM_dutycycle(23, 0)
pi.set_PWM_dutycycle(22, 0)


pi.stop()
'''
s = []
ser = serial.Serial('/dev/ttyUSB0', 9600)
while True:
    data = ''.join(list(ser.readline()))
    if data:
        s.append(data)
        print (s)
        s = []
'''
