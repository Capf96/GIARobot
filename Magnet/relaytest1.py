from time import sleep
import pigpio
  

pi = pigpio.pi()

pi.set_mode(27, pigpio.OUTPUT)


pi.write(27, 1)

sleep(3)

pi.write(27, 0)
