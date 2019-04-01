from MotorControl import Motor
import time
import serial
ser = serial.Serial('/dev/ttyUSB0', 9600)

motorDerecho = Motor("Motor Derecho", 23, 22, 19, 16)
motorDerecho.setup()

motorIzquierdo = Motor("Motor Izquierdo", 18, 17, 12, 6)
motorIzquierdo.setup()
a = 0
correct = 0
#data = ''.join(list(ser.readline())[:-2])

while a < 100:
    dt = 0.1

    rightEnc = motorDerecho.readEncoder()
    leftEnc = motorIzquierdo.readEncoder()




    #motorDerecho.run(+50 +5.90)
    #motorIzquierdo.run(50)
   
    #data = ''.join(list(ser.readline()))
    
    #print(int(data))
    #if float(data) > 400:
      # break
    print(str(rightEnc)+" "+str(leftEnc))
    a = a + dt
    time.sleep(dt)

"""
motorDerecho.run(60)
motorIzquierdo.run(50)
dt = 0.1
distancia_objetivo = 8
dl = 0
dr = 0

while (dl < distancia_objetivo and dr < distancia_objetivo) :

    dr = motorDerecho.readEncoder()

    dl = motorIzquierdo.readEncoder()

    print(str(dl) + " " + str(dr))

    time.sleep(dt)
    
"""

motorIzquierdo.stop()
motorDerecho.stop()
