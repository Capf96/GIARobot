from MotorControl import Motor
import time

motorDerecho = Motor("Motor Derecho", 23, 22, 16, 19)
motorDerecho.setup()

motorIzquierdo = Motor("Motor Izquierdo", 18, 17, 12, 6)
motorIzquierdo.setup()
a = 0
while a < 3:
    dt = 0.1
    motorDerecho.run(-90)
    motorIzquierdo.run(-90)
    print(str(motorIzquierdo.readEncoder()) + "," + str(motorDerecho.readEncoder()))
    a = a + dt
    time.sleep(dt)

motorIzquierdo.stop()
motorDerecho.stop()

