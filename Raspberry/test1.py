import time

from MotorControl import Motors
#from arduinoReader import Arduino
#from robot import Robot

motores = Motors(17,27) # Pines GPIO17 y GPIO27
#arduino = Arduino()
#robot = Robot()

time.sleep(3)
motores.run(50)
time.sleep(2)
motores.stop()

#robot.followLine(25)
