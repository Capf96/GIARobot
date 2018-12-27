import RPi.GPIO as IO
import time
IO.setwarnings(False)
IO.setmode(IO.BCM)
IO.setup(4,IO.IN) #GPIO 4 -> IR sensor as input
while 1:
    print(IO.input(4))
    """if(IO.input(4)==True): #object is far away
          print("lejos")
    
    if(IO.input(4)==False): #object is near
       print("cerca")"""
