from RPi import GPIO
from time import sleep

clkl = 19
dtl = 16
clkr = 06
dtr = 12

GPIO.setmode(GPIO.BCM)
GPIO.setup(clkr, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dtr, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(clkl, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dtl, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

encoder1pos = 0
n1 = 0
encoder1PinLast = n1

encoder2pos = 0
n2 = 0
encoder2PinLast = n2

while True:
    
    encoder1PinLast = n1
    n1 = GPIO.input(dtl)
    if( encoder1PinLast == 0 and n1==1):
        if(GPIO.input(clkl) == 0):
            encoder1pos +=1
        else:
            encoder1pos -=1
    print(encoder1pos)
    
    encoder2PinLast = n2
    n2 = GPIO.input(dtr)
    if( encoderPinLast == 0 and n2==1):
        if(GPIO.input(clkr) == 0):
            encoder2pos +=1
        else:
            encoder2pos -=1
    print(encoder2pos)
    
GPIO.cleanup()
