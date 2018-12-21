from RPi import GPIO
from time import sleep

# Pinout setup
clkl, dtl, clkr, dtr = 19, 16, 06, 12
pwml, motl, pwmr, motr = 18, 17, 22, 23

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(clkr, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dtr, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(clkl, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dtl, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(pwml, GPIO.OUT)
GPIO.setup(motl, GPIO.OUT)
GPIO.setup(pwmr, GPIO.OUT)
GPIO.setup(motr, GPIO.OUT)

# GPIO.PWM(channel, frequency) 0<=frequency<=100
pwmLeft = GPIO.PWM(18, 50)
pwmRight = GPIO.PWM(22, 50)

encoder1pos, n1 = 0, 0
encoder1PinLast = n1

encoder2pos, n2 = 0, 0
encoder2PinLast = n2

# PID parametters
vel, pwmOut, pastProp = 120, 0, 0
der, prop, int = 0, 0, 0
kd, kp, ki = 0.0, 1.0, 0.0

while True:
    #  Que el robot AVANCE o RETROCEDA en linea recta una distancia d
    d, orientation = 10, 0
    encoder1pos, n1 = 0, 0
    encoder1PinLast = n1
    encoder2pos, n2 = 0, 0
    encoder2PinLast = n2
    encoder1PinLast = n1
    ########################################################
    # Actualizar encoders
    n1 = GPIO.input(dtl)
    if( encoder1PinLast == 0 and n1==1):
        if(GPIO.input(clkl) == 0):
            encoder1pos +=1
        else:
            encoder1pos -=1

    encoder2PinLast = n2
    n2 = GPIO.input(dtr)
    if( encoder2PinLast == 0 and n2==1):
        if(GPIO.input(clkr) == 0):
            encoder2pos +=1
        else:
            encoder2pos -=1
    ########################################################
    while encoder1pos<d*orientation and encoder2pos < d*sentido :
        print(encoder1pos + " " + encoder2pos)
    # motores(0,0)

GPIO.cleanup()
