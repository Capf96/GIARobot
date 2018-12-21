import RPi.GPIO as GPIO

A_pin = 12
B_pin = 6

GPIO.setmode(GPIO.BCM)
GPIO.setup(A_pin, GPIO.IN)
GPIO.setup(B_pin, GPIO.IN)

outcome = [0,-1,1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0]
last_AB = 0b00
counter = 0
while True:
    A = GPIO.input(A_pin)
    B = GPIO.input(B_pin)
    current_AB = (A << B) | B
    position = (last_AB << 2) | current_AB
    counter += outcome[position]
    last_AB = current_AB
    print counter
    

    
