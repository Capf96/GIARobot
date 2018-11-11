from RPi import GPIO
from time import sleep

clkr = 06
dtr = 12
clkl = 19
dtl = 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(clkr, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dtr, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(clkl, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dtl, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

counter = [0, 0]
clkLastState = [GPIO.input(clkr), GPIO.input(clkl)]

try:

        while True:
                clkState = [GPIO.input(clkr), GPIO.input(clkl)]
                dtState = [GPIO.input(dtr), GPIO.input(dtr)]
                if clkState[0] != clkLastState[0]:
                        if dtState[0] != clkState[0]:
                                counter[0] -= 1
                        else:
                                counter[0] += 1
                        print(counter)
                if clkState[1] != clkLastState[1]:
                        if dtState[1] != clkState[1]:
                                counter[1] -= 1
                        else:
                                counter[1] += 1
                        print(counter)
                
                clkLastState = clkState
                sleep(0.0000001)
finally:
        GPIO.cleanup()
