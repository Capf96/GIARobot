#!/usr/bin/python
import pigpio
import time
from rotaryEncoder import *
#
#Variables PID
dt = 0.01
Kd = 2
Kp = 100
speed = 128 # Velocidad a la que se quiere que giren los motores
speed_M1 = speed
speed_M2 = speed


pi = pigpio.pi() 

pi.set_mode(9,pigpio.OUTPUT)  # Motor 1 (left) pwm D1 gpio 9 atras
pi.set_mode(25,pigpio.OUTPUT) # Motor 1 (left) pwm D2 gpio 25 adelante

#pi.set_mode(20,pigpio.INPUT) # Motor 1 (left) Encoder A gpio 20
#pi.set_mode(26,pigpio.INPUT) # Motor 1 (left) Encoder B gpio 26

pi.set_mode(11,pigpio.OUTPUT) # Motor 2 (right) pwm D1 gpio 11 adelante
pi.set_mode(8,pigpio.OUTPUT)  # Motor 2 (right) pwm D2 gpio 8 atras

#pi.set_mode(16,pigpio.INPUT) # Motor 2 (right) Encoder A gpio 16
#pi.set_mode(19,pigpio.INPUT) # Motor 2 (right) Encoder B gpio 19

#M1_EncoderA_o = pi.read(20) # Leer el valor del Encoder A del Motor 1 (left) Encoder A gpio 20
#M1_EncoderB_o = pi.read(26) # Leer el valor del Encoder B del Motor 1 (left) Encoder B gpio 26

#M2_EncoderA_o = pi.read(16) # Leer el valor del Encoder A Motor 2 (right) Encoder A gpio 16
#M2_EncoderB_o = pi.read(19) # Leer el valor del Encoder B Motor 2 (right) Encoder B gpio 19

Encoder_M1 = Encoder(pi, 9, 25)

Encoder_M2 = Encoder(pi, 26, 20)

Encoder_M1_o = Encoder_M1.getTicks()
Encoder_M2_o = Encoder_M2.getTicks()

def PID(o, f):
    salida = Kp*(f-o) + Kd*(f-o)/dt
    return salida

def SpeedRange(n):
    if n >= 255:
        return 255
    elif n <= -255:
        return -255
    else :
     return n

def Stop():
    pi.set_PWM_dutycycle(9,  0) # Motor 1 D1 para
    pi.set_PWM_dutycycle(25, 0) # Motor 1 D2 para

    pi.set_PWM_dutycycle(11, 0) # Motor 2 D1 para
    pi.set_PWM_dutycycle(8,  0) # Motor 2 D2 para

    pi.stop()  

        
raw_input("Presiona algo para iniciar")
contador = 0
while contador < 700 :

    #M1_EncoderA_f = pi.read(20) # Leer el valor del Encoder A del Motor 1 (left) Encoder A gpio 20
    #M1_EncoderB_f = pi.read(26) # Leer el valor del Encoder B del Motor 1 (left) Encoder B gpio 26
    
    #M2_EncoderA_f= pi.read(16) # Leer el valor del Encoder A Motor 2 (right) Encoder A gpio 16
    #M2_EncoderB_f= pi.read(19) # Leer el valor del Encoder B Motor 2 (right) Encoder B gpio 19

    Encoder_M1_f = Encoder_M1.getTicks()
    Encoder_M2_f = Encoder_M2.getTicks()

    
    speed_M1 = SpeedRange(speed - PID(Encoder_M1_o, Encoder_M1_f))
    speed_M2 = SpeedRange(speed - PID(Encoder_M1_o, Encoder_M2_f))

    if  speed_M1 >= 0:
        
        speed_M1_bck = 0
        speed_M1_fwd = speed_M1 

    else:

        speed_M1_fwd = SpeedRange(0)
        speed_M1_bck = speed_M1 

    if  speed_M2 >= 0:
        
        speed_M2_bck = 0
        speed_M2_fwd = speed_M2

    else:

        speed_M2_fwd = 0
        speed_M2_bck = speed_M2
        

  
    
    pi.set_PWM_dutycycle(9,   0) # Motor 1 D1 gira 
    pi.set_PWM_dutycycle(25,  speed) # Motor 1 D2 gira 

    pi.set_PWM_dutycycle(11,  speed)# Motor 2 D1 gira 
    pi.set_PWM_dutycycle(8,   0) # Motor 2 D2 gira 

    
    print("V Motor 1 "+ str(Encoder_M1_f) +"V Motor 2 "+str(Encoder_M2_f))

    Encoder_M1_o = Encoder_M1_f 
    Encoder_M2_o = Encoder_M2_f 


    contador = contador + 1

    time.sleep(dt)
                              
Stop()


