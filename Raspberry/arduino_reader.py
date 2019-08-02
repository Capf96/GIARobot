import serial
import sys
import time

ser = serial.Serial('/dev/ttyACM0', 57600)
gyro_values = []
start = time.time()
while True:
        result = str(ser.readline().strip())    # read up to one hundred bytes
        splitted = result.decode('utf-8').split(',')
        gyro_reading = float(splitted[0])
        gyro_values.append(gyro_reading)
        end = time.time()
        if end - start >= 1800: # En secs
                print("#DIFF: ",gyro_values[0], gyro_reading)
                f = open("Results.txt","w+")
                f.write(gyro_values[0])
                for i in range(1,len(gyro_values)):
                        f.write(",{}".format(gyro_values[i]))
                        f.close()
                gyro_values.clear()
                start = time.time()
