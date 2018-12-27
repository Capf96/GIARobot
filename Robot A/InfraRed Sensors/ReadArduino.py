import serial
ser = serial.Serial('/dev/ttyUSB0', 9600)
s = []
while True:
    data = ''.join(list(ser.readline())[:-2])
    if data:
        s.append(data)
        print (s)
        s = []
