import serial
ser = serial.Serial('/dev/ttyACM0', 9600)
ser.flushInput()

class Arduino(object):

	def getQTR (self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			for i in range(len(s)):
				s[i]=int(s[i])
			self.values = s
		return s
