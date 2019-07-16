import serial
 
port = "/dev/ttyUSB0"
 
def parseGPS(data,f):
#    print "raw:", data #prints raw data
	print(data)
	data = data.decode("utf-8")
	if data[:6] == '$GPGGA':
		print(data)
		f.write(data)
def decode(coord):
	#Converts DDDMM.MMMMM > DD deg MM.MMMMM min
	x = coord.split(".")
	head = x[0]
	tail = x[1]
	deg = head[0:-2]
	min = head[-2:]
	return deg + " deg " + min + "." + tail + " min"
 
 
print("Receiving GPS data")
f = open("log.txt","w")
ser = serial.Serial(port, baudrate = 4800, timeout = 0.5)
while True:
	data = ser.readline()
	parseGPS(data,f)