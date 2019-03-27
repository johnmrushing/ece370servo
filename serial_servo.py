import serial
ser = serial.Serial(port='/dev/ttyS0',baudrate=9600)
print("Connected to: " + ser.portstr)
while True:
	input = raw_input("Enter angle for servo: ")
	direction = 1 if int(input) >= 0 else 0
	mag1 = abs(int(input))/256
	mag2 = abs(int(input))%256
	checksum = (direction + mag1 + mag2)%255
	ser.write(str( chr(0xFF) + chr(0xFF) + chr(direction) + chr(mag1) + chr(mag2) + chr(checksum)))
