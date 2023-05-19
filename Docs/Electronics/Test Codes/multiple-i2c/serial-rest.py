import serial
ser = serial.Serial()
ser.baudrate = 1200
ser.port = '/dev/ttyACM3' #This has to be the port the Arduino is connected to, won't work if it's another.
ser
ser.open()
ser.close()
