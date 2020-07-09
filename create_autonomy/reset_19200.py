import serial
from time import sleep

ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=0.1)
ser.read(100)
sleep(0.5)
ser.write(bytearray([128]))
sleep(0.5)
ser.write(bytearray([7]))
sleep(0.5)
ser.flush()
sleep(0.5)
ser.close()
sleep(0.5)
exit()
