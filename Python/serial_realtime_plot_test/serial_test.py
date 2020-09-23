import serial
import struct

#initialize serial port
ser = serial.Serial()
ser.port = 'COM6' #Arduino serial port
ser.baudrate = 115200
ser.timeout = 10 #specify timeout when using readline()
ser.open()
if ser.is_open==True:
    print("\nAll right, serial port now open. Configuration:\n")
    print(ser, "\n") #print serial parameters


size = struct.calcsize('fffffffff')
print(size)

while True:
    # #Aquire and parse data from serial port
    # line=ser.readline()      #ascii
    # print(line)


    data = ser.read(size)

    tup = struct.unpack('fffffffff', data)
    print("acc: ", tup[0:3])
    print("gyro: ", tup[3:6])
    print("mag: ", tup[6:9])
    print()