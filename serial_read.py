import serial
import time

arduino=serial.Serial('COM3',9600)
prevData = 0

def data_action(data):
    parse_data = data.split(",")
    angle = parse_data[0]
    time  = float(parse_data[1])/1000
    print('Time: {} \t Angle: {}'.format(str(time), str(angle)))

while True:
    inData = arduino.readline()
    try:
        inString = inData.decode('utf-8')
    except:
        continue
    headers = inString.split(";")
    if headers[0] == "DATA":
        data_action(headers[1])

