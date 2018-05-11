#Title:   Gimbal Tester
#Group:   Davis Drone Club
#Authors: Trevor Metz and Nicholas Chan
#Date:    10th May 2018

#Description: Measures response characteristics of one-axis gimbal

#--------------USER INPUT-----------------

COMPORT = 'COM3'
NUMPOINTS = 3000                                #Number of datapoints to collect
STARTPOINT = 0
TESTPOINT  = 30

#--------------INITIALIZATION-------------
#Importing libraries
import serial
import time
import csv
import sys
import matplotlib.pyplot as plt

#Initialize Variables
SP = 0                                          #Initialize setpoint
switch = 0
#Try to Open Serial
try:
    arduino=serial.Serial(COMPORT,115200)        #Initialize serial
except Exception as e:                          #If an error occurs, run this
    print("Check COM port number: " + str(e))
    sys.exit()
else:
    print("Arduino connected")

#Try to Open File
try:
    dataFile = open('data.csv', 'w')            #Initialize data file
except Exception as e:                          #If an error occurs, run this
    print("File may be open elsewhere: " + str(e))
    sys.exit()
    
time.sleep(5)                                   #Wait for Arduino to restart

def changeSetpoint(setpoint):
    printstr = "SETSP;"+str(setpoint)
    arduino.write(printstr.encode('utf-8'))

#Initialize Empty Lists
lines      = []
angle      = []
graphtime  = []
sp         = []

changeSetpoint(STARTPOINT)                      #Reset setpoint
arduino.write("SETDB;1.5".encode('utf-8'))      #Set deadband
time.sleep(0.01)                                #Pause between messages
arduino.write("TUNEI;5".encode('utf-8'))        #Set I parameter
print("Starting run")

#-----------------TEST--------------------

for i in range(NUMPOINTS):                      #Iterate from 0 to number of datapoints
    inData = arduino.readline()                 #Read data from Arduino
    try:                                        #Try to decode bytes to string
        inString = inData.decode('utf-8')
    except:                                     #If an error occurs,
        print('Read Error')                     #print a warning
        continue                                #and then skip this iteration.
    else:                                       #Otherwise,
        headers = inString.split(";")           #split the data by delimiter
        
    if headers[0] == "DATA":                    #If a data packet is received
        data = headers[1].split(",")            #Split data by delimiter
        data.append(str(SP))                    #Append current setpoint
    elif headers[0] == "WARN":                  #If a warning packet is received
        print("Warn received: " + headers[1])   #Print the warning message

    if (i > (NUMPOINTS/3))&(switch == 0):       #A third of the way through
        changeSetpoint(TESTPOINT)               #Change the setpoint
        SP = TESTPOINT
        switch = 1
    elif (i > (2*NUMPOINTS/3))&(switch == 1):   #Two thirds of the way through
        changeSetpoint(STARTPOINT)              #Reset the setpoint
        SP = STARTPOINT
        switch = 2
    try:
        lines.append(data)                      #Append data collected this iteration to list
    except:                                     #If something goes wrong, skip this iteration
        continue

#--------------POST PROCESSING------------
with dataFile:                                  #Open the datafile
    writer = csv.writer(dataFile, lineterminator='\r')  #Instantiate the writer
    for i in lines:                             #Iterate through each line
        angle.append(float(i[0]))               #Split the data into respective list
        graphtime.append(int(i[1]))
        sp.append(float(i[2]))
        writer.writerow([int(i[1]),float(i[0]),float(i[2])])    #Write data to CSV

plt.figure(1)                                   #Start a new figure
plt.plot(graphtime, angle, label='Measured angle') #Plot measured angle
plt.plot(graphtime, sp, label='Setpoint')       #Plot desired setpoint
plt.legend()                                    #Generate legend
plt.xlabel('Time (microseconds)')               #Label x axis
plt.ylabel('Angle (degrees)')                   #Label y axis
plt.show()                                      #Display plot
