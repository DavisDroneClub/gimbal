import matplotlib.pyplot as plt
import os
import csv
import numpy as np

files = os.listdir('data/')
params = []
time_values = []
euler_values = []
quaternion_values = []

for file in files:
    with open('data/'+file, encoding='utf-8-sig') as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        times = []
        euler_angles = []
        quaternion_angles = []

        params.append(file.split('.csv')[0].split('-'))
        
        for row in readCSV:
            times.append(int(row[0]))
            euler_angles.append(float(row[1]))
            quaternion_angles.append(float(row[2]))

        times[:] = [val - times[0] for val in times]
        euler_angles[:] = [val - euler_angles[0] for val in euler_angles]
        quaternion_angles[:] = [val - quaternion_angles[0] for val in quaternion_angles]
        
        time_values.append(times)
        euler_values.append(euler_angles)
        quaternion_values.append(quaternion_angles)
        
for i in range(len(params)):
    plt.figure()
    plt.grid()
    plt.xlim(0, time_values[i][-1])
    plt.plot(time_values[i], euler_values[i], label='Euler Angle')
    plt.plot(time_values[i], quaternion_values[i], label = 'Quaternion Angle')
    plt.legend()
    plt.xlabel('Time (ms)')
    plt.ylabel('Deviation (degrees)')
    title_string = 'Base Gain: ' + params[i][0] + ' Low Threshold: ' + params[i][1] + ' High Threshold: ' + params[i][2] + r" $\varepsilon$: " + params[i][3]
    plt.title(title_string)
    plt.savefig('plots/'+files[i].split('.csv')[0]+'.png')
    
