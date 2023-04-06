import csv
import matplotlib.pyplot as plt
from scipy import signal
import numpy as np
from operator import add
from os import listdir
from os.path import isfile, join

files = ['lineHeights/'+file for file in listdir('lineHeights') if isfile(join('lightHeights', file))]

differenceList = []
distances = [1000,170,225,250,295,360,420,500]

for file in files:

    fleft = []
    left = []
    centre = []
    right = []
    fright = []
    lbump = []
    rbump = []
    dist = []

    with open(file, newline='') as csvfile:
        reader = csv.reader(csvfile)
        #print(reader)

        for row in reader:
            try:
                fleft.append(int(row[0]))
                left.append(int(row[1]))
                centre.append(int(row[2]))
                right.append(int(row[3]))
                fright.append(int(row[4]))
                lbump.append(int(row[5]))
                rbump.append(int(row[6]))
                dist.append(float(row[7]))
            except ValueError:
                pass

    b,a = signal.butter(5, 0.05, btype='lowpass')

    lowerRange = 0
    upperRange = -1

    rcfilt = signal.filtfilt(b,a, list(map(add, centre[lowerRange:upperRange], right[lowerRange:upperRange])))
    lcfilt = signal.filtfilt(b, a, list(map(add, centre[lowerRange:upperRange], left[lowerRange:upperRange])))
    x = dist[lowerRange:upperRange]
    x =np.array(x)
    lc = np.polyfit(x, lcfilt, 1)
    rc = np.polyfit(x, rcfilt, 1)

    sumCL = list(map(add, centre, left))
    sumCR = list(map(add, centre, right))
    sumBump = list(map(add, lbump, rbump))

    dist = [max(dist) - dist[i] for i in range(0,len(dist))]

    plt.plot(dist, signal.filtfilt(b,a,sumBump))
    difference = max(signal.filtfilt(b,a,sumBump)) - min(signal.filtfilt(b,a,sumBump))
    differenceList.append(difference)

    #plt.axvline(x=max(dist)-72)

plt.grid()
plt.xlabel("mm")
plt.ylabel("Sensor Reading")
plt.legend(files)
plt.show()

plt.plot(distances,differenceList)
plt.xlabel("mm")
plt.ylabel("Difference between max and min sensor readings")
plt.show()