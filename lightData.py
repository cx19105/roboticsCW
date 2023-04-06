import csv
import matplotlib.pyplot as plt
from scipy import signal
import numpy as np
from operator import add


fleft = []
left = []
centre = []
right = []
fright = []
lbump = []
rbump = []
dist = []


with open('ambientLight.csv', newline='') as csvfile:
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
print(lc)
print(rc)

sumCL = list(map(add, centre, left))
sumCR = list(map(add, centre, right))
sumBump = list(map(add, lbump, rbump))
'''
plt.plot(dist, signal.filtfilt(b,a,lbump))
plt.plot(dist, signal.filtfilt(b,a,rbump))
plt.plot(dist, signal.filtfilt(b,a,centre))
plt.plot(dist, signal.filtfilt(b,a,left))
plt.plot(dist, signal.filtfilt(b,a,right))
plt.plot(dist, signal.filtfilt(b,a,fleft))
plt.plot(dist, signal.filtfilt(b,a,fright))

print(lc[0])
plt.plot(x, lc[0]*x + lc[1])
plt.plot(x, rc[0]*x + rc[1])
plt.plot(x, rcfilt)
plt.plot(x, lcfilt)'''

plt.plot(dist, signal.filtfilt(b,a,left))
plt.plot(dist, signal.filtfilt(b,a,centre))
plt.plot(dist, signal.filtfilt(b,a,right))
plt.grid()
plt.xlabel("mm")
plt.ylabel("Sensor Reading")
#plt.axvline(x=max(dist)-72)
#plt.legend(['Left bump', 'Right bumper', 'Centre', 'Left', 'Right', 'Far left', 'Far right'])
plt.legend(['Left', 'Centre', 'Right'])
plt.show()