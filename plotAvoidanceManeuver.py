import csv
import matplotlib.pyplot as plt
import matplotlib.lines as lines

left = []
centre = []
right = []
x = []
y = []

with open('avoidanceManeuverRoutine.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)

    for row in reader:
        try:
            left.append(int(row[0]))
            centre.append(int(row[1]))
            right.append(int(row[2]))
            x.append(float(row[3]))
            y.append(float(row[4]))
        except ValueError:
            pass
            
y[:100] = [val+0 for val in y[:100]]            
y[100:200] = [val+55 for val in y[100:200]]
y[200:300] = [val+110 for val in y[200:300]]
x[200:300] = [val-30 for val in x[200:300]]
plt.plot(x[:100],y[:100],'g-',linewidth=2)
plt.plot(x[100:200],y[100:200],'b-',linewidth=2)
plt.plot(x[200:300],y[200:300],'r-',linewidth=2)

plt.plot([x[12],x[99]],[y[12],y[12]],linestyle='dashed',color='g',linewidth=1)
plt.plot([x[120],x[199]],[y[120],y[120]],linestyle='dashed',color='b',linewidth=1)
plt.plot([x[233],x[299]],[y[233],y[233]],linestyle='dashed',color='r',linewidth=1)

plt.legend(['Test 1','Test 2','Test 3','Heading 1','Heading 2','Heading 3'])

plt.xlabel("x")
plt.ylabel("y")
circle = plt.Circle((260, 0), 170, color='grey')
robot = plt.Circle((260, 0), 40, color='k')
halo = plt.Circle((260, 0), 100, color='orange')
plt.xlim(0,550)
plt.ylim(-200,200)
ax = plt.gca()
ax.add_patch(circle)
ax.add_patch(halo)
ax.add_patch(robot)
ax.set_aspect('equal', adjustable='box')
plt.draw()
plt.show()

plt.figure(2)
plt.plot(x[110:190],left[110:190])
plt.plot(x[110:190],right[110:190])
plt.xlabel('x')
plt.ylabel('Reading')
plt.legend(['Left','Right'])
plt.show()
