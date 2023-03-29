from os import listdir
from os.path import isfile, join
import csv
import re
import numpy as np
import matplotlib.pyplot as plt


class Width:
    def __init__(self, filename):
        
        self.distname = self.getDist(filename)
        self.fleft = []
        self.left = []
        self.centre = []
        self.right = []
        self.fright = []
        self.lbump = []
        self.rbump = []
        self.dist = []
        self.distList = []
        self.writeData(filename)

    def getDist(self, filename):
        charList = []

        dir = ''.join([c for c in filename if c.isupper()])
        
        if dir == 'L':
            value = -int(value[0])
        else:
            value = int(value[0])
        return value
    
    def writeData(self, filename):
        with open('widths/'+filename, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                try:
                    self.fleft.append(int(row[0]))
                    self.left.append(int(row[1]))
                    self.centre.append(int(row[2]))
                    self.right.append(int(row[3]))
                    self.fright.append(int(row[4]))
                    self.lbump.append(int(row[5]))
                    self.rbump.append(int(row[6]))
                    self.dist.append(float(row[7]))
                except ValueError:
                    pass
        

    def getDistDiscretisation(self, step, maxDist):
        distVal = 0
        left = []
        right = []
        centre = []
        fleft = []
        fright = []
        lbump = []
        rbump = []
        while distVal < maxDist:
            index = self.dist.index(min(self.dist, key=lambda x:abs(x-distVal)))
            left.append(self.left[index])
            right.append(self.right[index])
            centre.append(self.centre[index])
            fleft.append(self.fleft[index])
            fright.append(self.fright[index])
            lbump.append(self.lbump[index])
            rbump.append(self.rbump[index])
            
            distVal += step

        return left, centre, right, fleft, fright, lbump, rbump



def plotHeatmap(dicts, labels):
    
    index = 0
    fig, axs = plt.subplots(4, 2)
    for ax in axs.flat:
        currentDict = dicts[index]
        dataMatrixLS = np.array([currentDict[i] for i in currentDict.keys()])
        im = ax.imshow(dataMatrixLS)

        ax.set_yticks(np.arange(len(currentDict.keys())))
        ax.set_yticklabels(currentDict.keys())
        ax.set_title(labels[index])
        index += 1
        if index >= len(dicts):
            break
    plt.tight_layout()
    plt.show()


widthfiles = [file for file in listdir('widths') if isfile(join('widths', file))]
widths = []

for file in widthfiles:
    widthFile = Width(file)
    widths.append(widthFile)

leftLS = {}
rightLS = {}
centreLS = {}
fLeftLS = {}
fRightLS = {}
lBumpLS = {}
rBumpLS = {}

widths.sort(key=lambda x:x.distname, reverse=True)


for width in widths:
    
    left, centre, right, fleft, fright, lbump, rbump = width.getDistDiscretisation(5, 150)
    leftLS[width.distname] = left
    rightLS[width.distname] = right
    centreLS[width.distname] = centre
    fLeftLS[width.distname] = fleft
    fRightLS[width.distname] = fright
    lBumpLS[width.distname] = lbump
    rBumpLS[width.distname] = rbump




labels = ['Left', 'Right', 'Centre', 'Far Left', 'Far Right', 'Left Bumper', 'Right Bumper']
plotHeatmap([leftLS, rightLS,centreLS, fLeftLS, fRightLS, lBumpLS, rBumpLS] , labels)
