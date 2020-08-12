import matplotlib.pyplot as plt
import numpy as np
import math

figList = list()
titles = [
    'Single cluster',
    'Multiple clusters',
    'Even distribution',
    'Random distribution',
    'Radial distribution'
]
titles2 = [
    'Random dispatch',
    'Single cluster dispatch',
    'Decreasing dispatch',
    'Increasing dispatch',
    'Multiple cluster dispatch'
]
axesList = list()
for i in range(0, 25):
    fig = plt.figure()
    axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    axes.set_title(str(i+1) + ": "+ titles[i//5] + " "+titles2[i%5]) #replace with file name
    axes.set_ylim(-1500, 1500)
    axes.set_xlim(-1500, 1500)
    axes.set_xlabel('x')
    axes.set_ylabel('y')
    axesList.append(axes)
    #axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    #axes.set_ylim(-1500, 1500)
    #axes.set_xlim(-1500, 1500)
    #axes.set_title(titles[i]+titles2[i])
    #axesList.append(axes)
    figList.append(fig)

#coords_range = np.arange(-1500, 1500, 15)
#x = np.arange(0, 100, 100/15)
filestrOrig = 'TaskSet'
graphSet = dict()
timeSet = dict()
for i in range(1, 26):
    filestr = filestrOrig + str(i) + '.dat'
    #print(i)
    #try:
    file = open(filestr)
    inputstr = file.read()
    #except:
    #print('No such file found' + filestr)
    taskList = inputstr.splitlines()
    coordsListX = list()
    coordsListY = list()
    timeList = list()
    for task in taskList:
        taskSplit1 = task.split('/')
        taskTimeSplit = taskSplit1[0].split(' ')
        taskTime = taskTimeSplit[0]
        timeFlot = float(taskTime)
        print(timeFlot)
        timeList.append(timeFlot/10)
        taskCondition = taskSplit1[1]
        taskSplit2 = str(taskCondition).split()
        taskCoordsx = float(taskSplit2[2])
        coordsListX.append(taskCoordsx)
        taskCoordsy = float(taskSplit2[3])
        coordsListY.append(taskCoordsy)
    graphSet[i] = (coordsListX, coordsListY)
    timeSet[i] = timeList

#axes.scatter(coords_range, graphSet[1])
print(timeSet)
k = 1
for i in range(0, 5):
    for j in range(0, 5):
        axesList[k-1].scatter(graphSet[k][0], graphSet[k][1], c = timeSet[k])
        k+=1
plt.show()

i = 1
for fig in figList:
    fig.savefig("TaskSetImg" + str(i) + ".png")
    i += 1
#x = np.arange(0, 10, 0.2)
#y = np.sin(x)
#fig, ax = plt.subplots()
#ax.plot(x, y)
#plt.show()
