import matplotlib.pyplot as plt
import pandas as pd
from pandas import Series, DataFrame
import numpy as np
import json

dirNewcomer = 'MRS_RTA'
dirStateOfTheArt = 'CBAA'
filestrOrig = 'TaskSet'
resultSet = dict()

for i in range(1, 26):
    filestr = dirNewcomer + '\\' + filestrOrig + str(i) + '.res'
    #print(i)
    #try:
    file = open(filestr)
    inputstr = file.read()
    #except:
    #print('No such file found' + filestr)
    resList = inputstr.splitlines()
    #print(resList)
    taskInfo = (0, 0, 0)# Task number, Start time, end time, difference
    results = dict()
    for line in resList:
        resSplit = line.split('\t')
        if len(resSplit) == 4:
            results[resSplit[0]] = (resSplit[1], resSplit[2], resSplit[3])
    resultSet[i] = results.copy()
    file.close()

resultSet2 = dict()
for i in range(1, 26):
    filestr =  dirStateOfTheArt + '\\' + filestrOrig + str(i) + '.res'
    #print(i)
    #try:
    file = open(filestr)
    inputstr = file.read()
    #except:
    #print('No such file found' + filestr)
    resList = inputstr.splitlines()
    taskInfo = (0, 0, 0)# Task number, Start time, end time, difference
    results = dict()
    for line in resList:
        resSplit = line.split('\t')
        if len(resSplit) == 4:
            results[resSplit[0]] = (resSplit[1], resSplit[2], resSplit[3])
    resultSet2[i] = results.copy()
    file.close()

#output results in a bar graph

plt.rcdefaults()
axesList = list()
figList = list()

# Example data
# for i in range(1,20):
#     y_pos = list(resultSet[i].keys())
#     fig = plt.figure()
#     performance1 = [round(float(resultSet[i][j][0])) for j in resultSet[i].keys()]
#     performance2 = [round(float(resultSet[i][j][1])) for j in resultSet[i].keys()]
#     performance3 = [round(float(resultSet[i][j][2])) for j in resultSet[i].keys()]
#     #print(json.dumps(resultSet))
#     ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
#     error = 3
#
#     ax.barh(y_pos, performance1)
#     ax.barh(y_pos, performance3, left=performance1)
#     ax.set_yticks(y_pos)
#     ax.set_yticklabels(resultSet[i].keys())
#
#     ax.invert_yaxis()  # labels read top-to-bottom
#     ax.set_xlabel('Time')
#     ax.set_title('Results 1')
#     axesList.append(ax)
#     figList.append(fig)
#
# for i in range(1,26):
#     y_pos = list(resultSet2[i].keys())
#     fig = plt.figure()
#     performance1 = [round(float(resultSet2[i][j][0])) for j in resultSet2[i].keys()]
#     performance2 = [round(float(resultSet2[i][j][1])) for j in resultSet2[i].keys()]
#     performance3 = [np.maximum(round(float(resultSet2[i][j][2])),0) for j in resultSet2[i].keys()]
#     #print(json.dumps(resultSet))
#     ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
#     error = 3
#
#     ax.barh(y_pos, performance1)
#     ax.barh(y_pos, performance3, left=performance1)
#     ax.set_yticks(y_pos)
#     ax.set_yticklabels(resultSet2[i].keys())
#
#     ax.invert_yaxis()  # labels read top-to-bottom
#     ax.set_xlabel('Time')
#     ax.set_title('Results 2')
#     axesList.append(ax)
#     figList.append(fig)

for i in range(1,26):
    y_pos = list(resultSet[i].keys())
    fig = plt.figure()
    performance1 = [round(float(resultSet[i][j][0])) for j in resultSet[i].keys()]
    performance2 = [round(float(resultSet[i][j][1])) for j in resultSet[i].keys()]
    performance3 = [round(float(resultSet[i][j][2])) for j in resultSet[i].keys()]
    #print(json.dumps(resultSet))
    ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    #error = [np.maximum(0,np.sign(float(resultSet2[i][j][1])) *-1) for j in resultSet2[i].keys()]
    ax.set_xlim(0, 300)

    ax.barh(y_pos, performance1)
    ax.barh(y_pos, performance3, left=performance1)
    ax.set_yticks(y_pos)
    ax.set_yticklabels(resultSet[i].keys())


    ax.invert_yaxis()  # labels read top-to-bottom
    ax.set_xlabel('Time')
    ax.set_title('MRS-RTA task completion times for Task set ' + str(i))
    axesList.append(ax)
    figList.append(fig)


#plt.show()
i = 1
for fig in figList:
    fig.savefig("ResultImgMRS-RTA\\ResultImg" + str(i) + ".png")
    i += 1

figList.clear()

for i in range(1,26):
    y_pos = list(resultSet2[i].keys())
    fig = plt.figure()
    performance1 = [round(float(resultSet2[i][j][0])) for j in resultSet2[i].keys()]
    performance2 = [round(float(resultSet2[i][j][1])) for j in resultSet2[i].keys()]
    performance3 = [round(float(resultSet2[i][j][2])) for j in resultSet2[i].keys()]
    #print(json.dumps(resultSet))
    ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    error = [np.maximum(0,np.sign(float(resultSet2[i][j][1])) *-1) for j in resultSet2[i].keys()]
    ax.set_xlim(0, 300)

    ax.barh(y_pos, performance1,xerr= error)
    ax.barh(y_pos, performance3, left=performance1)
    ax.set_yticks(y_pos)
    ax.set_yticklabels(resultSet2[i].keys())

    ax.invert_yaxis()  # labels read top-to-bottom
    ax.set_xlabel('Time')
    ax.set_title('CBAA task completion times for Task set ' + str(i))
    axesList.append(ax)
    figList.append(fig)


#plt.show()
i = 1
for fig in figList:
    fig.savefig("ResultImgCBAA\\ResultImg" + str(i) + ".png")
    i += 1

figList.clear()

for i in range(1,26):
    y_pos = list(resultSet2[i].keys())
    fig = plt.figure()
    performance1 = [round(float(resultSet[i][j][0])) for j in resultSet2[i].keys()]
    performance2 = [round(float(resultSet[i][j][1])) for j in resultSet2[i].keys()]
    performance3 = [np.maximum(round(float(resultSet2[i][j][2])),0) - round(float(resultSet[i][j][0]))  for j in resultSet2[i].keys()]
    #print(json.dumps(resultSet))
    ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    ax.set_xlim(-300, 300)
    error = [np.maximum(0,np.sign(float(resultSet2[i][j][1])) *-1) for j in resultSet2[i].keys()]

    ax.barh(y_pos, performance3,xerr= error)
    #ax.barh(y_pos, performance3, left=performance1)
    ax.set_yticks(y_pos)
    ax.set_yticklabels(resultSet2[i].keys())

    ax.invert_yaxis()  # labels read top-to-bottom
    ax.set_xlabel('Time')
    ax.set_title('Time differences for Task set ' + str(i))
    axesList.append(ax)
    figList.append(fig)


#plt.show()
i = 1
for fig in figList:
    fig.savefig("ResultImgDiff\\ResultImg" + str(i) + ".png")
    i += 1

figList.clear()


