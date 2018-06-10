import csv
import numpy as np 
time1 = []
gps_x = []

time2 = []
accelxy = []

with open('log/Graph1.txt') as csvDataFile:
    csvReader = csv.reader(csvDataFile)

    for row in csvReader:
        time1.append(row[0])
        gps_x.append(row[1])

with open('log/Graph2.txt') as csvDataFile:
    csvReader = csv.reader(csvDataFile)

    for row in csvReader:
        time2.append(row[0])
        accelxy.append(row[1])


del time1[0]
del gps_x[0]
del time2[0]
del accelxy[0]
gps_x = np.array(gps_x)
accelxy = np.array(accelxy)
xdata = gps_x.astype(np.float)
acceldata= accelxy.astype(np.float)
print(time1)
print(xdata)

print("Std. dev of GPS.x: ",np.std(xdata))
print("Std. dev of AccelXY: ",np.std(acceldata))