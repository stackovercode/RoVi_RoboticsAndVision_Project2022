import csv
import matplotlib.pyplot as plt


#
#
# A quick and ugly code for ploting data to test RRT
#
#

stepSizeT1 = []
timeList1 = []
stepSizeP1 = []
pathSize1 = []

stepSizeT2 = []
timeList2 = []
stepSizeP2 = []
pathSize2 = []

stepSizeT3 = []
timeList3 = []
stepSizeP3 = []
pathSize3 = []

# 1: from -0.959931, -0.959931, 1.0472, -1.57237, -1.57237, 0
with open('/home/rovi2022/projects/pythonMakePlots/stepSize_vs_pathSize1.txt', 'r') as file:

    reader = csv.reader(file, delimiter=' ')
    for row in reader:
        first_column = row[0]
        second_column = row[1]
        stepSizeP1.append(float(row[0]))
        pathSize1.append(int(row[1]))
        print(f'stepSizeP: {first_column}, pathSize: {second_column}')

with open('/home/rovi2022/projects/pythonMakePlots/stepSize_vs_generationTime1.txt', 'r') as file:
    reader = csv.reader(file, delimiter=' ')
    for row in reader:
        first_column = row[0]
        second_column = row[1]
        stepSizeT1.append(float(row[0]))
        timeList1.append(float(row[1]))
        print(f'stepSizeT: {first_column}, timeList: {second_column}')
with open('/home/rovi2022/projects/pythonMakePlots/stepSize_vs_pathSize2.txt', 'r') as file:
    reader = csv.reader(file, delimiter=' ')
    for row in reader:
        first_column = row[0]
        second_column = row[1]
        stepSizeP2.append(float(row[0]))
        pathSize2.append(int(row[1]))
        print(f'stepSizeP: {first_column}, pathSize: {second_column}')

with open('/home/rovi2022/projects/pythonMakePlots/stepSize_vs_generationTime2.txt', 'r') as file:
    reader = csv.reader(file, delimiter=' ')
    for row in reader:
        first_column = row[0]
        second_column = row[1]
        stepSizeT2.append(float(row[0]))
        timeList2.append(float(row[1]))
        print(f'stepSizeT: {first_column}, timeList: {second_column}')


with open('/home/rovi2022/projects/pythonMakePlots/stepSize_vs_pathSize3.txt', 'r') as file:
    reader = csv.reader(file, delimiter=' ')
    for row in reader:
        first_column = row[0]
        second_column = row[1]
        stepSizeP3.append(float(row[0]))
        pathSize3.append(int(row[1]))
        print(f'stepSizeP: {first_column}, pathSize: {second_column}')

with open('/home/rovi2022/projects/pythonMakePlots/stepSize_vs_generationTime3.txt', 'r') as file:
    reader = csv.reader(file, delimiter=' ')
    for row in reader:
        first_column = row[0]
        second_column = row[1]
        stepSizeT3.append(float(row[0]))
        timeList3.append(float(row[1]))
        print(f'stepSizeT: {first_column}, timeList: {second_column}')
fig, ax = plt.subplots()
# Plot the data
ax.plot(stepSizeP1, pathSize1, color='b', label="Top start")
ax.plot(stepSizeP2, pathSize2, color='g', label="Middel start")
ax.plot(stepSizeP3, pathSize3, color='r', label="Lower start")
ax.set_yticks(range(min(pathSize1), max(pathSize1),10))
# Add a title and axis labels
ax.set_title('My Plot')
ax.set_xlabel('Step Size')
ax.set_ylabel('Path Size')
ax.set_title(label="Step size vs Path Size")
# Show the plot
plt.legend()
plt.show()

fig, ax = plt.subplots()
# Plot the data
ax.plot(stepSizeT1, timeList1,color='b', label="Top start")
ax.plot(stepSizeT2, timeList2,color='g', label="Middel start")
ax.plot(stepSizeT3, timeList3,color='r', label="Lower start")
# Add a title and axis labels
ax.set_title('My Plot')
ax.set_xlabel('Step Size')
ax.set_ylabel('Time (ms)')
ax.set_title(label="Step size vs Generation time")
# Show the plot
plt.legend()
plt.show()
