# %%
import glob
import csv
import numpy as np
import matplotlib.pyplot as plt
import pylab
import matplotlib as mpl
import unidecode

file_PATH = ''

x, y, time = [], [], []

# read ground truth
with open(file_PATH + 'pose_slam.txt') as csv_file:
    csv_reader = csv.DictReader(csv_file, delimiter=' ')
    for row in csv_reader:
        time.append(float(row['timestamp']))
        x.append(float(row['x']))
        y.append(float(row['y']))
time_np = np.array(time)
xy = np.array([x, y])

pylab.figure('Trajectory')
pylab.plot(x, y, label='Ground Truth')
pylab.axis('equal')
pylab.legend(loc='upper left')
pylab.show(block=False)

# scan and read student files
# plt.ion()
results = {}
csv_files = glob.glob(file_PATH + '*.csv')
for csv_path in csv_files:
    print(csv_path)
    with open(csv_path) as csv_file:
        x_i, y_i, time_i = [], [], []
        student_name = csv_path[0:len(file_PATH)-4]
        csv_reader = csv.DictReader(csv_file, delimiter=',')
        for row in csv_reader:
            # time_i.append(float(row['timestamp']))
            x_i.append(float(row['x']))
            y_i.append(float(row['y']))
        # time_i_np = np.array(time_i)
        xy_i = np.array([x_i, y_i])

        # plot trajectory
        pylab.figure('Trajectory')
        pylab.plot(x_i, y_i, label=unidecode.unidecode(student_name))
        pylab.axis('equal')
        pylab.legend(loc='upper right')
        pylab.xlabel('X (meter)')
        pylab.ylabel('Y (meter)')
        # pylab.show()

        # check file size and shape
        if xy_i.shape != xy.shape:
            raise ValueError('CSV file size wrong: ' + student_name)
        # if not np.array_equal(time_np, time_i_np):
        #     raise ValueError('CSV file timestampe not equal: ' + student_name)

        mse = np.sqrt(((xy - xy_i) ** 2).sum(axis=0))

        # evaluate the histogram
        values, base = np.histogram(mse, bins=400)
        cumulative = np.cumsum(values)

        mse_mean = np.mean(mse)
        results[student_name] = mse_mean
        print("Student name:%s, Error:%f" % (student_name, mse_mean))

        pylab.figure('Results')
        pylab.plot(base[:-1], cumulative, label=student_name)
        pylab.legend(loc='upper right')
        pylab.xlabel('Error (meter)')
        pylab.ylabel('Number of Samples')
        # pylab.show()
        # results{} =

pylab.figure('Trajectory')
pylab.show(block=False)
pylab.figure('Results')
pylab.show(block=False)

pylab.figure('Trajectory')
pylab.show()
