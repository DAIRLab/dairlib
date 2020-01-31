
import csv
import os
import pdb
import sys
import time

import numpy as np

from scipy import signal
from scipy.spatial.transform import Rotation, RotationSpline
from scipy.interpolate import CubicSpline

import matplotlib.pyplot as plt

if len(sys.argv) < 3:
    print("usage: python[3] process_dynamics.py [INBAG] [OUTCSV]")
    sys.exit()

# static parameters for toss detection
TOSS_BALL_TOLERANCE = 1e-3
TOSS_BALL_H = 10

# far edge of board in board coordinates (y-axis)
BOARD_EDGE_Y = 0.6

# false if datapoint is over the board
OFFBOARD_CONDITION = lambda data, i: data[i,1] > BOARD_EDGE_Y

# true if datapoint is over the board
ONBOARD_CONDITION = lambda data, i: not OFFBOARD_CONDITION(data, i)

# true if block moves significantly over horizon H
MOVING_CONDITION = lambda data, i: maxDeltaOverHorizon(data[i:,:], TOSS_BALL_H) > TOSS_BALL_TOLERANCE

# false if block moves significantly over horizon H
STOPPED_CONDITION = lambda data, i: not MOVING_CONDITION(data, i)

# returns maximum ||\delta x|\ over a horizon of H in data
def maxDeltaOverHorizon(data, H):
    diff_data = data[1:H,:] - data[0:(H-1),:]
    return np.max(np.linalg.norm(diff_data,axis=1))

# find first index of data for which condition() returns true
def get_first_true(data,condition,H):
    for i in range(0, data.shape[0] - (H-1)):
        if condition(data, i):
            return i
    return -1

# finds first toss in data, return (-1, 0) if no experiment found
def get_first_experiment(data):
    removal_time = get_first_true(data[:,1:4],OFFBOARD_CONDITION,1)
    if removal_time < 0:
        return (-1,0)
    start_time = get_first_true(data[removal_time:,1:4],ONBOARD_CONDITION,1) + removal_time
    if start_time < 0:
        return (-1,0)
    stopped_time = get_first_true(data[start_time:,1:4],STOPPED_CONDITION,TOSS_BALL_H) + start_time
    if stopped_time < 0:
        return (-1,0)
    end_time = stopped_time + TOSS_BALL_H
    return (start_time,end_time)

# permutation of get_first_experiment to start at index start
def get_first_experiment_after(start, data):
    (s,e) = get_first_experiment(data[start:,:])
    return (s + start, e + start)

def extract_experiments(data):
    starts = []
    ends = []
    s_last = 0

    # loop through data and extract experiment indices
    while True:
        # find next experiment
        (s,e) = get_first_experiment_after(s_last, data)

        # break if no remaining expeiment found
        if s < s_last:
            break
        s_last = e
        starts = starts + [s]
        ends = ends + [e]

    return (starts,ends)


# run apriltag_csv.py to convert rosbag to csv file
datestr = str(int(time.time()))
csvfn = datestr + '_temp.csv'
inbag = sys.argv[1]
print('python2 apriltag_csv.py ' + inbag + ' ' + csvfn)
os.system('python2 apriltag_csv.py ' + inbag + ' ' + csvfn)
outcsv = sys.argv[2]

# get number of datapoints
num_t = 0
with open(csvfn,'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    num_t = sum(1 for row in plots)

# read CSV trajectory
t = np.zeros(num_t)
p_t = np.zeros((3,num_t))
q_t = np.zeros((4,num_t))
bp_t = np.zeros((3,num_t))
bq_t = np.zeros((4,num_t))
i = 0
with open(csvfn,'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for (i, row) in enumerate(plots):
        row = np.asarray([float(e) for e in row])
        t[i] = row[0]
        q_t[:,i] = row[1:5]
        p_t[:,i] = row[5:8]
        bq_t[:,i] = row[8:12]
        bp_t[:,i] = row[12:15]

# clean up temporary CSV
os.system('rm ./' + csvfn)

# assume orientation of plate is constant
Rot_board = Rotation.from_quat(bq_t.T)[0]

rot_t = Rot_board.inv() * Rotation.from_quat(q_t.T)
p_t =  Rot_board.inv().apply(p_t.T - bp_t.T).T

t = t-t[0]
# calculate derivatives with spline interpolation
rspline = RotationSpline(t, rot_t)
pspline = CubicSpline(t, p_t.T)

quat_t = rspline(t).as_quat().T

w_t = rspline(t, 1).T
dp_t = pspline(t, 1).T

# butterworth filter of order 2 to smooth velocity states

# sampling frequency
fs = 40.

# Cut-off frequency of angular velocity filter. < 20.0 (Nyquist)
fc_w = 5.

# Cut-off frequency of linear velocity filter. < 20.0 (Nyquist)
fc_v = 5.

# filter angular velocity
w_w = np.clip((fc_w / (fs / 2)), a_min = 0.000001, a_max = 0.999999) # Normalize the frequency
b, a = signal.butter(2, w_w, 'low')
for i in range(3):
    w_t[i,:] = signal.filtfilt(b, a, w_t[i,:],padtype='even',padlen=100)

# filter linear velocity
w_v = np.clip((fc_v / (fs / 2)), a_min = 0.000001, a_max = 0.999999) # Normalize the frequency
b, a = signal.butter(2, w_v, 'low')
for i in range(3):
    dp_t[i,:] = signal.filtfilt(b, a, dp_t[i,:],padtype='even',padlen=100)


# package data into matrix
data = np.concatenate((np.expand_dims(t, axis=0), p_t, quat_t, dp_t, w_t), axis=0)
data = data.T

# get indices bounding experiment trials
(starts, ends) = extract_experiments(data)

# add indices to data matrix for CSV saving
start_vect = 0*t
end_vect = 0*t
for i in range(len(starts)):
    start_vect[i] = starts[i]
    end_vect[i] = ends[i]
data = np.concatenate((np.expand_dims(start_vect, axis=0), \
                       np.expand_dims(end_vect, axis=0), \
                       np.expand_dims(t, axis=0), \
                       p_t, quat_t, dp_t, w_t), axis=0)

# save data to CSV
np.savetxt(outcsv, data, delimiter=',')

# plotting

plt.figure(1)
plt.plot(t, p_t.T)
#plt.plot(t, data[:,2])
#pdb.set_trace()
for i in range(3):
    plt.scatter(t[starts],p_t[i,starts].T)
for i in range(3):
    plt.scatter(t[ends],p_t[i,ends].T)
plt.legend(['x','y','z'])

plt.show()
