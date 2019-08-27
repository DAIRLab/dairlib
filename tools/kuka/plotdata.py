import matplotlib.pyplot as plt
import csv
import numpy as np
import pdb


from scipy import signal
fs = 40.



NKQ = 7
NKV = 7

NK = NKQ + NKV

NCR = 4
NCP = 3

NCW = 3
NCDP = 3

NCQ = NCR + NCP
NCV = NCW + NCDP

NC = NCQ + NCV


csvfn = 'test66.csv'
num_t = 0
with open(csvfn,'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        num_t = num_t + 1
t = np.zeros(num_t)
w_t = np.zeros((3,num_t))
v_t = np.zeros((3,num_t))

i = 0
with open(csvfn,'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        row = np.asarray([float(e) for e in row])
        t[i] = row[0]
        w_t[:,i] = row[(1 + NK + NCQ):(-NCDP)]
        v_t[:,i] = row[-NCDP:]
        i = i + 1
#pdb.set_trace()
'''
fc_w = 1.  # Cut-off frequency of the filter
w_w = np.clip((fc_w / (fs / 2)), a_min = 0.000001, a_max = 0.999999) # Normalize the frequency
#pdb.set_trace()
print(w_w)
b, a = signal.butter(2, w_w, 'low')
w_t = signal.filtfilt(b, a, w_t,padtype='even',padlen=100)

fc_v = 1.  # Cut-off frequency of the filter
w_v = np.clip((fc_v / (fs / 2)), a_min = 0.000001, a_max = 0.999999) # Normalize the frequency
print(w_w)
b, a = signal.butter(2, w_v, 'low')
v_t = signal.filtfilt(b, a, v_t,padtype='even',padlen=100)

'''

t = t - t[0]
plt.plot(t,w_t.T)
plt.xlabel('x')
plt.ylabel('y')
plt.title('Cube Rotational Velocity')
plt.legend(['wx','wy','wz'])
plt.show()

plt.plot(t,v_t.T)
plt.xlabel('x')
plt.ylabel('y')
plt.title('Cube Translational Velocity')
plt.legend(['vx','vy','vz'])
plt.show()