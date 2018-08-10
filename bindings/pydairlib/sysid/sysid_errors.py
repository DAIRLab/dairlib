import numpy as np
from scipy.signal import butter, lfilter
from cassie_utils import makeFixedBaseCassieTreePointer
from cassie_sysid_utils import getDerivativePredictionAtTime
from cassie_sysid_utils import getSimulationTrajectoryOverTime
from cassie_sysid_utils import lcmLogToTrajectory
from pydrake.trajectories import PiecewisePolynomial


def PlotSimulationErrorTrajectory(xLogPath, uLogPath):
	xtraj = lcmLogToTrajectory(xLogPath, "CASSIE_STATE", 10)
	utraj = lcmLogToTrajectory(uLogPath, "CASSIE_INPUT", 10)

	xdottraj = xtraj.derivative(1)
	udottraj = utraj.derivative(1)

	xtraj_simulated = getSimulationTrajectoryOverTime(utraj, xdottraj.end_time() - xdottraj.start_time(), xtraj.value(0))
	xdottraj_simulated = xtraj_out.derivative()

	return xdottraj_simulated

def PlotPredictionErrorTrajectory(xLogPath, uLogPath, timestep):
	xtraj = lcmLogToTrajectory(xLogPath)
	utraj = lcmLogToTrajectory(uLogPath)

	time = xtraj.start_time()

	udotarray_calculated = np.array([]).reshape(0, utraj.rows())
	timevector = []

	while time < xtraj.end_time():
		nextvector = getDerivativePredictionAtTime(time, utraj, xtraj.value(time))
		udotarray_calculated.concatenate(nextvector)
		timevector.concatenate(time)
		time+=timestep

	return PiecewisePolynomial.FirstOrderHold(timevector, udotarray_calculated)


def TrajectoryMicrosecondsToSeconds(trajectory):
	data = []
	matrix = np.array([]).reshape(0, trajectory.rows())
	timematrix = np.array([])

	realtime = 0
	realtime_end = (trajectory.end_time() - trajectory.start_time()) / 1e6  #in seconds

	realtime_step = (1/500.0)
	corrected_step = realtime_step * 1e6
	corrected_time = trajectory.start_time()

	while (realtime < realtime_end):
		timematrix = np.concatenate((timematrix, [realtime]))
		data.append(trajectory.value(corrected_time).flatten())
		#matrix = np.concatenate((matrix, [trajectory.value(corrected_time).flatten()]))

		realtime += realtime_step
		corrected_time += corrected_step

	matrix = np.array(data)
	return PiecewisePolynomial.FirstOrderHold(timematrix, matrix.transpose())

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a

def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

def CalcOneCassieSpringCoefficients(dataTraj, simTraj):
	tree = makeFixedBaseCassieTreePointer()

	B = np.array([]).reshape(0, 1)
	A = np.array([]).reshape(0, 16)

	time = simTraj.start_time()
	trajderiv = dataTraj.derivative(1)

	while(time < simTraj.end_time()):

		q = dataTraj.value(time)[0:16]  
		v = dataTraj.value(time)[16:32]
		kinsol = tree.doKinematics(q, v)
		j = tree.positionConstraintsJacobian(kinsol, False)
		mass = tree.massMatrix(kinsol)

		diff = (trajderiv.value(time)[16:32] - simTraj.value(time)[16:32])
		product = np.matmul(mass, diff)

		left = product.reshape(16,1)
		B = np.concatenate((B, left))

		right = np.diagflat(dataTraj.value(time)[0:16])

		jt = np.transpose(j)
		mi = np.linalg.inv(mass)

		right = (np.identity(16) - jt.dot(np.linalg.inv(j.dot(mi).dot(jt))).dot(j).dot(mi)).dot(right)	#I - Jt(JH^-1Jt)^-1JH^-1q

		A = np.concatenate((A, right))

		time += 1e-3

	print A.shape
	print B.shape

	constants = np.linalg.lstsq(A, B)
	return constants
