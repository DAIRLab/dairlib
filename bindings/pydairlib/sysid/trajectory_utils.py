import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from pydrake.trajectories import PiecewisePolynomial

class trajectoryUtils():

	def __init__(self):
		self.graph = plt
		self.labels = []
		self.graph.rcParams.update({'font.size': 36})

	def getDerivativeFromData(self, breaks, knots):
		if len(breaks) != knots.shape[0]:
			print len(breaks)
			print knots.shape[0]
			raise IndexError("Dimensions of breaks and knots do not match")

		knotsderiv = np.array([]).reshape(0, knots.shape[1])
		for i in range(knots.shape[0]-1):
			deriv = (knots[i+1] - knots[i])/(breaks[i+1]-breaks[i])
			knotsderiv = np.concatenate([knotsderiv, [deriv]])
		print knotsderiv
		knotsderiv = np.concatenate([knotsderiv, [np.zeros(knotsderiv.shape[1])]])
		
		return PiecewisePolynomial.ZeroOrderHold(breaks, knotsderiv.transpose())

	def TrajectoryToMatrix(self, trajectory):
		data = []
		trajvector = np.array([]).reshape(0, 32)
		timevector = []	

		time = trajectory.start_time()
		print time
		while time < trajectory.end_time():
			data.append(trajectory.value(time)[0:32].flatten())
			timevector.append(time)
			time += (1/700.0)
		trajvector = np.array(data)
		return (timevector, trajvector)

		
	def plotTrajectory(self, trajectory, points, mycolor=None, mylabel=None, startIndex=None, endIndex=None):
		data = []

		if startIndex == None:
			startIndex = 0

		if endIndex == None:
			endIndex=trajectory.rows()
		
		x = np.linspace(trajectory.start_time(), trajectory.end_time(),((trajectory.end_time() - trajectory.start_time()))/points)
		y = np.array([]).reshape(0, endIndex - startIndex)

		for point in x:
			data.append(trajectory.value(point).flatten()[startIndex:endIndex])

		y = np.array(data)
		self.graph.plot(x, y, color=mycolor, label=mylabel)

	def plotTrajectoryDiff(self, traj1, traj2, points, mycolor=None, mylabel=None, startIndex=None, endIndex=None):
		data = []

		if startIndex == None:
			startIndex = 0

		if endIndex == None:
			endIndex=traj1.rows()
		
		x = np.linspace(traj1.start_time(), traj1.end_time(),((traj1.end_time() - traj1.start_time()))/points)
		y = np.array([]).reshape(0, endIndex - startIndex)

		for point in x:
			data.append(traj1.value(point).flatten()[startIndex:endIndex] - traj2.value(point).flatten()[startIndex:endIndex])

		y = np.array(data)
		self.graph.plot(x, y, color=mycolor, label=mylabel)

	def titleGraph(self, title, xlabel, ylabel):
		self.graph.title(title)
		self.graph.xlabel(xlabel)
		self.graph.ylabel(ylabel)

	def CalcDifferenceVector(self, traj1, traj2, samplingFrequency):

		if (traj1.rows() != traj2.rows()):
			raise ValueError("Traj1.rows() does not match Traj2.rows()")

		timeDiff = traj1.start_time() - traj2.start_time() #add to traj2.time
		currTime = traj1.start_time()

		differenceVector = np.array([]).reshape(0, traj1.rows())

		while (currTime < min(traj1.start_time(), traj2.start_time())):
			differenceVector = np.concatenate([differenceVector, [traj1.value(currTime) - traj2.value(currTime + timeDiff)]])

		return differenceVector
	
	def showTrajectoryGraph(self):
		self.graph.legend()
		self.graph.show()

	def title(self, mytitle):
		self.graph.title(title)

	def butterworth(nyf, x, y):
		b, a = butter(4, 1.5/nyf)
		fl = filtfilt(b, a, y)
		return fl