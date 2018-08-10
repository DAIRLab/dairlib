import unittest
import numpy as np
import scipy.signal as signal
from trajectory_utils import *
from sysid_errors import *
from pydrake.trajectories import PiecewisePolynomial
from cassie_sysid_utils import lcmLogToTrajectory
from cassie_utils import makeFixedBaseCassieTreePointer

class TestSysidErrors(unittest.TestCase):

	# def test_massMatrix(self):
	# 	tree = makeFixedBaseCassieTreePointer()
	# 	trajs = lcmLogToTrajectory("testjoint0.log")
	# 	xtraj = self.TrajectoryMicrosecondsToSeconds(trajs[0])
	# 	utraj = self.TrajectoryMicrosecondsToSeconds(trajs[1])

	# 	q = xtraj.value(xtraj.start_time())[0:16]
	# 	v = xtraj.value(xtraj.start_time())[16:32]

	# 	kinsol = tree.doKinematics(q, v)
	# 	print tree.massMatrix(kinsol)

	# def test_plotSimulationErrorTrajectoryTwoSimulations(self):

	# 	print 'what'
	# 	trajs = lcmLogToTrajectory("alljoints3.log")
	# 	xtraj = TrajectoryMicrosecondsToSeconds(trajs[0])
	# 	utraj = TrajectoryMicrosecondsToSeconds(trajs[1])
	# 	xtrajdot = xtraj.derivative(1)
	# 	offset = (trajs[1].start_time() - trajs[0].start_time())/1e6
	# 	print offset

	# 	tu = trajectoryUtils()
	# 	#tu.plotTrajectory(utraj, 1e-3)
	# 	#tu.plotTrajectory(trajs[1], 1e3)

	# 	print offset
	# 	xtest = getSimulationTrajectoryOverTime(utraj, utraj.end_time(), xtraj.value(offset)[0:32])
	# 	print 'made plot'
	# 	xtestdot = xtest.derivative(1)
	# 	time = xtraj.start_time()
	# 	tu.plotTrajectory(xtraj, 1e-3)
	# 	#tu.plotTrajectory(xtest, 1e-3)
	# 	tu.showTrajectoryGraph()

		# print ((xtraj.start_time() - xtraj.end_time()))
		# print ((utraj.start_time() - utraj.end_time()))
		# print xtest.start_time() - xtest.end_time()


		# print xtest.value(time)
		# print xtest.value(time + 1)

		# print xtrajdot.value(10)[0:32]
		# print xtestdot.value(10)

		# tu.plotTrajectory(xtest, 1e-3, 0, 8)
		# #tu.showTrajectoryGraph()

		# tu.plotTrajectory(xtraj, 1e-3, 0, 8)
		# tu.showTrajectoryGraph()

		# print xtraj.value(10)[0:32]
		# print xtest.value(10)

	# def test_PredictionErrorDifferenceBaseCase(self):

	# 	utraj = PiecewisePolynomial.FirstOrderHold([0, 10], np.zeros(20).reshape(10, 2))
	# 	xtraj = PiecewisePolynomial.FirstOrderHold([0, 10], np.array([ [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]))
	# 	derivatives = getDerivativePredictionAtTime(0, utraj, xtraj)
	# 	tu = trajectoryUtils()
	# 	tu.plotTrajectory(derivatives, 1e-3)
	# 	tu.showTrajectoryGraph()

	# def test_PlotTrajectory(self):
	# 	extrajs = lcmLogToTrajectory("lcmlog-2018-07-30.log")
	# 	trajs = lcmLogToTrajectory("alljoints13-1.log")
	# 	xtraj = TrajectoryMicrosecondsToSeconds(trajs[0])
	# 	utraj = TrajectoryMicrosecondsToSeconds(trajs[1])
	# 	exutraj = TrajectoryMicrosecondsToSeconds(extrajs[1])
	# 	tu = trajectoryUtils()
	# 	tu.plotTrajectory(exutraj, 1e-3, 'blue', 'Hip Pitch Motor Torque Experimental', 4, 5)
	#   	tu.plotTrajectory(utraj, 1e-3, 'green', 'Hip Pitch Motor Torque', 4, 5)
	#   	tu.titleGraph("u(t) of Hip Pitch Motor", "Time (s)", "Torque")
	# 	tu.showTrajectoryGraph()

	# def test_TrajToMatrix(self):
	# 	trajs = lcmLogToTrajectory("alljoints13-3.log")
	# 	xtraj = TrajectoryMicrosecondsToSeconds(trajs[0])
	# 	utraj = TrajectoryMicrosecondsToSeconds(trajs[1])
	# 	xtrajdot = xtraj.derivative(1)

	# 	tu = trajectoryUtils() 

	# 	matrix = tu.TrajectoryToMatrix(xtraj)
	# 	newxtraj = PiecewisePolynomial.FirstOrderHold(matrix[0], matrix[1].transpose())


	# 	for i in range(0, 16):
	# 		tu.plotTrajectory(xtraj, 1e-3, 'green', 'idk', i, i+1)
	# 		tu.plotTrajectory(newxtraj, 1e-3, 'blue', 'idk', i, i+1)
	# 		tu.showTrajectoryGraph()



	def test_PlotPredictionErrorTrajectoryTwoSimulations(self):
		intToJointMap = {0: 'hip_roll_left', 1:'hip_roll_right', 2:'hip_yaw_left', 3:'hip_yaw_right', 4:'hip_pitch_left', 5:'hip_pitch_right', 6:'knee_left', 7:'knee_right', 8:'knee_joint_left', 9:'knee_joint_right', 10:'ankle_joint_left', 11:'ankle_joint_right', 12:'ankle_spring_joint_left', 13:'ankle_spring_joint_right', 14:'toe_left', 15:'toe_right'}

		trajs = lcmLogToTrajectory("lcmlog-2018-07-30.log")

		xtraj = TrajectoryMicrosecondsToSeconds(trajs[0])
		utraj = TrajectoryMicrosecondsToSeconds(trajs[1])
		xtrajdot = xtraj.derivative(1)

		tu = trajectoryUtils()

		PredictionErrorMatrix = np.array([]).reshape(0, 32)
		time = utraj.start_time()
		timematrix = np.array([])

		derivatives = getDerivativePredictionAtTime(time, utraj, xtraj)

		fc = 40.0
		fs = 700.0
		w = fc / (fs / 2.0)

		matrix = tu.TrajectoryToMatrix(derivatives)
		b, a = signal.butter(6, w, 'low')
		derivativesmatrix = signal.filtfilt(b, a, matrix[1].transpose())
		derivativesfiltered = PiecewisePolynomial.FirstOrderHold(matrix[0], derivativesmatrix)

		xmatrix = tu.TrajectoryToMatrix(xtrajdot)
		xb, xa = signal.butter(6, w, 'low')
		xtrajmatrix = signal.filtfilt(xb, xa, xmatrix[1].transpose())
		xtrajdotfiltered = PiecewisePolynomial.FirstOrderHold(xmatrix[0], xtrajmatrix)

		xmatrix = tu.TrajectoryToMatrix(xtraj)
		xb, xa = signal.butter(6, w, 'low')
		xtrajmatrix = signal.filtfilt(xb, xa, xmatrix[1].transpose())
		xtrajfiltered = PiecewisePolynomial.FirstOrderHold(xmatrix[0], xtrajmatrix)

		# xtrajdotmatrix = tu.TrajectoryToMatrix(xtrajdot)
		# b, a = signal.butter(2, 0.2, output='ba')
		# xtrajdotmatrixfiltered = signal.filtfilt(b, a, xtrajdotmatrix[1])
		# xtrajdotfiltered = PiecewisePolynomial.FirstOrderHold(xtrajdotmatrix[0], xtrajdotmatrixfiltered.transpose())



		#tu.plotTrajectory(xtraj, 1e-3, 'green', 8, 9)
		#tu.plotTrajectory(utraj, 1e-3, 'green')
		# for i in range(0, 16):
		# 	# tu.titleGraph("u(t) of Hip Pitch Motor", "Time (s)", "Motor Torques")
		# 	# tu.plotTrajectory(utraj, 1e-3, 'green', 'Hip Pitch Motor u(t)', i, i+1)
		# 	# tu.showTrajectoryGraph()

		# 	# tu.titleGraph("q(t) of Hip Pitch Motor", "Time (s)", "Radial Position (rads)")
		# 	# tu.plotTrajectory(xtraj, 1e-3, 'blue', 'Hip Pitch Motor q(t)', i, i+1)
		# 	# tu.showTrajectoryGraph()
		# 	# # tu.plotTrajectory(xtraj, 1e-3, 'black', 'velocity', i+16, i+17)

		# 	# tu.titleGraph("qdot(t) of Hip Pitch Motor", "Time (s)", "Radial Velocity (rads/s)")
		# 	# tu.plotTrajectory(xtrajdot, 1e-3, 'orange', 'Radial Velocity from Data', i, i+1)
		# 	# tu.plotTrajectory(derivatives, 1e-3, 'red', 'Radial Velocity from Simulation', i, i+1)
		# 	# tu.showTrajectoryGraph()

		# 	# tu.plotTrajectory(xtrajdotfiltered, 1e-3, 'cyan', 'data vel filt', i, i+1)
		# 	# tu.plotTrajectory(derivativesfiltered, 1e-3, 'magenta', 'sim vel filt', i, i+1)

		# 	# tu.titleGraph("qddot(t) of Hip Pitch Motor", "Time (s)", "Radial Acceleration (rads/s/s)")
		# 	# tu.plotTrajectory(xtrajdot, 1e-3, 'cyan', 'Radial Acceleration from Data', i+16, i+17)
		# 	# tu.plotTrajectory(derivatives, 1e-3, 'magenta', 'Radial Acceleration from Simulation', i+16, i+17)
		# 	# tu.showTrajectoryGraph()

		# 	print "joint: " + str(i)
		# 	#tu.titleGraph("qddot(t) of Hip Pitch Motor", "Time (s)", "Radial Acceleration (rads/s/s)")
		# 	tu.titleGraph(intToJointMap[i], "Time (s)", "Radial Acceleration (rads/s/s)")
		# 	tu.plotTrajectory(xtrajdot, 1e-3, 'cyan', 'Experimental Data', i+16, i+17)
		# 	tu.plotTrajectory(derivatives, 1e-3, 'magenta', 'Corrected Simulation Data', i+16, i+17)
		# 	tu.plotTrajectoryDiff(xtrajdot, derivatives, 1e-3, 'red', 'uncorrected', i+16, i+17)
		# 	#tu.plotTrajectory(xtrajdotfiltered.derivative(1), 1e-3, 'purple', 'data acc filt', i, i+1)
		# 	#tu.plotTrajectory(derivativesfiltered.derivative(1), 1e-3, 'pink', 'sim acc filt', i, i+1)
		# 	#tu.plotTrajectoryDiff(xtrajdotfiltered.derivative(1), derivativesfiltered.derivative(1), 1e-3, 'purple', 'uncorrected', i, i+1)
		# 	tu.showTrajectoryGraph()


		# 	# tu.plotTrajectory(xtraj, 1e-3, 'blue', 'data vel.', i+16, i+17) #velocities
		# 	# #tu.plotTrajectory(xtrajdot, 1e-3, 'purple', 'acc.', i+16, i+17) #accelerations
		# 	# tu.plotTrajectory(xtrajfiltered, 1e-3, 'purple', 'filt vel data', i, i+1)
		# 	# tu.plotTrajectory(xtrajfiltered, 1e-3, 'yellow', 'filt acc data', i+16, i+17) #filtered velocities
		# 	# #tu.plotTrajectory(derivatives, 1e-3, 'red', 'sim vel.', i, i+1) #velocities
			
		# 	# tu.plotTrajectory(derivativesfiltered, 1e-3, 'orange', 'sim vel', i, i+1) #filtered velocities
		# 	# #u.plotTrajectory(derivatives, 1e-3, 'orange', 'sim acc.', i+16, i+17) #accelerations
		# 	# tu.plotTrajectory(derivativesfiltered, 1e-3, 'pink', 'filt acc', i+16, i+17) #filtered accelerations
		# 	#tu.plotTrajectory(xtrajdotfiltered, 1e-2, 'yellow', i, i+1)
		# 	# print "joint: " + str(i)
		# 	# tu.showTrajectoryGraph()
		# 	#tu.plotTrajectory(utraj, 1e-3, 'green')
		# 	#tu.plotTrajectory(derivativesfiltered, 1e-3, 'red', i+16, i+17)
		# 	#print xmatrix
		# 	#print xtrajmatrix
			#tu.titleGraph(intToJoinMap[i])
			#tu.showTrajectoryGraph()
		print xtraj.start_time()
		print xtraj.end_time()
		print CalcOneCassieSpringCoefficients(xtrajfiltered, derivativesfiltered)

	# def test_CalcSpringConstants(self):
	# 	trajs = lcmLogToTrajectory("alljoints7-1.log")
	# 	xtraj = TrajectoryMicrosecondsToSeconds(trajs[0])
	# 	utraj = TrajectoryMicrosecondsToSeconds(trajs[1])
	# 	xtest = getDerivativePredictionAtTime(0, utraj, xtraj)

	# 	tu = trajectoryUtils()

	# 	matrix = tu.TrajectoryToMatrix(xtest)
	# 	b, a = signal.butter(4, 0.2, output='ba')
	# 	derivativesmatrix = signal.filtfilt(b, a, matrix[1])
	# 	#derivativesmatrix = self.reject_outliers(derivativesmatrix)
	# 	derivativesfiltered = PiecewisePolynomial.FirstOrderHold(matrix[0], derivativesmatrix.transpose())

	# 	matrix = tu.TrajectoryToMatrix(xtraj)
	# 	b, a = signal.butter(4, 0.2, output='ba')
	# 	derivativesmatrix = signal.filtfilt(b, a, matrix[1])
	# 	#derivativesmatrix = self.reject_outliers(derivativesmatrix)
	# 	xtrajfiltered = PiecewisePolynomial.FirstOrderHold(matrix[0], derivativesmatrix.transpose())


	# 	print CalcCassieKneeSpringCoefficients(utraj, xtraj, xtest)

	# def reject_outliers(data, m=2):
	# 	return data[abs(data - np.mean(data)) < m * np.std(data)]

''''
	def test_plotPredictionErrorTrjaectoryTwoSimulations(self):
		print '2'
		xtraj = lcmLogToTrajectory("input.log", "CASSIE_STATE", 10)
		print 'did first log'
		utraj = lcmLogToTrajectory("state.log", "CASSIE_INPUT", 10)
		print 'did log'

		xtest = PlotPredictionErrorTrajectory("test.log", "test.log")
		print 'made plot'

		time = xtraj.start_time()

		while time < xtraj.end_time():
			self.assertEqual(xtraj.value(time), xtest.value(time))
			time += 1
'''




if __name__ == '__main__':
    unittest.main()
