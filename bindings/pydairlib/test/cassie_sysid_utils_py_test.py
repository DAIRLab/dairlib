import unittest
import numpy as np
import math
import trajectory_utils
from pydrake.trajectories import PiecewisePolynomial as pp
from cassie_sysid_utils import getDerivativePredictionAtTime
from cassie_sysid_utils import getSimulationTrajectoryOverTime
from cassie_sysid_utils import lcmLogToTrajectory

class TestCassieSysidUtils(unittest.TestCase):
	def setUp(self):
		x = np.linspace(0,5*math.pi, 1500)
		y = np.sin(x)
		self.traj = pp.FirstOrderHold(x, np.array([y, y, y, y, y, y, y, y, y, y]))

	def test_getSimulationTrajectoryOverTime(self):
		getSimulationTrajectoryOverTime(self.traj, 1, np.zeros(32))

	def test_getDerivativePredictionAtTime(self):
		time = 0
		x = np.zeros(32)
		getDerivativePredictionAtTime(time, self.traj, x)

	def test_lcmLogToTrajectory(self):
		x = lcmLogToTrajectory("test.log", "CASSIE_STATE", 10.0)
		tu = trajectory_utils.trajectoryUtils()
		tu.plotTrajectory(x)

if __name__ == '__main__':
    unittest.main()
