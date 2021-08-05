import numpy as np
import pickle
from scipy.spatial.transform import Rotation as R
from pydairlib.common import FindResourceOrThrow


LOSS_WEIGHTS_FOLDER = 'examples/contact_parameter_learning/cassie_loss_weights/'

class CassieLossWeights():
  def __init__(self,
               pos=np.ones((19,)),
               vel=np.ones((19,)),
               omega=np.ones((3,)),
               quat=1):
    self.pos = np.diag(pos)
    self.vel = np.diag(vel)
    self.omega = np.diag(omega)
    self.quat = quat

  def save(self, filename):
    rel_filepath = LOSS_WEIGHTS_FOLDER + filename
    with open(rel_filepath + '.pkl', 'wb') as f:
      pickle.dump(self, f)

class CassieLoss():

  def __init__(self, filename='default_loss_weights'):
    self.position_slice = slice(4, 23)
    self.velocity_slice = slice(26, 45)
    self.quat_slice = slice(0, 4)
    self.rot_vel_slice = slice(23, 26)
    self.weights = self.load_weights(filename)

  def load_weights(self, filename):
    with open(LOSS_WEIGHTS_FOLDER + filename + '.pkl', 'rb') as f:
      return pickle.load(f)

  def CalcPositionsLoss(self, traj1, traj2):
    diff = traj1 - traj2
    return np.dot(diff.ravel(), (diff @ self.weights.pos).ravel()) / traj1.shape[0]

  def CalcVelocitiesLoss(self, traj1, traj2):
    diff = traj1 - traj2
    return np.dot(diff.ravel(), (diff @ self.weights.vel).ravel()) / traj1.shape[0]

  def CalcOmegaLoss(self, traj1, traj2):
    diff = traj1 - traj2
    return np.dot(diff.ravel(), (diff @ self.weights.omega).ravel()) / traj1.shape[0]

  def CalcQuatLoss(self, traj1, traj2):
    loss = 0
    for i in range(traj1.shape[0]):
      quat_diff = self.calc_rotational_distance(traj1[i], traj2[i])
      loss += quat_diff ** 2
    loss *= self.weights.quat / traj1.shape[0]
    return loss

  def CalculateLoss(self, traj1, traj2):
    l_pos = self.CalcPositionsLoss(traj1[:, self.position_slice], traj2[:, self.position_slice])
    l_vel = self.CalcVelocitiesLoss(traj1[:, self.velocity_slice], traj2[:, self.velocity_slice])
    l_omega = self.CalcOmegaLoss(traj1[:, self.rot_vel_slice], traj2[:, self.rot_vel_slice])
    l_quat = self.CalcQuatLoss(traj1[:, self.quat_slice], traj2[:, self.quat_slice])
    return l_pos + l_vel + l_omega + l_quat

  def calc_rotational_distance(self, quat1, quat2):
    q1 = quat1.ravel()
    q2 = quat2.ravel()
    R1 = R.from_quat([q1[1], q1[2], q1[3], q1[0]])
    R2 = R.from_quat([q2[1], q2[2], q2[3], q2[0]])
    Rel = R1 * R2.inv()
    return np.linalg.norm(Rel.as_rotvec()) ** 2
