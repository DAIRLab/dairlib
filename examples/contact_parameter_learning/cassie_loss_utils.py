import numpy as np
import pickle
from scipy.spatial.transform import Rotation as R
from pydairlib.common import FindResourceOrThrow
from scipy.integrate import trapezoid, simpson
from pyquaternion import Quaternion

LOSS_WEIGHTS_FOLDER = 'examples/contact_parameter_learning/cassie_loss_weights/'

class CassieLossWeights():
  def __init__(self,
               pos=np.ones((19,)),
               vel=np.ones((19,)),
               omega=np.ones((3,)),
               quat=1,
               pos_offset_weight=1):
    self.pos = np.diag(pos)
    self.vel = np.diag(vel)
    self.omega = np.diag(omega)
    self.quat = quat
    self.pos_offset_weight = pos_offset_weight
    self.impulse_weight = 1e-7

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
    # self.weights = CassieLossWeights()
    # self.weights.pos = 10 * self.weights.pos
    # self.weights.quat = 10 * self.weights.quat
    # self.weights.pos_offset_weight = 20
    # self.weights.save('pos_loss_weights')
    # self.weights.save('pos_loss_weights')

  def load_weights(self, filename):
    with open(LOSS_WEIGHTS_FOLDER + filename + '.pkl', 'rb') as f:
      return pickle.load(f)

  def print_weights(self):
    print('pos weights: ')
    print(np.diag(self.weights.pos))
    print('vel weights: ')
    print(np.diag(self.weights.vel))
    print('omega weights: ')
    print(np.diag(self.weights.omega))
    print('quat weights: ')
    print(self.weights.quat)

  # Using trapezoidal integration, calculate the per state integral of the trajectory
  def integrate_trajectory(self, traj):
    return trapezoid(traj, axis=0)

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
    loss = np.zeros((traj1.shape[0],))
    for i in range(traj1.shape[0]):
      loss[i] = (2 * Quaternion.distance(Quaternion(traj1[i]), Quaternion(traj2[i]))) ** 2
    return np.mean(loss)

  def CalculateLossTraj(self, traj1, traj2):
    l_pos = self.CalcPositionsLoss(traj1[:, self.position_slice], traj2[:, self.position_slice])
    l_vel = self.CalcVelocitiesLoss(traj1[:, self.velocity_slice], traj2[:, self.velocity_slice])
    l_omega = self.CalcOmegaLoss(traj1[:, self.rot_vel_slice], traj2[:, self.rot_vel_slice])
    l_quat = self.CalcQuatLoss(traj1[:, self.quat_slice], traj2[:, self.quat_slice])
    # print('l_pos: ' + str(l_pos))
    # print('l_vel: ' + str(l_vel))
    # print('l_omega: ' + str(l_omega))
    # print('l_quat: ' + str(l_quat))
    return l_pos + l_vel + l_omega + l_quat

  def CalculateLossParams(self, params):
    # Regularize the state offsets
    vel_offset = params['vel_offset']
    z_offset = params['z_offset']

    vel_offset_cost = vel_offset.T @ vel_offset
    z_offset_cost = self.weights.pos_offset_weight * z_offset.T @ z_offset

    return vel_offset_cost + z_offset_cost

  def CalculateLossNetImpulse(self, traj1, traj2):
    total_impulse_error = 0
    impulse_errors = np.zeros((4, 3))
    for foot in range(4):
      net_impulse_1 = self.integrate_trajectory(traj1[foot])
      net_impulse_2 = self.integrate_trajectory(traj2[foot])
      impulse_errors[foot] = net_impulse_1 - net_impulse_2
      total_impulse_error += self.weights.impulse_weight * impulse_errors[foot].T @ impulse_errors[foot]
    return total_impulse_error

  # def calc_rotational_distance(self, quat1, quat2):
  #   q1 = quat1.ravel()
  #   q2 = quat2.ravel()
  #   R1 = R.from_quat([q1[1], q1[2], q1[3], q1[0]])
  #   R2 = R.from_quat([q2[1], q2[2], q2[3], q2[0]])
  #   Rel = R1 * R2.inv()
  #   return np.linalg.norm(Rel.as_rotvec()) ** 2
