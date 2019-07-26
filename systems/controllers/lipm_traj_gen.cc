#include "systems/controllers/lipm_traj_gen.h"

#include <math.h>
#include <string>

using std::cout;
using std::endl;
using std::string;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::EventStatus;
using drake::systems::BasicVector;

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;

namespace dairlib {
namespace systems {

LIPMTrajGenerator::LIPMTrajGenerator(const RigidBodyTree<double>& tree,
                                     double desired_com_height,
                                     double stance_duration_per_leg,
                                     int left_foot_idx,
                                     Vector3d pt_on_left_foot,
                                     int right_foot_idx,
                                     Vector3d pt_on_right_foot) :
    tree_(tree),
    desired_com_height_(desired_com_height),
    stance_duration_per_leg_(stance_duration_per_leg),
    left_foot_idx_(left_foot_idx),
    right_foot_idx_(right_foot_idx),
    pt_on_left_foot_(pt_on_left_foot),
    pt_on_right_foot_(pt_on_right_foot) {
  this->set_name("lipm_traj");

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(OutputVector<double>(
                  tree.get_num_positions(),
                  tree.get_num_velocities(),
                  tree.get_num_actuators())).get_index();
  fsm_port_ = this->DeclareVectorInputPort(
                BasicVector<double>(1)).get_index();
  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp_part(VectorXd(0));
  MatrixXd K = MatrixXd::Ones(0,0);
  MatrixXd A = MatrixXd::Identity(0, 0);
  MatrixXd alpha = MatrixXd::Ones(0, 0);
  ExponentialPlusPiecewisePolynomial<double> exp(K, A, alpha, pp_part);
  drake::trajectories::Trajectory<double>& traj_inst = exp;
  this->DeclareAbstractOutputPort("lipm_traj", traj_inst,
      &LIPMTrajGenerator::CalcTraj);

  // Discrete state event
  DeclarePerStepDiscreteUpdateEvent(&LIPMTrajGenerator::DiscreteVariableUpdate);
  // The time of the last touch down
  prev_td_time_idx_ = this->DeclareDiscreteState(1);
  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));

  // Check if the model is floating based
  is_quaternion_ = multibody::IsFloatingBase(tree);
}


EventStatus LIPMTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {

  // Read in finite state machine
  const BasicVector<double>* fsm_output = (BasicVector<double>*)
      this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  auto prev_td_time = discrete_state->get_mutable_vector(
                        prev_td_time_idx_).get_mutable_value();
  auto prev_fsm_state = discrete_state->get_mutable_vector(
                          prev_fsm_state_idx_).get_mutable_value();

  if (fsm_state(0) != prev_fsm_state(0)) {  //if at touchdown
    prev_fsm_state(0) = fsm_state(0);

    // Get time
    const OutputVector<double>* robot_output = (OutputVector<double>*)
        this->EvalVectorInput(context, state_port_);
    double timestamp = robot_output->get_timestamp();
    double current_time = static_cast<double>(timestamp);
    prev_td_time(0) = current_time;
  }

  return EventStatus::Succeeded();
}


void LIPMTrajGenerator::CalcTraj(const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {

  // Read in current state
  const OutputVector<double>* robot_output = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  VectorXd v = robot_output->GetVelocities();

  // Read in finite state machine
  const BasicVector<double>* fsm_output = (BasicVector<double>*)
      this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  // Get discrete states
  const auto prev_td_time = context.get_discrete_state(
                              prev_td_time_idx_).get_value();

  // Get time
  double timestamp = robot_output->get_timestamp();
  double current_time = static_cast<double>(timestamp);

  double end_time_of_this_interval = prev_td_time(0) + stance_duration_per_leg_;
  // Ensure "current_time < end_time_of_this_interval" to avoid error in
  // creating trajectory.
  if ((end_time_of_this_interval <= current_time + 0.001)) {
    end_time_of_this_interval = current_time + 0.002;
  }

  // Kinematics cache and indices
  KinematicsCache<double> cache = tree_.CreateKinematicsCache();
  VectorXd q = robot_output->GetPositions();

  // Modify the quaternion in the begining when the state is not received from
  // the robot yet (cannot have 0-norm quaternion when using doKinematics)
  if (is_quaternion_){
    multibody::SetZeroQuaternionToIdentity(&q);
  }

  cache.initialize(q);
  tree_.doKinematics(cache);
  int stance_foot_index = (fsm_state(0) == right_stance_state_) ?
                          right_foot_idx_ : left_foot_idx_;
  Vector3d pt_on_stance_foot = (fsm_state(0) == right_stance_state_) ?
                          pt_on_right_foot_ : pt_on_left_foot_;

  // Get center of mass position and velocity
  Vector3d CoM = tree_.centerOfMass(cache);
  MatrixXd J = tree_.centerOfMassJacobian(cache);
  Vector3d dCoM = J * v;

  // Stance foot position (Forward Kinematics)
  Vector3d stance_foot_pos = tree_.transformPoints(cache,
      pt_on_stance_foot, stance_foot_index, 0);

  // Get CoM_wrt_foot for LIPM
  const double CoM_wrt_foot_x = CoM(0) - stance_foot_pos(0);
  const double CoM_wrt_foot_y = CoM(1) - stance_foot_pos(1);
  const double CoM_wrt_foot_z = (CoM(2) - stance_foot_pos(2));
  const double dCoM_wrt_foot_x = dCoM(0);
  const double dCoM_wrt_foot_y = dCoM(1);
  const double dCoM_wrt_foot_z = dCoM(2);
  DRAKE_DEMAND(CoM_wrt_foot_z > 0);

  // create a 3D one-segment polynomial for ExponentialPlusPiecewisePolynomial
  std::vector<double> T_waypoint_com = {current_time, end_time_of_this_interval};

  std::vector<MatrixXd> Y(T_waypoint_com.size(), MatrixXd::Zero(3, 1));
  Y[0](0, 0) = stance_foot_pos(0);
  Y[1](0, 0) = stance_foot_pos(0);
  Y[0](1, 0) = stance_foot_pos(1);
  Y[1](1, 0) = stance_foot_pos(1);
  Y[0](2, 0) = CoM(2);
  // Y[0](2, 0) = desired_com_height_;
  Y[1](2, 0) = desired_com_height_;

  MatrixXd Y_dot_start = MatrixXd::Zero(3, 1);
  Y_dot_start(2, 0) = dCoM_wrt_foot_z;
  MatrixXd Y_dot_end = MatrixXd::Zero(3, 1);

  PiecewisePolynomial<double> pp_part = PiecewisePolynomial<double>::Cubic(
                                          T_waypoint_com,
                                          Y, Y_dot_start,
                                          Y_dot_end);

  // Dynamics of LIPM
  // ddy = 9.81/CoM_wrt_foot_z*y, which has an analytical solution.
  // Let omega^2 = 9.81/CoM_wrt_foot_z.
  // Let y0 and dy0 be the intial position and velocity. Then the solution is
  //   y = k_1 * exp(w*t) + k_2 * exp(-w*t)
  // where k_1 = (y0 + dy0/w)/2
  //       k_2 = (y0 - dy0/w)/2.
  double omega = sqrt(9.81 / CoM_wrt_foot_z);
  double k1x = 0.5 * (CoM_wrt_foot_x + dCoM_wrt_foot_x / omega);
  double k2x = 0.5 * (CoM_wrt_foot_x - dCoM_wrt_foot_x / omega);
  double k1y = 0.5 * (CoM_wrt_foot_y + dCoM_wrt_foot_y / omega);
  double k2y = 0.5 * (CoM_wrt_foot_y - dCoM_wrt_foot_y / omega);

  // Sum of two exponential + one-segment 3D polynomial
  MatrixXd K = MatrixXd::Zero(3, 2);
  MatrixXd A = MatrixXd::Zero(2, 2);
  MatrixXd alpha = MatrixXd::Zero(2, 1);
  K << k1x, k2x,
       k1y, k2y,
       0  , 0  ;
  A << omega, 0,
       0    , -omega;
  alpha << 1, 1;

  // Assign traj
  ExponentialPlusPiecewisePolynomial<double>* casted_traj =
      (ExponentialPlusPiecewisePolynomial<double>*)
      dynamic_cast<ExponentialPlusPiecewisePolynomial<double>*> (traj);
  *casted_traj = ExponentialPlusPiecewisePolynomial<double>(K, A, alpha, pp_part);
}

}  // namespace systems
}  // namespace dairlib


