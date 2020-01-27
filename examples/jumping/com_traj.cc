#include "examples/jumping/com_traj.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib {
namespace examples {
namespace jumping {
namespace osc {

CoMTraj::CoMTraj(const RigidBodyTree<double>& tree, int hip_idx,
                 int left_foot_idx, int right_foot_idx,
                 PiecewisePolynomial<double> crouch_traj, double height)
    : tree_(tree),
      hip_idx_(hip_idx),
      left_foot_idx_(left_foot_idx),
      right_foot_idx_(right_foot_idx),
      crouch_traj_(crouch_traj),
      height_(height) {
  this->set_name("com_traj");
  // Input/Output Setup
  state_port_ = this
                    ->DeclareVectorInputPort(OutputVector<double>(
                        tree.get_num_positions(), tree.get_num_velocities(),
                        tree.get_num_actuators()))
                    .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort("com_traj", traj_inst, &CoMTraj::CalcTraj);
  time_idx_ = this->DeclareDiscreteState(1);
  com_x_offset_idx_ = this->DeclareDiscreteState(1);
  fsm_idx_ = this->DeclareDiscreteState(1);

  DeclarePerStepDiscreteUpdateEvent(&CoMTraj::DiscreteVariableUpdate);
}

EventStatus CoMTraj::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto prev_fsm_state =
      discrete_state->get_mutable_vector(fsm_idx_).get_mutable_value();
  auto prev_time =
      discrete_state->get_mutable_vector(time_idx_).get_mutable_value();
  auto com_x_offset =
      discrete_state->get_mutable_vector(com_x_offset_idx_).get_mutable_value();

  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double timestamp = robot_output->get_timestamp();
  double current_time = static_cast<double>(timestamp);

  if (prev_fsm_state(0) != fsm_state(0)) {  // When to reset the clock
    prev_fsm_state(0) = fsm_state(0);
    prev_time(0) = current_time;
    VectorXd q = robot_output->GetPositions();
    VectorXd v = robot_output->GetVelocities();

    KinematicsCache<double> cache = tree_.CreateKinematicsCache();
    multibody::SetZeroQuaternionToIdentity(&q);
    cache.initialize(q, v);
    tree_.doKinematics(cache);
    com_x_offset(0) = tree_.centerOfMass(cache)(0) - crouch_traj_.value
        (crouch_traj_.end_time())(0);
    std::cout << "x offset: " << com_x_offset(0) << std::endl;
  }
  return EventStatus::Succeeded();
}

PiecewisePolynomial<double> CoMTraj::generateNeutralTraj(
    const drake::systems::Context<double>& context, VectorXd& q,
    VectorXd& v) const {
  return generateBalancingComTraj(q);
}

PiecewisePolynomial<double> CoMTraj::generateCrouchTraj(
    const drake::systems::Context<double>& context, VectorXd& q,
    VectorXd& v) const {
  return crouch_traj_;
}

PiecewisePolynomial<double> CoMTraj::generateLandingTraj(
    const drake::systems::Context<double>& context, VectorXd& q,
    VectorXd& v) const {
//  const VectorXd com_x_offset =
//      this->EvalVectorInput(context, com_x_offset_idx_)->get_value();
  const VectorXd com_x_offset = context.get_discrete_state().get_vector(
      com_x_offset_idx_).get_value();

  // Only offset the x-position
  Vector3d offset;
  offset << com_x_offset(0), 0, 0;

  std::vector<double> breaks = crouch_traj_.get_segment_times();
  MatrixXd offset_matrix = offset.replicate(1, breaks.size());
  VectorXd breaks_vector = Eigen::Map<VectorXd>(breaks.data(), breaks.size());
  PiecewisePolynomial<double> com_offset =
      PiecewisePolynomial<double>::FirstOrderHold(breaks_vector, offset_matrix);
  return crouch_traj_ + com_offset;
}
PiecewisePolynomial<double> CoMTraj::generateBalancingComTraj(
    VectorXd& q) const {  // Kinematics cache and indices
  KinematicsCache<double> cache = tree_.CreateKinematicsCache();
  // Modify the quaternion in the begining when the state is not received from
  // the robot yet
  // Always remember to check 0-norm quaternion when using doKinematics
  multibody::SetZeroQuaternionToIdentity(&q);
  cache.initialize(q);
  tree_.doKinematics(cache);

  Vector3d pt_on_foot = Eigen::VectorXd::Zero(3);

  Vector3d l_foot = tree_.transformPoints(cache, pt_on_foot, left_foot_idx_, 0);
  Vector3d r_foot =
      tree_.transformPoints(cache, pt_on_foot, right_foot_idx_, 0);
  // Vector3d center_of_mass = tree_.centerOfMass(cache);

  Vector3d feet_center = (l_foot + r_foot) / 2;

  // desired pos is in between the two feet and at the current COM height
  Vector3d desired_com(feet_center(0), feet_center(1),
                       feet_center(2) + height_);
  return PiecewisePolynomial<double>(desired_com);
}

void CoMTraj::CalcTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();

  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  PiecewisePolynomial<double>* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);

  switch (static_cast<int>(fsm_state(0))) {
    case (0):  //  NEUTRAL
      *casted_traj = generateNeutralTraj(context, q, v);
      // std::cout << "Generated com for netural traj: " <<
      // casted_traj->getPolynomialMatrix(0) << std::endl;
      break;
    case (1):  //  CROUCH
      *casted_traj = generateCrouchTraj(context, q, v);
      break;
    case (2):  //  FLIGHT
      // *casted_traj = generateFlightTraj(context, q, v);
      // does nothing in flight
      break;
    case (3):  //  LAND
      *casted_traj = generateLandingTraj(context, q, v);
      break;
  }
}

}  // namespace osc
}  // namespace jumping
}  // namespace examples
}  // namespace dairlib
