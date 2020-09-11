#include "examples/Cassie/osc_jump/com_traj_generator.h"

#include "multibody/multibody_utils.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

using dairlib::multibody::createContext;
using std::cout;
using std::endl;
using std::pair;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::examples::osc_jump {

COMTrajGenerator::COMTrajGenerator(
    const MultibodyPlant<double>& plant, Context<double>* context,
    PiecewisePolynomial<double>& crouch_traj,
    const std::vector<std::pair<const Eigen::Vector3d,
                                const drake::multibody::Frame<double>&>>&
        feet_contact_points,
    double time_offset, FSM_STATE init_fsm_state)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      crouch_traj_(crouch_traj),
      feet_contact_points_(feet_contact_points),
      time_offset_(time_offset){
  this->set_name("com_traj");
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  clock_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort("com_traj", traj_inst,
                                  &COMTrajGenerator::CalcTraj);
  com_x_offset_idx_ = this->DeclareDiscreteState(1);
  initial_com_idx_ = this->DeclareDiscreteState(3);
  switch_time_idx_ = this->DeclareDiscreteState(3);
  VectorXd init_fsm(1);
  init_fsm << init_fsm_state;
  prev_fsm_idx_ = this->DeclareDiscreteState(init_fsm);

  DeclarePerStepDiscreteUpdateEvent(&COMTrajGenerator::DiscreteVariableUpdate);
  crouch_traj_.shiftRight(time_offset_);
}

EventStatus COMTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto prev_fsm_state =
      discrete_state->get_mutable_vector(prev_fsm_idx_).get_mutable_value();
  auto com_x_offset =
      discrete_state->get_mutable_vector(com_x_offset_idx_).get_mutable_value();
  auto initial_com =
      discrete_state->get_mutable_vector(initial_com_idx_).get_mutable_value();
  auto switch_time =
      discrete_state->get_mutable_vector(switch_time_idx_).get_mutable_value();

  const BasicVector<double>* fsm_output =
      this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  double timestamp = robot_output->get_timestamp();

  if (prev_fsm_state(0) != fsm_state(0)) {  // When to reset the clock
    prev_fsm_state(0) = fsm_state(0);
    VectorXd q = robot_output->GetPositions();
    plant_.SetPositions(context_, q);
    VectorXd center_of_mass = plant_.CalcCenterOfMassPosition(*context_);
    if (fsm_state(0) == BALANCE) {  // Either the simulator restarted or the
                                    // controller switch was triggered
      initial_com << center_of_mass;
      switch_time << timestamp;
    }
    com_x_offset(0) =
        kLandingOffset + (center_of_mass(0) - crouch_traj_.value(timestamp)(0));
    // TODO(yangwill) Remove this or calculate it based on the robot's state.
    // Actually, this is necessary due to the traj opt solution's placement
    // of the final CoM
  }
  if (initial_com == VectorXd::Zero(3)){
    VectorXd q = robot_output->GetPositions();
    plant_.SetPositions(context_, q);
    VectorXd center_of_mass = plant_.CalcCenterOfMassPosition(*context_);
    initial_com << center_of_mass;

  }
  return EventStatus::Succeeded();
}

drake::trajectories::PiecewisePolynomial<double>
COMTrajGenerator::generateBalanceTraj(
    const drake::systems::Context<double>& context, const Eigen::VectorXd& x,
    double time) const {
  plant_.SetPositionsAndVelocities(context_, x);

  double clock = this->EvalVectorInput(context, clock_port_)->get_value()(0);
  const auto& initial_com =
      context.get_discrete_state(initial_com_idx_).get_value();
  const double switch_time =
      context.get_discrete_state(switch_time_idx_).get_value()(0);

  // Calculate feet positions
  Vector3d contact_pos_sum = Vector3d::Zero();
  Vector3d position;
  for (const auto& point_and_frame : feet_contact_points_) {
    plant_.CalcPointsPositions(*context_, point_and_frame.second,
                               VectorXd::Zero(3), world_, &position);
    contact_pos_sum += position;
  }

  Vector3d target_com = crouch_traj_.value(time_offset_);
//  Vector3d curr_com = plant_.CalcCenterOfMassPosition(*context_);

  // generate a trajectory from current position to target position
  MatrixXd centerOfMassPoints(3, 2);
  centerOfMassPoints << initial_com, target_com + 0.5 * contact_pos_sum;

  VectorXd breaks_vector(2);
//  breaks_vector << switch_time,
//      switch_time + kTransitionSpeed * (curr_com - target_com).norm();
  breaks_vector << switch_time,
      switch_time + time_offset_;

  return PiecewisePolynomial<double>::CubicHermite(breaks_vector,
                                                     centerOfMassPoints, MatrixXd::Zero(3, 2));
}

drake::trajectories::PiecewisePolynomial<double>
COMTrajGenerator::generateCrouchTraj(const Eigen::VectorXd& x,
                                     double time) const {
  // This assumes that the crouch is starting at the exact position as the
  // start of the target trajectory which should be handled by balance
  // trajectory
  plant_.SetPositionsAndVelocities(context_, x);
  Vector3d contact_pos_sum = Vector3d::Zero();
  Vector3d position;
  for (const auto& point_and_frame : feet_contact_points_) {
    plant_.CalcPointsPositions(*context_, point_and_frame.second,
                               VectorXd::Zero(3), world_, &position);
    contact_pos_sum += position;
  }

  std::vector<double> breaks = crouch_traj_.get_segment_times();
  VectorXd breaks_vector = Eigen::Map<VectorXd>(breaks.data(), breaks.size());
  MatrixXd offset_matrix = 0.5 * contact_pos_sum.replicate(1, breaks.size());

  PiecewisePolynomial<double> com_offset =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks_vector, offset_matrix);

  return crouch_traj_ + com_offset;
}

drake::trajectories::PiecewisePolynomial<double>
COMTrajGenerator::generateLandingTraj(
    const drake::systems::Context<double>& context, const Eigen::VectorXd& x,
    double time) const {
  const auto& com_x_offset =
      context.get_discrete_state().get_vector(com_x_offset_idx_);

  // Only offset the x-position
  Vector3d offset(com_x_offset[0], 0, 0);

  auto traj_segment =
      crouch_traj_.slice(crouch_traj_.get_segment_index(time), 1);
  std::vector<double> breaks = traj_segment.get_segment_times();
  MatrixXd offset_matrix = offset.replicate(1, breaks.size());
  VectorXd breaks_vector = Eigen::Map<VectorXd>(breaks.data(), breaks.size());
  PiecewisePolynomial<double> com_offset =
      PiecewisePolynomial<double>::FirstOrderHold(breaks_vector, offset_matrix);
  return traj_segment + com_offset;
}

void COMTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  double time = robot_output->get_timestamp();

  // Read in finite state machine
  const auto& fsm_state =
      this->EvalVectorInput(context, fsm_port_)->get_value();

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  const drake::VectorX<double>& x = robot_output->GetState();

  if (fsm_state[0] == BALANCE)
    *casted_traj = generateBalanceTraj(context, x, time);
  else if (fsm_state[0] == CROUCH)
    *casted_traj = generateCrouchTraj(x, time);
  else if (fsm_state[0] == LAND)
    *casted_traj = generateLandingTraj(context, x, time);
}

}  // namespace dairlib::examples::osc_jump