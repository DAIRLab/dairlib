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

namespace dairlib::examples::Cassie::osc_jump {

COMTrajGenerator::COMTrajGenerator(
    const MultibodyPlant<double>& plant,
    const vector<pair<const Vector3d, const Frame<double>&>>&
        feet_contact_points,
    PiecewisePolynomial<double> crouch_traj, double time_offset)
    : plant_(plant),
      world_(plant_.world_frame()),
      feet_contact_points_(feet_contact_points),
      crouch_traj_(crouch_traj),
      time_offset_(time_offset) {
  this->set_name("com_traj");
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort("com_traj", traj_inst,
                                  &COMTrajGenerator::CalcTraj);
  com_x_offset_idx_ = this->DeclareDiscreteState(1);
  fsm_idx_ = this->DeclareDiscreteState(1);

  DeclarePerStepDiscreteUpdateEvent(&COMTrajGenerator::DiscreteVariableUpdate);
  crouch_traj_.shiftRight(time_offset_);
  context_ = plant_.CreateDefaultContext();
}

EventStatus COMTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto prev_fsm_state =
      discrete_state->get_mutable_vector(fsm_idx_).get_mutable_value();
  auto com_x_offset =
      discrete_state->get_mutable_vector(com_x_offset_idx_).get_mutable_value();

  const BasicVector<double>* fsm_output =
      this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  double timestamp = robot_output->get_timestamp();

  if (prev_fsm_state(0) != fsm_state(0)) {  // When to reset the clock
    prev_fsm_state(0) = fsm_state(0);

    VectorXd q = robot_output->GetPositions();
    plant_.SetPositions(context_.get(), q);
    VectorXd center_of_mass = plant_.CalcCenterOfMassPosition(*context_);
    com_x_offset(0) =
        kLandingOffset + (center_of_mass(0) - crouch_traj_.value(timestamp)(0));
    // TODO(yangwill) Remove this or calculate it based on the robot's state.
    // Actually, this is necessary due to the traj opt solution's placement
    // of the final CoM
  }
  return EventStatus::Succeeded();
}

drake::trajectories::PiecewisePolynomial<double>
COMTrajGenerator::generateBalanceTraj(
    const drake::systems::Context<double>& context, const Eigen::VectorXd& x,
    double time) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q = robot_output->GetPositions();
  plant_.SetPositions(context_.get(), q);

  Vector3d target_com = crouch_traj_.value(time_offset_);
  Vector3d curr_com = plant_.CalcCenterOfMassPosition(*context_);

  // generate a trajectory from current position to target position
  MatrixXd centerOfMassPoints(3, 2);
  centerOfMassPoints << curr_com, target_com;
  VectorXd breaks_vector(2);
  breaks_vector << time,
      time + kTransitionSpeed * (curr_com - target_com).norm();

  return PiecewisePolynomial<double>::FirstOrderHold(breaks_vector,
                                                     centerOfMassPoints);
}

drake::trajectories::PiecewisePolynomial<double>
COMTrajGenerator::generateCrouchTraj(
    const drake::systems::Context<double>& context, const Eigen::VectorXd& x,
    double time) const {
  // This assumes that the crouch is starting at the exact position as the
  // start of the target trajectory which should be handled by balance
  // trajectory

  return crouch_traj_;
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
    *casted_traj = generateCrouchTraj(context, x, time);
  else if (fsm_state[0] == LAND)
    *casted_traj = generateLandingTraj(context, x, time);
}

}  // namespace dairlib::examples::Cassie::osc_jump