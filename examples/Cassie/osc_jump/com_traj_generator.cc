#include "com_traj_generator.h"
#include <boost/signals2/detail/signal_template.hpp>
#include "multibody/multibody_utils.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

using dairlib::multibody::createContext;
using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
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

COMTrajGenerator::COMTrajGenerator(const MultibodyPlant<double>& plant,
                                   int hip_idx, int left_foot_idx,
                                   int right_foot_idx,
                                   PiecewisePolynomial<double> crouch_traj,
                                   double height)
    : plant_(plant),
      hip_idx_(hip_idx),
      left_foot_idx_(left_foot_idx),
      right_foot_idx_(right_foot_idx),
      crouch_traj_(crouch_traj),
      height_(height) {
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
  time_idx_ = this->DeclareDiscreteState(1);
  com_x_offset_idx_ = this->DeclareDiscreteState(1);
  fsm_idx_ = this->DeclareDiscreteState(1);

  DeclarePerStepDiscreteUpdateEvent(&COMTrajGenerator::DiscreteVariableUpdate);
}

EventStatus COMTrajGenerator::DiscreteVariableUpdate(
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
  auto current_time = static_cast<double>(timestamp);

  if (prev_fsm_state(0) != fsm_state(0)) {  // When to reset the clock
    prev_fsm_state(0) = fsm_state(0);
    prev_time(0) = current_time;
    VectorXd q = robot_output->GetPositions();
    VectorXd v = robot_output->GetVelocities();

    auto plant_context = createContext(plant_, q, v);
    // Return the x diff between the desired and current COM pos
    com_x_offset(0) = plant_.CalcCenterOfMassPosition(*plant_context)(0) -
                      crouch_traj_.value(crouch_traj_.end_time())(0);
  }
  return EventStatus::Succeeded();
}

PiecewisePolynomial<double> COMTrajGenerator::generateBalanceTraj(
    const drake::systems::Context<double>& context, VectorXd& q,
    VectorXd& v) const {

}

PiecewisePolynomial<double> COMTrajGenerator::generateCrouchTraj(
    const drake::systems::Context<double>& context, VectorXd& q,
    VectorXd& v) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double t = robot_output->get_timestamp();

  return crouch_traj_.slice(crouch_traj_.get_segment_index(t), 1);
}

PiecewisePolynomial<double> COMTrajGenerator::generateLandingTraj(
    const drake::systems::Context<double>& context, VectorXd& q,
    VectorXd& v) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double t = robot_output->get_timestamp();
  const VectorXd com_x_offset =
      context.get_discrete_state().get_vector(com_x_offset_idx_).get_value();

  // Only offset the x-position
  Vector3d offset;
  offset << com_x_offset(0), 0, 0;

  auto traj_segment = crouch_traj_.slice(crouch_traj_.get_segment_index(t), 1);
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
    case (BALANCE):  //  BALANCE
      *casted_traj = generateBalanceTraj(context, q, v);
      break;
    case (CROUCH):  //  CROUCH
      *casted_traj = generateCrouchTraj(context, q, v);
      break;
    case (LAND):  //  LAND
      *casted_traj = generateLandingTraj(context, q, v);
      break;
  }
}

}  // namespace dairlib::examples::Cassie::osc_jump