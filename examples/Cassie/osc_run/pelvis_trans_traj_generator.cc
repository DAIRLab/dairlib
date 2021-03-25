#include "examples/Cassie/osc_run/pelvis_trans_traj_generator.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

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
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::examples::osc {

PelvisTransTrajGenerator::PelvisTransTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::trajectories::PiecewisePolynomial<double>& traj,
    const std::vector<std::pair<const Eigen::Vector3d,
                                const drake::multibody::Frame<double>&>>&
        feet_contact_points)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      traj_(traj),
      feet_contact_points_(feet_contact_points) {
  this->set_name("pelvis_trans_traj");
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort("pelvis_trans_traj", traj_inst,
                                  &PelvisTransTrajGenerator::CalcTraj);
  //  prev_fsm_idx_ = this->DeclareDiscreteState(init_fsm);

  DeclarePerStepDiscreteUpdateEvent(
      &PelvisTransTrajGenerator::DiscreteVariableUpdate);
}

EventStatus PelvisTransTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto prev_fsm_state =
      discrete_state->get_mutable_vector(prev_fsm_idx_).get_mutable_value();
  auto pelvis_x_offset =
      discrete_state->get_mutable_vector(pelvis_x_offset_idx_)
          .get_mutable_value();
  auto initial_pelvis_pos =
      discrete_state->get_mutable_vector(initial_pelvis_pos_idx_)
          .get_mutable_value();
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
    VectorXd pelvis_pos(3);
    plant_.CalcPointsPositions(*context_,
                               plant_.GetBodyByName("pelvis").body_frame(),
                               VectorXd::Zero(3), world_, &pelvis_pos);
    if (fsm_state(0) == BALANCE) {  // Either the simulator restarted or the
      // controller switch was triggered
      initial_pelvis_pos << pelvis_pos;
      switch_time << timestamp;
    }
    // Offset desired pelvis location based on final landing location
    pelvis_x_offset(0) = kLandingOffset;
  }
  if (initial_pelvis_pos == VectorXd::Zero(3)) {
    VectorXd q = robot_output->GetPositions();
    plant_.SetPositions(context_, q);
    VectorXd pelvis_pos(3);
    plant_.CalcPointsPositions(*context_,
                               plant_.GetBodyByName("pelvis").body_frame(),
                               VectorXd::Zero(3), world_, &pelvis_pos);
    initial_pelvis_pos << pelvis_pos;
  }
  return EventStatus::Succeeded();
}

void PelvisTransTrajGenerator::CalcTraj(
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

}  // namespace dairlib::examples::osc