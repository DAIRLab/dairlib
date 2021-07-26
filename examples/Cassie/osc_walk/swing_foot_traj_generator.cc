#include "swing_foot_traj_generator.h"

#include "multibody/multibody_utils.h"

using std::cout;
using std::endl;

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;

using dairlib::systems::OutputVector;
using drake::multibody::BodyFrame;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::examples::osc_walk {

SwingFootTrajGenerator::SwingFootTrajGenerator(
    const MultibodyPlant<double>& plant, Context<double>* context,
    const string& stance_foot_name, bool isLeftFoot,
    const PiecewisePolynomial<double>& foot_traj, double time_offset)
    : plant_(plant),
      context_(context),
      world_(plant.world_frame()),
      stance_foot_frame_(plant.GetFrameByName(stance_foot_name)),
      foot_traj_(foot_traj),
      time_offset_(time_offset) {
  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort("x",OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort("fsm",BasicVector<double>(1)).get_index();

  fsm_idx_ = this->DeclareDiscreteState(1);
  time_shift_idx_ = this->DeclareDiscreteState(1);
  stance_foot_pos_idx_ = this->DeclareDiscreteState(3);

  if (isLeftFoot) {
    this->set_name("l_foot_traj");
    this->DeclareAbstractOutputPort("l_foot_traj", traj_inst,
                                    &SwingFootTrajGenerator::CalcTraj);
    active_state_ = RIGHT_STANCE;
  } else {
    this->set_name("r_foot_traj");
    this->DeclareAbstractOutputPort("r_foot_traj", traj_inst,
                                    &SwingFootTrajGenerator::CalcTraj);
    active_state_ = LEFT_STANCE;
  }
  // Shift trajectory by time_offset
//  foot_traj_.shiftRight(time_offset_);

  // TODO(yangwill) add this shift elsewhere to make the gait periodic
  DeclarePerStepDiscreteUpdateEvent(
      &SwingFootTrajGenerator::DiscreteVariableUpdate);
}

EventStatus SwingFootTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto prev_fsm_state =
      discrete_state->get_mutable_vector(fsm_idx_).get_mutable_value();
  auto time_shift =
      discrete_state->get_mutable_vector(time_shift_idx_).get_mutable_value();
  auto offset =
      discrete_state->get_mutable_vector(stance_foot_pos_idx_).get_mutable_value();

  const BasicVector<double>* fsm_output =
      this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  double timestamp = robot_output->get_timestamp();

  if (prev_fsm_state(0) != fsm_state(0)) {  // When to reset the clock
    prev_fsm_state(0) = fsm_state(0);

    // A cycle has been reached
    if (fsm_state(0) == LEFT_STANCE) {
      time_shift << timestamp + time_offset_;
      plant_.SetPositionsAndVelocities(context_, robot_output->GetState());
      Vector3d stance_foot_pos = Vector3d::Zero();
      plant_.CalcPointsPositions(*context_, stance_foot_frame_, Vector3d::Zero(), world_,
                                 &stance_foot_pos);
      offset << stance_foot_pos;
    }
  }
  return EventStatus::Succeeded();
}

/*
  Move the swing foot relative to the stance foot. The stance foot is fixed so
  should be a good reference point
*/
PiecewisePolynomial<double> SwingFootTrajGenerator::generateFootTraj(
    const drake::systems::Context<double>& context, const VectorXd& x,
    double t) const {
  auto stance_foot_pos =
      context.get_discrete_state(stance_foot_pos_idx_).get_value();

  // offset stuff
  std::vector<double> breaks = foot_traj_.get_segment_times();
  VectorXd breaks_vector = Map<VectorXd>(breaks.data(), breaks.size());
  MatrixXd stance_foot = stance_foot_pos.replicate(1, breaks.size());
  PiecewisePolynomial<double> stance_foot_offset =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks_vector, stance_foot);

//  return foot_traj_ + stance_foot_offset;
  return foot_traj_;
}

void SwingFootTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  auto time_shift = context.get_discrete_state(time_shift_idx_).get_value();
  VectorXd x = robot_output->GetState();
  double timestamp = robot_output->get_timestamp();

  // Read in finite state machine
  const auto fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value();

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (fsm_state(0) == active_state_) {
    *casted_traj =
        generateFootTraj(context, robot_output->GetState(), timestamp);
  } else {
    // Do nothing to avoid bugs, maybe return a zero trajectory?
  }
//  casted_traj->shiftRight(time_shift(0));
}

}  // namespace dairlib::examples::osc_walk
