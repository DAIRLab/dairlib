#include "examples/goldilocks_models/controller/optimal_rom_planner.h"

#include <math.h>
#include <string>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

using dairlib::systems::OutputVector;

namespace dairlib {
namespace goldilocks_models {

OptimalRomPlanner::OptimalRomPlanner(
    const MultibodyPlant<double>& plant, double desired_com_height,
    const vector<int>& unordered_fsm_states,
    const vector<double>& unordered_state_durations,
    const vector<vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>>&
        contact_points_in_each_state)
    : plant_(plant),
      desired_com_height_(desired_com_height),
      unordered_fsm_states_(unordered_fsm_states),
      unordered_state_durations_(unordered_state_durations),
      contact_points_in_each_state_(contact_points_in_each_state),
      world_(plant_.world_frame()) {
  this->set_name("lipm_traj");

  // Checking vector dimension
  DRAKE_DEMAND(unordered_fsm_states.size() == unordered_state_durations.size());
  DRAKE_DEMAND(unordered_fsm_states.size() ==
               contact_points_in_each_state.size());

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp(VectorXd::Zero(0));
  drake::trajectories::Trajectory<double>& traj_instance = pp;
  this->DeclareAbstractOutputPort("lipm_traj", traj_instance,
                                  &OptimalRomPlanner::CalcTraj);

  // Discrete state event
  DeclarePerStepDiscreteUpdateEvent(&OptimalRomPlanner::DiscreteVariableUpdate);
  // The time of the last touch down
  prev_td_time_idx_ = this->DeclareDiscreteState(1);
  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));

  // Create context
  context_ = plant_.CreateDefaultContext();
}

EventStatus OptimalRomPlanner::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  auto prev_td_time =
      discrete_state->get_mutable_vector(prev_td_time_idx_).get_mutable_value();
  auto prev_fsm_state = discrete_state->get_mutable_vector(prev_fsm_state_idx_)
                            .get_mutable_value();

  if (fsm_state(0) != prev_fsm_state(0)) {  // if at touchdown
    prev_fsm_state(0) = fsm_state(0);

    // Get time
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
    double timestamp = robot_output->get_timestamp();
    double current_time = static_cast<double>(timestamp);
    prev_td_time(0) = current_time;
  }

  return EventStatus::Succeeded();
}

void OptimalRomPlanner::CalcTraj(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd v = robot_output->GetVelocities();

  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  // Find fsm_state in unordered_fsm_states_
  auto it = find(unordered_fsm_states_.begin(), unordered_fsm_states_.end(),
                 int(fsm_state(0)));
  int mode_index = std::distance(unordered_fsm_states_.begin(), it);
  if (it == unordered_fsm_states_.end()) {
    cout << "WARNING: fsm state number " << fsm_state(0)
         << " doesn't exist in OptimalRomPlanner\n";
    mode_index = 0;
  }

  // Get discrete states
  const auto prev_td_time =
      context.get_discrete_state(prev_td_time_idx_).get_value();

  // Get time
  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);

  VectorXd q = robot_output->GetPositions();
  plant_.SetPositions(context_.get(), q);

  // Assign traj
  auto pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  // TODO: implement this
  *pp_traj = PiecewisePolynomial<double>(Vector3d::Zero());
}

}  // namespace goldilocks_models
}  // namespace dairlib
