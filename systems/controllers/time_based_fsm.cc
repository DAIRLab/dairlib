#include "systems/controllers/time_based_fsm.h"

using std::cout;
using std::endl;

using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::VectorXd;
using std::string;

namespace dairlib {
namespace systems {

TimeBasedFiniteStateMachine::TimeBasedFiniteStateMachine(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::vector<int>& states, const std::vector<double>& state_durations,
    double t0)
    : states_(states), state_durations_(state_durations), t0_(t0) {
  DRAKE_DEMAND(states.size() == state_durations.size());

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &TimeBasedFiniteStateMachine::CalcFiniteState);

  // Accumulate the durations to get timestamps
  double sum = 0;
  for (auto& duration_i : state_durations) {
    sum += duration_i;
    accu_state_durations_.push_back(sum);
  }
  period_ = sum;
}

typedef std::numeric_limits<double> dbl;


void TimeBasedFiniteStateMachine::CalcFiniteState(
    const Context<double>& context, BasicVector<double>* fsm_state) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto current_sim_time = static_cast<double>(robot_output->get_timestamp());

  double m = floor((current_sim_time - t0_) / period_);
  double remainder = (current_sim_time - t0_) - m * period_;

  double eps = 0;//1e-12;
//  cout.precision(dbl::max_digits10);
  cout << "================================== start of FSM ====================================\n";
  cout << "(output time, context time) = " << current_sim_time << ", "<<  context.get_time() << endl;

  // Get current finite state
  VectorXd current_finite_state(1);
  if (current_sim_time >= t0_) {

    cout << "states_ = ";
    for (auto mem : states_) {
      cout << mem << ", ";
    }
    cout << "\naccu_state_durations_ = ";
    for (unsigned int i = 0; i < accu_state_durations_.size(); i++) {
      cout << accu_state_durations_[i] << " ("<< accu_state_durations_[i] - remainder<<"), ";
      if (remainder < accu_state_durations_[i] - eps) {
        current_finite_state << states_[i];
        cout << "\n";
        cout << "in if statement: remainder = " << remainder << endl;
        cout << "in if statement: current_finite_state = " << current_finite_state << endl;
        break;
      }
    }
  } else {
    current_finite_state << initial_state_;
  }

  // Assign fsm_state
  fsm_state->get_mutable_value() = current_finite_state;
}

}  // namespace systems
}  // namespace dairlib
