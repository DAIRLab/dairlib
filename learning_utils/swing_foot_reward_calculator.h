/// Utility system to calculate the running reward for swing foot tracking

#include "drake/systems/framework/leaf_system.h"

namespace dairlib::learning {

class SwingFootRewardCalculator : public drake::systems::LeafSystem<double> {
 public:
  SwingFootRewardCalculator(const Eigen::Matrix3d& W_swing_foot_tracking,
                            const Eigen::Matrix3d& W_swing_foot_smoothness,
                            const std::vector<int>& single_support_fsm_states,
                            const std::string& swing_foot_traj_name);

  const drake::systems::InputPort<double>& get_osc_debug_input_port(){
    return this->get_input_port(osc_debug_input_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() {
    return this->get_input_port(fsm_input_port_);
  }
  const drake::systems::OutputPort<double>& get_reward_output_port() {
    return this->get_output_port(reward_output_port_);
  }

 private:
  drake::systems::EventStatus StateUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void CopyReward(const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* reward) const;

  drake::systems::InputPortIndex osc_debug_input_port_;
  drake::systems::InputPortIndex  fsm_input_port_;
  drake::systems::OutputPortIndex reward_output_port_;

  drake::systems::DiscreteStateIndex running_reward_idx_;
  drake::systems::DiscreteStateIndex prev_fsm_state_idx_;
  drake::systems::DiscreteStateIndex prev_time_idx_;

  const Eigen::Matrix3d& W_swing_foot_tracking_;
  const Eigen::Matrix3d& W_swing_foot_smoothness_;
  const std::vector<int>& single_support_fsm_states_;
  std::string traj_name_;
};
}
