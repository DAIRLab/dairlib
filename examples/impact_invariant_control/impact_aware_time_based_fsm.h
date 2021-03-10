#pragma once

#include <string>

#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/output_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

enum BLEND_FUNC {SIGMOID, EXP};

class ImpactTimeBasedFiniteStateMachine
    : public systems::TimeBasedFiniteStateMachine {
 public:
  ImpactTimeBasedFiniteStateMachine(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<int>& states,
      const std::vector<double>& state_durations, double t0 = 0,
      double near_impact_threshold = 0,
      BLEND_FUNC blend_func = SIGMOID);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_fsm() const {
    return this->get_output_port(fsm_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_impact() const {
    return this->get_output_port(near_impact_port_);
  }

 private:
  void CalcNearImpact(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* near_impact) const;

  int near_impact_port_;

  std::vector<int> states_;
  std::vector<double> state_durations_;
  double t0_;

  std::vector<double> accu_state_durations_;
  std::vector<double> impact_states_;
  std::vector<double> impact_times_;
  double period_;
  double tau_ = 0.0025;
  double near_impact_threshold_;
  BLEND_FUNC blend_func_;
};

}  // namespace dairlib
