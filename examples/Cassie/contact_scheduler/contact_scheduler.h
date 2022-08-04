#pragma once

#include <string>

#include "systems/framework/impact_info_vector.h"
#include "systems/framework/output_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

enum BLEND_FUNC { SIGMOID, EXP };

class ContactScheduler : drake::systems::LeafSystem<double> {
 public:
  ContactScheduler(
      const drake::multibody::MultibodyPlant<double>& plant,
      double near_impact_threshold = 0, double tau = 0.0025, BLEND_FUNC blend_func = SIGMOID);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_fsm() const {
    return this->get_output_port(fsm_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_clock() const {
    return this->get_output_port(clock_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_impact() const {
    return this->get_output_port(impact_info_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_contact_scheduler() const {
    return this->get_output_port(contact_scheduler_port_);
  }

 private:
  void CalcNextImpactInfo(const drake::systems::Context<double>& context,
                      systems::ImpactInfoVector<double>* near_impact) const;
  void CalcClock(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* clock) const;
  void CalcContactScheduler(const drake::systems::Context<double>& context,
                            drake::systems::BasicVector<double> *clock) const;

  drake::systems::InputPortIndex state_port_;
  drake::systems::OutputPortIndex fsm_port_;
  drake::systems::OutputPortIndex clock_port_;
//  drake::systems::OutputPortIndex contact_timing_;
  drake::systems::OutputPortIndex impact_info_port_;
  drake::systems::OutputPortIndex contact_scheduler_port_;


//  const drake::multibody::MultibodyPlant<double>& plant_;

  double t0_;
  std::vector<int> states_;
  std::vector<double> state_durations_;
  std::vector<double> accu_state_durations_;
  std::vector<int> impact_states_;
  std::vector<double> impact_times_;
  double period_;
  double tau_ = 0.0025;
  double near_impact_threshold_;
  BLEND_FUNC blend_func_;
};

}  // namespace dairlib
