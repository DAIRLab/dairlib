#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "systems/framework/output_vector.h"

typedef std::pair<const Eigen::Vector3d&,
                  const drake::multibody::Frame<double>&> PointOnFramed;

namespace dairlib::systems {
class AlipS2SPlanner : public drake::systems::LeafSystem<double> {
 public:
  AlipS2SPlanner(const drake::multibody::MultibodyPlant<double>& plant,
                 drake::systems::Context<double>* context,
                 std::vector<int> left_right_support_fsm_states,
                 std::vector<double> left_right_support_durations,
                 std::vector<PointOnFramed> left_right_foot,
                 double double_support_duration,
                 double nominal_foot_y);

 private:

  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  // drake input ports
  drake::systems::InputPortIndex state_input_port_;
  drake::systems::InputPortIndex vdes_input_port_;
  drake::systems::InputPortIndex stepping_stone_input_port_;

  // drake output ports
  drake::systems::OutputPortIndex fsm_output_port_;
  drake::systems::OutputPortIndex time_to_impact_output_port_;
  drake::systems::OutputPortIndex time_since_impact_output_port_;
  drake::systems::OutputPortIndex footstep_target_output_port_;
  drake::systems::OutputPortIndex com_traj_output_port_;

  // multibody objects
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;
  std::map<int, PointOnFramed> stance_foot_map_;

  std::vector<int> left_right_support_fsm_states_;
  double double_support_duration_;

  // Parameters
  double m_;
  const double nominal_foot_y_;

  // Maps
  std::map<int, double> duration_map_;
};
}