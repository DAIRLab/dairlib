#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "multibody/multibody_utils.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

/// This class is the same as LIPMTrajGenerator except that the lipm is wrt
/// to stance foot (in xyz), and the reference frame's axes align with pelvis's
/// on the x-y plane (so we rotate the lipm position by R_pelvis_to_world)

class LocalLIPMTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  LocalLIPMTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context, double desired_com_height,
      const std::vector<int>& unordered_fsm_states,
      const std::vector<double>& unordered_state_durations,
      const std::vector<std::vector<std::pair<
          const Eigen::Vector3d, const drake::multibody::Frame<double>&>>>&
          contact_points_in_each_state,
      const std::vector<bool>& flip_in_y);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
      const {
    return this->get_input_port(fsm_switch_time_port_);
  }

  void SetGroundIncline(double ground_incline) {
    ground_incline_ = ground_incline;
  }

 private:
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  // Port indices
  int state_port_;
  int fsm_port_;
  int fsm_switch_time_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  double desired_com_height_;
  std::vector<int> unordered_fsm_states_;

  std::vector<double> unordered_state_durations_;

  // A list of pairs of contact body frame and contact point in each FSM state
  const std::vector<std::vector<std::pair<
      const Eigen::Vector3d, const drake::multibody::Frame<double>&>>>&
      contact_points_in_each_state_;

  const std::vector<bool>& flip_in_y_;

  const drake::multibody::BodyFrame<double>& world_;

  //
  double ground_incline_;
};

}  // namespace systems
}  // namespace dairlib
