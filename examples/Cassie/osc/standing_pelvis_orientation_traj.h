#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::cassie::osc {

class StandingPelvisOrientationTraj
    : public drake::systems::LeafSystem<double> {
 public:
  StandingPelvisOrientationTraj(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const std::vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>&
          feet_contact_points,
      const std::string& traj_name);
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

 private:
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;

  // A list of pairs of contact body frame and contact point
  const std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
      feet_contact_points_;
  int state_port_;
  int radio_port_;
};

}  // namespace dairlib::cassie::osc
