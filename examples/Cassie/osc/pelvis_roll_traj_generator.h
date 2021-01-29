#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::cassie::osc {

class PelvisRollTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  PelvisRollTrajGenerator(const drake::multibody::MultibodyPlant<double>& plant,
                          drake::systems::Context<double>* context,
                          const int hip_idx, const std::string& traj_name);

  // Input/output ports
  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }

 private:
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;

  int hip_idx_;
  int state_port_;
};
}  // namespace dairlib::cassie::osc
