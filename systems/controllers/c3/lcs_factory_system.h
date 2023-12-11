#pragma once

#include <string>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>

#include "solvers/c3_options.h"
#include "solvers/lcs.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Outputs a lcmt_timestamped_saved_traj
class LCSFactorySystem : public drake::systems::LeafSystem<double> {
 public:
  explicit LCSFactorySystem(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      drake::systems::Context<drake::AutoDiffXd>* context_ad,
      const std::vector<drake::SortedPair<drake::geometry::GeometryId>>& contact_geoms,
      C3Options c3_options);

  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
    return this->get_input_port(lcs_state_input_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_lcs() const {
    return this->get_output_port(lcs_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_lcs_contact_jacobian() const {
    return this->get_output_port(lcs_contact_jacobian_port_);
  }

 private:
  void OutputLCS(const drake::systems::Context<double>& context,
                 solvers::LCS* output_traj) const;
  void OutputLCSContactJacobian(const drake::systems::Context<double>& context,
                 Eigen::MatrixXd* output_jacobian) const;

  drake::systems::InputPortIndex lcs_state_input_port_;
  drake::systems::OutputPortIndex actor_trajectory_port_;
  drake::systems::OutputPortIndex lcs_port_;
  drake::systems::OutputPortIndex lcs_contact_jacobian_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<drake::AutoDiffXd>* context_ad_;
  const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
      contact_pairs_;

  C3Options c3_options_;

  // convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;
  int N_;
  double dt_;
};

}  // namespace systems
}  // namespace dairlib
