#pragma once

#include <string>
#include <vector>

#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "solvers/c3.h"
#include "solvers/c3_miqp.h"
#include "solvers/c3_options.h"
#include "solvers/lcs.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Outputs a lcmt_timestamped_saved_traj
class C3Controller : public drake::systems::LeafSystem<double> {
 public:
  explicit C3Controller(solvers::LCS& lcs, C3Options c3_options,
                        std::vector<Eigen::MatrixXd> Q,
                        std::vector<Eigen::MatrixXd> R,
                        std::vector<Eigen::MatrixXd> G,
                        std::vector<Eigen::MatrixXd> U);

  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(target_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(lcs_state_input_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_trajectory() const {
    return this->get_output_port(trajectory_output_port_);
  }

 private:
  void OutputTrajectory(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  drake::systems::InputPortIndex target_input_port_;
  drake::systems::InputPortIndex lcs_state_input_port_;
  drake::systems::OutputPortIndex trajectory_output_port_;

  std::unique_ptr<solvers::C3MIQP> c3_;
  solvers::LCS lcs_;

  std::vector<Eigen::MatrixXd> Q_;
  std::vector<Eigen::MatrixXd> R_;
  std::vector<Eigen::MatrixXd> G_;
  std::vector<Eigen::MatrixXd> U_;
};

}  // namespace systems
}  // namespace dairlib
