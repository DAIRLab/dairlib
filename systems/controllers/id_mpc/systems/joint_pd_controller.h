#pragma once

#include "systems/framework/timestamped_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

#include "systems/controllers/id_mpc/core/solution_trajectories.h"


namespace dairlib::systems::controllers::id_mpc {

class JointPDController : public drake::systems::LeafSystem<double> {
 public:
  explicit JointPDController(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::unordered_map<std::string, double>& kp,
      const std::unordered_map<std::string, double>& kd);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return get_input_port(input_port_state_);
  }
  const drake::systems::InputPort<double>& get_input_port_lcm_traj() const {
    return get_input_port(input_port_lcm_traj_);
  }

 private:

  void CalcTrajs(const drake::systems::Context<double>& context,
                 SolutionTraj* traj) const;

  void CalcTorques(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* u) const;

  Eigen::MatrixXd Kp_;
  Eigen::MatrixXd Kd_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_lcm_traj_;

  drake::systems::CacheIndex trajectory_cache_;

};

}