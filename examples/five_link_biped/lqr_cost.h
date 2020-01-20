#pragma once

#include "drake/common/trajectories/trajectory.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

#include "systems/framework/output_vector.h"

namespace dairlib {

class LQRCost : public drake::systems::LeafSystem<double> {
 public:
  LQRCost(
      const drake::multibody::MultibodyPlant<double>& plant,
      const Eigen::MatrixXd& q, const Eigen::MatrixXd& r,
      const std::vector<
          std::shared_ptr<drake::trajectories::Trajectory<double>>>& state_traj,
      const std::vector<
          std::shared_ptr<drake::trajectories::Trajectory<double>>>&
          input_traj);
  //  void get_output_port(int) = delete;
 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
  void CalcCost(const drake::systems::Context<double>& context,
                systems::BasicVector<double>* control) const;

  int fsm_port_;
  int cost_idx_;
  int time_idx_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  const std::vector<std::shared_ptr<drake::trajectories::Trajectory<double>>>
      state_traj_;
  const std::vector<std::shared_ptr<drake::trajectories::Trajectory<double>>>
      input_traj_;
};

}  // namespace dairlib