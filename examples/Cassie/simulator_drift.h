#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

class SimulatorDrift : public drake::systems::LeafSystem<double> {
 public:
  SimulatorDrift(const RigidBodyTree<double>& tree,
                 const Eigen::VectorXd& drift_mean,
                 const Eigen::MatrixXd& drift_cov);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

 private:
  void CalcAdjustedState(const drake::systems::Context<double>& context,
                         dairlib::systems::OutputVector<double>* output) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  int time_idx_;
  int accumulated_drift_index_;
  const RigidBodyTree<double>& tree_;
  Eigen::VectorXd drift_mean_;
  Eigen::MatrixXd drift_cov_;
  int state_port_;
};
