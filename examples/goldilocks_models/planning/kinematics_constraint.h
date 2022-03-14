#pragma once

#include <string>

#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "multibody/multibody_utils.h"
#include "solvers/nonlinear_constraint.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"

namespace dairlib {
namespace goldilocks_models {
namespace planning {

class KinematicsConstraint : public solvers::NonlinearConstraint<double> {
 public:
  KinematicsConstraint(
      const ReducedOrderModel& rom,
      const drake::multibody::MultibodyPlant<double>& plant, bool left_stance,
      const StateMirror& state_mirror, const std::set<int>& relax_index,
      const std::vector<int>& initialize_with_rom_state,
      const std::string& description = "rom_fom_mapping_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& zxeps,
                          drake::VectorX<double>* output) const override;

  const ReducedOrderModel& rom_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  bool left_stance_;
  const StateMirror& state_mirror_;
  std::set<int> relax_index_;
  std::vector<int> complement_of_initialize_with_rom_state_;
  std::vector<int> initialize_with_rom_state_;
  int n_eps_;
  int n_y_;
  int n_z_;
  int n_q_;
  int n_v_;
  int n_x_;
};
}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
