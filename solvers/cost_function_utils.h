#pragma once

#include "solvers/nonlinear_cost.h"
#include "systems/trajectory_optimization/dircon/dircon.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace dairlib {

namespace solvers {

template <typename T>
void AddPositiveWorkCost(
    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
    drake::multibody::MultibodyPlant<T>& plant);

template <typename T>
class PositiveMechanicalWork : public NonlinearCost<T> {
 public:
  PositiveMechanicalWork(
      const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& h_i,
      const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& h_ip1,
      const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& v_i,
      const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& v_ip1,
      const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& u_i,
      const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& u_ip1,
      const Eigen::MatrixXd& B, const double W, const std::string& description);

 private:
  void EvaluateCost(const Eigen::Ref<const drake::VectorX<T>>& x,
                    drake::VectorX<T>* y) const override;
  const drake::multibody::MultibodyPlant<double>& plant_;
  const Eigen::MatrixXd B_t_;
  double W_;  //
  int n_h_ = 2;
  int n_v_;
  int n_u_;
};
}  // namespace solvers
}  // namespace dairlib
