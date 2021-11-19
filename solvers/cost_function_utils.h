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
    const drake::multibody::MultibodyPlant<T>& plant, double W);

template <typename T>
class PositiveMechanicalWork : public NonlinearCost<T> {
 public:
  PositiveMechanicalWork(int n_v, int n_u, const Eigen::MatrixXd& B,
                         double W, const std::string& description);

 private:
  void EvaluateCost(const Eigen::Ref<const drake::VectorX<T>>& x,
                    drake::VectorX<T>* y) const override;
  double W_;  //
  const Eigen::MatrixXd B_t_;
  int n_h_ = 2;
  int n_v_;
  int n_u_;
};

template <typename T>
class ElectricalLoss : public NonlinearCost<T> {
 public:
  ElectricalLoss(int n_v, int n_u, const Eigen::MatrixXd& B,
                         double W, const std::string& description);

 private:
  void EvaluateCost(const Eigen::Ref<const drake::VectorX<T>>& x,
                    drake::VectorX<T>* y) const override;
  double W_;  //
  const Eigen::MatrixXd B_t_;
  int n_h_ = 2;
  int n_v_;
  int n_u_;
};

}  // namespace solvers
}  // namespace dairlib
