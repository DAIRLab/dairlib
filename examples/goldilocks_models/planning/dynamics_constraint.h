#pragma once

#include <string>

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

class DynamicsConstraint : public solvers::NonlinearConstraint<double> {
 public:
  DynamicsConstraint(const ReducedOrderModel& rom,
                     const std::set<int>& idx_constant_rom_vel,
                     bool is_RL_training,
                     const std::string& description = "rom_dyn_constraint");

  // For RL training
  Eigen::MatrixXd GetGradientWrtTheta(const Eigen::VectorXd& x) const;

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override;

  // z = [y; ydot]
  // Calculate the dynamics for z
  drake::VectorX<double> g(const drake::VectorX<double>& z,
                           const drake::VectorX<double>& tau) const;

  const ReducedOrderModel& rom_;
  int n_y_;
  int n_z_;
  int n_tau_;

  const std::set<int>& idx_constant_rom_vel_;

  ///////////////////// RL training /////////////////////
  bool is_RL_training_;
  std::unique_ptr<ReducedOrderModel> mutable_rom_;
  drake::VectorX<double> EvaluateConstraintWithSpecifiedThetaYddot(
      const Eigen::Ref<const drake::VectorX<double>>& ztzth,
      const Eigen::VectorXd& theta_yddot) const;

  // Finite differencing to get gradient of constraints wrt theta
  // The eps's are tuned in model optimization trajopt
  //  double eps_fd_ = 1e-6;
  double eps_cd_ = 1e-4;
  //  double eps_ho_ = 1e-3;
  //  std::vector<double> fd_shift_vec_{0, eps_fd_};  // forward difference
  std::vector<double> cd_shift_vec_{-eps_cd_ / 2,
                                    eps_cd_ / 2};  // central difference
  //  std::vector<double> ho_shift_vec_{-eps_ho_ / 2, -eps_ho_ / 4, eps_ho_ / 4,
  //                                    eps_ho_ / 2};
  ///////////////////////////////////////////////////////
};
}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
