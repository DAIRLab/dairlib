#pragma once

#include <string>

#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "multibody/multibody_utils.h"
#include "solvers/nonlinear_constraint.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"

namespace dairlib {
namespace goldilocks_models {
namespace find_models {

///
/// First version of dynamics constraint. We create a spline first for each
/// segment and then take second time derivatives and evaluate it at
/// beginning/end of the segment
///

class DynamicsConstraint : public solvers::NonlinearConstraint<double> {
 public:
  DynamicsConstraint(const ReducedOrderModel& rom,
                     const drake::multibody::MultibodyPlant<double>& plant,
                     bool is_head,
                     const std::string& description = "rom_dyn_constraint");

  // Getters
  // Use the methods with context as a argument to speed up computation
  Eigen::VectorXd GetY(const Eigen::VectorXd& q,
                       const drake::systems::Context<double>& context) const;
  Eigen::VectorXd GetYdot(const Eigen::VectorXd& x,
                          const drake::systems::Context<double>& context) const;
  Eigen::VectorXd GetY(const Eigen::VectorXd& q) const;
  Eigen::VectorXd GetYdot(const Eigen::VectorXd& x) const;
  Eigen::VectorXd GetYddot(const Eigen::VectorXd& y,
                           const Eigen::VectorXd& ydot,
                           const Eigen::VectorXd& tau) const;

  Eigen::MatrixXd getGradientWrtTheta(const Eigen::VectorXd& x_i_double,
                                      const Eigen::VectorXd& tau_i_double,
                                      const Eigen::VectorXd& x_iplus1_double,
                                      const Eigen::VectorXd& tau_iplus1_double,
                                      const Eigen::VectorXd& h_i_double) const;

  // Extend the model by assuming the parameters of the new dynamics row are 0's
  // the new dynamics row = tau.
  Eigen::VectorXd computeTauToExtendModel(
      const Eigen::VectorXd& x_i_double, const Eigen::VectorXd& x_iplus1_double,
      const Eigen::VectorXd& h_i, const Eigen::VectorXd& theta_y_append);

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override;
  Eigen::VectorXd EvalConstraintWithModelParams(
      const Eigen::VectorXd& x_i, const Eigen::VectorXd& tau_i,
      const Eigen::VectorXd& x_iplus1, const Eigen::VectorXd& tau_iplus1,
      const Eigen::VectorXd& h_i, const Eigen::VectorXd& theta_y,
      const Eigen::VectorXd& theta_yddot) const;

  std::unique_ptr<ReducedOrderModel> rom_;

  int n_q_;
  int n_v_;
  int n_u_;
  int n_tau_;

  bool is_head_;

  // Finite differencing to get gradient of feature wrt q
  // double eps_fd_feature_ = 1e-8;  // this is tuned. difference norm = 1e-8
  // double eps_cd_feature_ = 1e-5;  // this is tuned. difference norm = 1e-11
  // double eps_ho_feature_ = 1e-3;  // this is tuned. difference norm = 1e-12

  // Finite differencing to get gradient of constraints wrt theta
  double eps_fd_ = 1e-6;
  double eps_cd_ = 1e-4;
  double eps_ho_ = 1e-3;
  // The above number is tuned in getGradientWrtTheta(), and the result is:
  // 1e-6 good for fd
  // 1e-4 good for cd;  // B matrix error ~ 1e-13 to 1e-15
  // 1e-3 good for ho;
  std::vector<double> fd_shift_vec_{0, eps_fd_};  // forward difference
  std::vector<double> cd_shift_vec_{-eps_cd_ / 2,
                                    eps_cd_ / 2};  // central difference
  std::vector<double> ho_shift_vec_{-eps_ho_ / 2, -eps_ho_ / 4, eps_ho_ / 4,
                                    eps_ho_ / 2};

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
};

///
/// Second version of dynamics constraint. We don't create a spline in this
/// version. The constraint at knot i only depends on x_i and tau_i.
///

class DynamicsConstraintV2 : public solvers::NonlinearConstraint<double> {
 public:
  DynamicsConstraintV2(const ReducedOrderModel& rom,
                       const drake::multibody::MultibodyPlant<double>& plant,
                       DirconKinematicDataSet<double>* constraint,
                       const std::string& description = "rom_dyn_constraint");

  // Getters
  // Use the methods with context as a argument to speed up computation
  Eigen::VectorXd GetY(const Eigen::VectorXd& q,
                       const drake::systems::Context<double>& context) const;
  Eigen::VectorXd GetYdot(const Eigen::VectorXd& x,
                          const drake::systems::Context<double>& context) const;
  Eigen::VectorXd GetY(const Eigen::VectorXd& q) const;
  Eigen::VectorXd GetYdot(const Eigen::VectorXd& x) const;
  Eigen::VectorXd GetYddot(const Eigen::VectorXd& y,
                           const Eigen::VectorXd& ydot,
                           const Eigen::VectorXd& tau) const;
  void GetYYdotAndYddot(const Eigen::VectorXd& x_u_lambda_tau,
                        Eigen::VectorXd* y, Eigen::VectorXd* ydot,
                        Eigen::VectorXd* yddot) const;
  Eigen::VectorXd GetTau(const Eigen::VectorXd& x_u_lambda_tau) const;

  void ExtractInputVariables(const Eigen::VectorXd& x_u_lambda_tau,
                             Eigen::VectorXd* x, Eigen::VectorXd* u,
                             Eigen::VectorXd* lambda,
                             Eigen::VectorXd* tau) const;
  Eigen::MatrixXd getGradientWrtTheta(
      const Eigen::VectorXd& x_u_lambda_tau) const;

  // Extend the model by assuming the parameters of the new dynamics row are 0's
  // the new dynamics row = tau.
  Eigen::VectorXd computeTauToExtendModel(
      const Eigen::VectorXd& x_i_double, const Eigen::VectorXd& x_iplus1_double,
      const Eigen::VectorXd& h_i, const Eigen::VectorXd& theta_y_append);

 private:
  void EvaluateConstraint(
      const Eigen::Ref<const drake::VectorX<double>>& x_u_lambda_tau,
      drake::VectorX<double>* y) const override;
  Eigen::VectorXd EvalConstraintWithModelParams(
      const Eigen::VectorXd& x_u_lambda_tau, const Eigen::VectorXd& theta_y,
      const Eigen::VectorXd& theta_yddot) const;

  std::unique_ptr<ReducedOrderModel> rom_;

  int n_q_;
  int n_v_;
  int n_x_;
  int n_u_;
  int n_lambda_;
  int n_tau_;

  // Finite differencing to get gradient of feature wrt q
  // double eps_fd_feature_ = 1e-8;  // this is tuned. difference norm = 1e-8
  // double eps_cd_feature_ = 1e-5;  // this is tuned. difference norm = 1e-11
  // double eps_ho_feature_ = 1e-3;  // this is tuned. difference norm = 1e-12

  // Finite differencing to get gradient of constraints wrt theta
  double eps_fd_ = 1e-6;
  double eps_cd_ = 1e-4;
  double eps_ho_ = 1e-3;
  // The above number is tuned in getGradientWrtTheta(), and the result is:
  // 1e-6 good for fd
  // 1e-4 good for cd;  // B matrix error ~ 1e-13 to 1e-15
  // 1e-3 good for ho;
  std::vector<double> fd_shift_vec_{0, eps_fd_};  // forward difference
  std::vector<double> cd_shift_vec_{-eps_cd_ / 2,
                                    eps_cd_ / 2};  // central difference
  std::vector<double> ho_shift_vec_{-eps_ho_ / 2, -eps_ho_ / 4, eps_ho_ / 4,
                                    eps_ho_ / 2};

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;

  // Kinematic constraint data set
  DirconKinematicDataSet<double>* constraint_;
};

}  // namespace find_models
}  // namespace goldilocks_models
}  // namespace dairlib
