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
// #include "examples/goldilocks_models/attic/kinematics_expression.h"

namespace dairlib {
namespace goldilocks_models {

/*class KinematicsConstraint : public Constraint {
 public:
  KinematicsConstraint(int n_s, int n_feature, VectorXd & theta_s,
                       const MultibodyPlant<AutoDiffXd> * plant,
                       int robot_option,
                       const std::string& description = "");
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& s_q,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& s_q,
              drake::AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<Variable>>& q,
              VectorX<Expression>*y) const override;

  VectorXd getGradientWrtTheta(const VectorXd & q);

  AutoDiffVecXd getKinematicsConstraint( const AutoDiffVecXd & s,
    const AutoDiffVecXd & q, const VectorXd & theta) const;
  VectorXd getKinematicsConstraint( const VectorXd & s,
    const VectorXd & q, const VectorXd & theta) const;

  KinematicsExpression<double> expression_double;
  KinematicsExpression<AutoDiffXd> expression_autoDiff;

 private:
  const MultibodyPlant<AutoDiffXd> * plant_;
  int n_constraint_;
  int n_feature_;
  int n_q_;
  VectorXd theta_s_;

};*/

///
/// ConstantKinematicsConstraint is for testing
///

class ConstKinematicsConstraint : public solvers::NonlinearConstraint<double> {
 public:
  ConstKinematicsConstraint(
      const ReducedOrderModel& rom,
      const drake::multibody::MultibodyPlant<double>& plant,
      const Eigen::VectorXd& rom_val, bool include_rom_vel,
      const std::vector<int>& active_dim,
      const std::string& description = "const_rom_kin_constraint");

  // Getters
  // Use the methods with context as a argument to speed up computation
  Eigen::VectorXd GetY(const Eigen::VectorXd& q,
                       const drake::systems::Context<double>& context) const;
  Eigen::VectorXd GetYdot(const Eigen::VectorXd& x,
                          const drake::systems::Context<double>& context) const;
  Eigen::VectorXd GetY(const Eigen::VectorXd& q) const;
  Eigen::VectorXd GetYdot(const Eigen::VectorXd& x) const;
  //  Eigen::MatrixXd getGradientWrtTheta(
  //      const Eigen::VectorXd& x_u_lambda_tau) const;

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* value) const override;

  std::unique_ptr<ReducedOrderModel> rom_;

  int n_q_;
  int n_v_;
  int n_x_;
  int n_y_;
  int n_output_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;

  Eigen::VectorXd rom_val_;
  bool include_rom_vel_;

  std::vector<int> active_dim_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
