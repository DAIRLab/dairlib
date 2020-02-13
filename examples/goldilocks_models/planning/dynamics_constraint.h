#pragma once

#include <string>

#include "drake/common/drake_throw.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "multibody/multibody_utils.h"

#include "drake/common/drake_assert.h"
#include "examples/goldilocks_models/kinematics_expression.h"
#include "examples/goldilocks_models/dynamics_expression.h"

using std::map;
using std::string;
using std::vector;
using std::list;
using std::unique_ptr;
using std::make_unique;
using std::make_shared;
using std::isnan;
using std::isinf;

using Eigen::Dynamic;
using Eigen::AutoDiffScalar;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix;
using Eigen::MatrixXd;
using drake::VectorX;
using drake::MatrixX;
using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::math::DiscardGradient;
using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::math::initializeAutoDiff;
using drake::solvers::to_string;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::Constraint;
using drake::solvers::VariableRefList;
using drake::solvers::Binding;
using drake::symbolic::Variable;
using drake::symbolic::Expression;
using drake::multibody::MultibodyPlant;


namespace dairlib {
namespace goldilocks_models {
namespace planning {


class DynamicsConstraint : public Constraint {
 public:
  DynamicsConstraint(int n_r, int n_ddr, int n_feature_dyn,
                     const VectorXd & theta_dyn,
                     int n_tau,
                     MatrixXd B_tau,
                     int robot_option,
                     const std::string& description = "");
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& ytyth,
              drake::AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<Variable>>& q,
              VectorX<Expression>*y) const override;

 private:
  AutoDiffVecXd getConstraintValueInAutoDiff(
    const AutoDiffVecXd & y_i, const AutoDiffVecXd & tau_i,
    const AutoDiffVecXd & y_iplus1, const AutoDiffVecXd & tau_iplus1,
    const AutoDiffVecXd & h_i) const;

  // y = [r; dr]
  // Calculate the dynamics for y
  AutoDiffVecXd g(const AutoDiffVecXd & y, const AutoDiffVecXd & tau) const;

  int n_r_;
  int n_ddr_;
  int n_feature_dyn_;
  int n_theta_dyn_;
  VectorXd theta_dyn_;
  int n_y_;
  int n_tau_;
  DynamicsExpression dyn_expression_;

};
}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
