#pragma once

#include <string>

#include "drake/common/drake_throw.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "multibody/multibody_utils.h"

#include "drake/common/drake_assert.h"
#include "examples/goldilocks_models/kinematics_expression.h"

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

class KinematicsConstraint : public Constraint {
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

};

}  // namespace goldilocks_models
}  // namespace dairlib
