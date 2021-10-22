#include "examples/goldilocks_models/find_models/kinematics_constraint.h"

using std::cout;
using std::endl;
using std::isinf;
using std::isnan;
using std::list;
using std::make_shared;
using std::make_unique;
using std::map;
using std::string;
using std::unique_ptr;
using std::vector;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::VectorX;
using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using drake::math::DiscardGradient;
using drake::math::initializeAutoDiff;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::to_string;
using drake::solvers::VariableRefList;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;
using Eigen::AutoDiffScalar;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib {
namespace goldilocks_models {

/*KinematicsConstraint::KinematicsConstraint(
                                 int n_s, int n_feature, VectorXd & theta_s,
                                 const MultibodyPlant<AutoDiffXd> * plant,
                                 int robot_option,
                                 const std::string& description):
  Constraint(n_s,
             n_s + plant->num_positions(),
             VectorXd::Zero(n_s),
             VectorXd::Zero(n_s),
             description),
  expression_double(KinematicsExpression<double>(n_s, n_feature, robot_option)),
  expression_autoDiff(KinematicsExpression<AutoDiffXd>(n_s, n_feature,
robot_option)), plant_(plant), n_constraint_(n_s), n_feature_(n_feature),
  n_q_(plant->num_positions()),
  theta_s_(theta_s) {

  // Check the theta size
  DRAKE_DEMAND(n_s * n_feature == theta_s.size());

  // Check the feature size implemented in the model expression
  VectorXd q_temp = VectorXd::Ones(plant->num_positions());
  DRAKE_DEMAND(n_feature == expression_double.getFeature(q_temp).size());
}

void KinematicsConstraint::DoEval(const
                             Eigen::Ref<const Eigen::VectorXd>& s_q,
                             Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(s_q), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

void KinematicsConstraint::DoEval(const
                             Eigen::Ref<const AutoDiffVecXd>& s_q,
                             AutoDiffVecXd* y) const {
  const AutoDiffVecXd s = s_q.head(n_constraint_);
  const AutoDiffVecXd q = s_q.tail(n_q_);

  *y = getKinematicsConstraint(s, q, theta_s_);
}

void KinematicsConstraint::DoEval(const
                             Eigen::Ref<const VectorX<Variable>>& q,
                             VectorX<Expression>*y) const {
  throw std::logic_error(
    "This constraint class does not support symbolic evaluation.");
}

VectorXd KinematicsConstraint::getGradientWrtTheta(const VectorXd & q){
  VectorXd s = VectorXd::Zero(n_constraint_);
  VectorXd gradient(n_feature_);
  for(int i = 0; i<n_feature_; i++){
    VectorXd theta_unit = VectorXd::Zero(theta_s_.size());
    theta_unit(i) = 1;
    gradient(i) = getKinematicsConstraint(s,q,theta_unit)(0);
  }
  return gradient;
}

AutoDiffVecXd KinematicsConstraint::getKinematicsConstraint(
  const AutoDiffVecXd & s, const AutoDiffVecXd & q, const VectorXd & theta)
const{ return s - expression_autoDiff.getExpression(theta, q);
}
VectorXd KinematicsConstraint::getKinematicsConstraint(
  const VectorXd & s, const VectorXd & q, const VectorXd & theta) const{
  return s - expression_double.getExpression(theta, q);
}*/

///
/// ConstantKinematicsConstraint is for testing
///

ConstKinematicsConstraint::ConstKinematicsConstraint(
    const ReducedOrderModel& rom, const MultibodyPlant<double>& plant,
    const VectorXd& rom_val, bool include_rom_vel,
    const std::vector<int>& active_dim, const std::string& description)
    : NonlinearConstraint<double>(
          include_rom_vel ? 2 * active_dim.size() : active_dim.size(),
          include_rom_vel ? plant.num_positions() + plant.num_velocities()
                          : plant.num_positions(),
          VectorXd::Zero(include_rom_vel ? 2 * active_dim.size()
                                         : active_dim.size()),
          VectorXd::Zero(include_rom_vel ? 2 * active_dim.size()
                                         : active_dim.size()),
          description),
      rom_(rom.Clone()),
      n_q_(plant.num_positions()),
      n_v_(plant.num_velocities()),
      n_x_(n_q_ + n_v_),
      n_y_(rom.n_y()),
      n_output_(include_rom_vel ? 2 * active_dim.size() : active_dim.size()),
      plant_(plant),
      context_(plant.CreateDefaultContext()),
      rom_val_(rom_val),
      include_rom_vel_(include_rom_vel),
      active_dim_(active_dim) {
  DRAKE_DEMAND(rom.n_y() == rom.n_yddot());

  if (include_rom_vel_)
    DRAKE_DEMAND(rom_val.size() == 2 * rom.n_yddot());
  else
    DRAKE_DEMAND(rom_val.size() == rom.n_yddot());
}

// Getters
VectorXd ConstKinematicsConstraint::GetY(
    const VectorXd& q, const drake::systems::Context<double>& context) const {
  return rom_->EvalMappingFunc(q, context);
}
VectorXd ConstKinematicsConstraint::GetYdot(
    const VectorXd& x, const drake::systems::Context<double>& context) const {
  return rom_->EvalMappingFuncJV(x.head(n_q_), x.tail(n_v_), context);
}

void ConstKinematicsConstraint::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<double>>& x,
    drake::VectorX<double>* value) const {
  // Set context
  if (include_rom_vel_)
    multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, x,
                                                      context_.get());
  else
    multibody::SetPositionsIfNew<double>(plant_, x, context_.get());

  // Full output
  VectorX<double> full_output(rom_val_.size());
  full_output.head(n_y_) = rom_val_.head(n_y_) - GetY(x.head(n_q_), *context_);
  if (include_rom_vel_)
    full_output.tail(n_y_) = rom_val_.tail(n_y_) - GetYdot(x, *context_);

  // Get constraint value
  *value = VectorX<double>(n_output_);
  for (int i = 0; i < active_dim_.size(); i++) {
    value->segment<1>(i) = full_output.segment<1>(active_dim_.at(i));
    if (include_rom_vel_)
      value->segment<1>(i + active_dim_.size()) =
          full_output.segment<1>(active_dim_.at(i) + n_y_);
  }
}

}  // namespace goldilocks_models
}  // namespace dairlib
