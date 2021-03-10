#include "examples/goldilocks_models/planning/FoM_reset_map_constraint.h"

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
namespace planning {

FomResetMapConstraint::FomResetMapConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::vector<std::pair<const Eigen::Vector3d,
                                const drake::multibody::Frame<double>&>>&
        impact_foot_contacts,
    const std::string& description)
    : NonlinearConstraint<double>(
          plant.num_velocities(),
          2 * plant.num_velocities() + plant.num_positions() +
              3 * impact_foot_contacts.size(),
          VectorXd::Zero(plant.num_velocities()),
          VectorXd::Zero(plant.num_velocities()), description),
      plant_(plant),
      world_(plant.world_frame()),
      context_(plant.CreateDefaultContext()),
      impact_foot_contacts_(impact_foot_contacts),
      n_q_(plant.num_positions()),
      n_v_(plant.num_velocities()),
      n_lambda_(3 * impact_foot_contacts.size()) {}

void FomResetMapConstraint::EvaluateConstraint(
    const Eigen::Ref<const VectorX<double>>& x, VectorX<double>* y) const {
  // m stands for minus (pre-impact)
  // p stands for plus (post-impact)
  VectorX<double> qm = x.head(n_q_);
  VectorX<double> vm = x.segment(n_q_, n_v_);
  VectorX<double> vp = x.segment(n_q_ + n_v_, n_v_);
  VectorX<double> Lambda = x.tail(n_lambda_);

  MatrixXd M = MatrixXd(n_v_, n_v_);
  plant_.SetPositions(context_.get(), qm);
  plant_.CalcMassMatrix(*context_, &M);

  MatrixX<double> J_impact_foot(n_lambda_, n_v_);
  MatrixX<double> J_per_contact(3, n_v_);
  for (int i = 0; i < impact_foot_contacts_.size(); i++) {
    const auto& contact = impact_foot_contacts_.at(i);
    plant_.CalcJacobianTranslationalVelocity(
        *context_, drake::multibody::JacobianWrtVariable::kV, contact.second,
        contact.first, world_, world_, &J_per_contact);
    J_impact_foot.block(3 * i, 0, 3, n_v_) = J_per_contact;
  }

  *y = VectorX<double>(n_v_);
  *y << M * (vp - vm) - J_impact_foot.transpose() * Lambda;
}

}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
