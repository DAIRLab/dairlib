#include "examples/goldilocks_models/planning/FoM_guard_constraint.h"

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

using Eigen::AutoDiffScalar;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::VectorX;
using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using drake::math::DiscardGradient;
using drake::math::initializeAutoDiff;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::to_string;
using drake::solvers::VariableRefList;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;

namespace dairlib {
namespace goldilocks_models {
namespace planning {

FomGuardConstraint::FomGuardConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const vector<std::pair<const Vector3d, const Frame<double>&>>&
        swing_foot_contacts,
    const VectorXd& lb, const VectorXd& ub, const std::string& description)
    : NonlinearConstraint<double>(
          2 * swing_foot_contacts.size(),
          plant.num_positions() + plant.num_velocities(), lb, ub, description),
      plant_(plant),
      world_(plant.world_frame()),
      context_(plant.CreateDefaultContext()),
      swing_foot_contacts_(swing_foot_contacts) {}

void FomGuardConstraint::EvaluateConstraint(
    const Eigen::Ref<const VectorX<double>>& x, VectorX<double>* y) const {
  plant_.SetPositions(context_.get(), x.head(plant_.num_positions()));

  drake::VectorX<double> pt(3);
  drake::MatrixX<double> J(3, plant_.num_velocities());

  *y = VectorX<double>(2 * swing_foot_contacts_.size());
  for (int i = 0; i < swing_foot_contacts_.size(); i++) {
    const auto& contact = swing_foot_contacts_.at(i);
    // fill in position
    this->plant_.CalcPointsPositions(*context_, contact.second, contact.first,
                                     world_, &pt);
    y->segment<1>(2 * i) = pt.row(2);

    // fill in velocity
    plant_.CalcJacobianTranslationalVelocity(
        *context_, drake::multibody::JacobianWrtVariable::kV, contact.second,
        contact.first, world_, world_, &J);
    y->segment<1>(2 * i + 1) = J.row(2) * x.tail(plant_.num_velocities());
  }
}

}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
