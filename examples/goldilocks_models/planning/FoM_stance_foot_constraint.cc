#include "examples/goldilocks_models/planning/FoM_stance_foot_constraint.h"

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

FomStanceFootConstraint::FomStanceFootConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const vector<std::pair<const Vector3d, const Frame<double>&>>&
        stance_foot_contacts,
    const std::string& description)
    : NonlinearConstraint<double>(
          3 * stance_foot_contacts.size(), 2 * plant.num_positions(),
          VectorXd::Zero(3 * stance_foot_contacts.size()),
          VectorXd::Zero(3 * stance_foot_contacts.size()), description),
      plant_(plant),
      world_(plant.world_frame()),
      context_(plant.CreateDefaultContext()),
      stance_foot_contacts_(stance_foot_contacts) {}

void FomStanceFootConstraint::EvaluateConstraint(
    const Eigen::Ref<const VectorX<double>>& x, VectorX<double>* y) const {
  VectorX<double> q0 = x.head(7);
  VectorX<double> qf = x.tail(7);

  drake::VectorX<double> pt_0(3);
  drake::VectorX<double> pt_f(3);

  *y = VectorX<double>(3 * stance_foot_contacts_.size());
  for (int i = 0; i < stance_foot_contacts_.size(); i++) {
    const auto& contact = stance_foot_contacts_.at(i);

    plant_.SetPositions(context_.get(), q0);
    this->plant_.CalcPointsPositions(*context_, contact.second, contact.first,
                                     world_, &pt_0);
    plant_.SetPositions(context_.get(), qf);
    this->plant_.CalcPointsPositions(*context_, contact.second, contact.first,
                                     world_, &pt_f);
    y->segment<3>(3 * i) = pt_0 - pt_f;
  }
}

}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
