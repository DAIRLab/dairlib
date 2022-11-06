#include "examples/goldilocks_models/planning/visualization/ik/kinematics_constraint_cost.h"

using std::isinf;
using std::isnan;
using std::list;
using std::make_shared;
using std::make_unique;
using std::map;
using std::string;
using std::unique_ptr;
using std::vector;

using drake::MatrixX;
using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::to_string;
using drake::solvers::VariableRefList;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib {
namespace goldilocks_models {
namespace planning {

KinematicsConstraintCost::KinematicsConstraintCost(
    const ReducedOrderModel& rom,
    const drake::multibody::MultibodyPlant<double>& plant, double weight,
    const std::string& description)
    : solvers::NonlinearCost<double>(rom.n_y() + plant.num_positions(),
                                     description),
      rom_(rom),
      plant_(plant),
      context_(plant.CreateDefaultContext()),
      n_y_(rom.n_y()),
      n_q_(plant.num_positions()),
      weight_(weight) {}

void KinematicsConstraintCost::EvaluateCost(
    const Eigen::Ref<const drake::VectorX<double>>& rq,
    drake::VectorX<double>* value) const {
  // Extract elements
  VectorX<double> r = rq.segment(0, n_y_);
  VectorX<double> q = rq.segment(n_y_, n_q_);

  // Update context
  plant_.SetPositions(context_.get(), q);

  // Impose dynamics constraint
  VectorX<double> h = rom_.EvalMappingFunc(q, *context_);
  *value = weight_ * (r - h).transpose() * (r - h);
}

}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
