#include "examples/goldilocks_models/dummy_constraint.h"


namespace dairlib {
namespace goldilocks_models {


DummyConstraint::DummyConstraint(const
                                 MultibodyPlant<double>& plant,
                                 const std::string& description):
  Constraint(4,
             16,  // plant.get_num_positions(),
             VectorXd::Zero(4),
             VectorXd::Zero(4),
             description),
  plant_(plant) {
}


void DummyConstraint::DoEval(const
                             Eigen::Ref<const Eigen::VectorXd>& q,
                             Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(q), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

void DummyConstraint::DoEval(const
                             Eigen::Ref<const AutoDiffVecXd>& q,
                             AutoDiffVecXd* y) const {

  *y = initializeAutoDiff(VectorXd::Zero(4));

}

void DummyConstraint::DoEval(const
                             Eigen::Ref<const VectorX<Variable>>& x,
                             VectorX<Expression>*y) const {
  throw std::logic_error(
    "This constraint class does not support symbolic evaluation.");
}




}  // namespace goldilocks_models
}  // namespace dairlib
