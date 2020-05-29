#include "multibody/multibody_solvers.h"

namespace dairlib {
namespace multibody {

using Eigen::VectorXd;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::VectorX;
using solvers::NonlinearConstraint;

template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const std::vector<KinematicEvaluator<T>>& evaluators,
    std::shared_ptr<Context<T>> context, const std::string& description)
    : NonlinearConstraint<T>(0, plant.num_positions(), VectorXd::Zero(0),
          VectorXd::Zero(0), description),
      plant_(plant), 
      evaluators_(evaluators),
      context_(context) {
  // Length and bounds set above are fake. Count up constraints now.
  int num_constraints = 0;
  for (const auto& e : evaluators_) {
    num_constraints += e.num_active();
  }

  set_num_outputs(num_constraints);
  set_bounds(VectorXd::Zero(num_constraints), VectorXd::Zero(num_constraints));

}

template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const std::vector<KinematicEvaluator<T>>& evaluators,
    const std::string& description) 
    : KinematicPositionConstraint<T>(plant, evaluators,
      std::shared_ptr<Context<T>>(plant_.CreateDefaultContext().release()),
      description) {}

template <typename T>
void KinematicPositionConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& q, VectorX<T>* y) const {
  // Verifying the size of the input vector
  DRAKE_DEMAND(q.size() == plant_.num_positions());

  plant_.SetPositions(context_.get(), q);

  
  y->resize(this->num_constraints());
  int ind = 0;
  for (const auto& e : evaluators_) {
    y->segment(ind, e.num_active()) = e.EvalActive(*context_);
    ind += e.num_active();
  }
}

}  // namespace multibody
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicPositionConstraint)
