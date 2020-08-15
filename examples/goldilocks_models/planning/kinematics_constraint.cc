#include "examples/goldilocks_models/planning/kinematics_constraint.h"

#include <utility>

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

KinematicsConstraint::KinematicsConstraint(
    const ReducedOrderModel& rom,
    const drake::multibody::MultibodyPlant<double>& plant, bool left_stance,
    const StateMirror& state_mirror, const std::string& description)
    : NonlinearConstraint<double>(
          2 * rom.n_y(),
          2 * rom.n_y() + plant.num_positions() + plant.num_velocities(),
          VectorXd::Zero(2 * rom.n_y()), VectorXd::Zero(2 * rom.n_y()),
          description),
      rom_(rom),
      plant_(plant),
      context_(plant.CreateDefaultContext()),
      left_stance_(left_stance),
      state_mirror_(state_mirror),
      n_y_(rom.n_y()),
      n_z_(2 * rom.n_y()),
      n_q_(plant.num_positions()),
      n_v_(plant.num_velocities()),
      n_x_(plant.num_positions() + plant.num_velocities()) {}

void KinematicsConstraint::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<double>>& zx,
    drake::VectorX<double>* output) const {
  // Extract elements
  VectorX<double> x(n_x_);
  if (left_stance_) {
    x = zx.tail(n_x_);
  } else {
    x << state_mirror_.MirrorPos(zx.segment(n_z_, n_q_)),
        state_mirror_.MirrorVel(zx.segment(n_z_ + n_q_, n_v_));
  }

  // Update context
  plant_.SetPositionsAndVelocities(context_.get(), x);

  VectorX<double> value(n_z_);
  value << zx.segment(0, n_y_) - rom_.EvalMappingFunc(x.head(n_q_), *context_),
      zx.segment(n_y_, n_y_) -
          rom_.EvalMappingFuncJV(x.head(n_q_), x.tail(n_v_), *context_);

  // Impose constraint
  *output = value;
}

}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
