#include "dircon_opt_constraints.h"
#include <stdexcept>
#include <utility>
#include <vector>

#include "common/file_utils.h"
#include "multibody/multibody_utils.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::map;
using std::string;

template <typename T>
QuaternionNormConstraint<T>::QuaternionNormConstraint()
    : solvers::NonlinearConstraint<T>(1, 4, VectorXd::Zero(1), 
          VectorXd::Zero(1), "quaternion_norm_constraint") {}
template <typename T>
void QuaternionNormConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  VectorX<T> output(1);
  output << x.norm() - 1;
  *y = output;
}

template <typename T>
DirconDynamicConstraint<T>::DirconDynamicConstraint(
    const MultibodyPlant<T>& plant, DirconKinematicDataSet<T>& constraints,
    bool is_quaternion)
    : DirconDynamicConstraint(plant, constraints, plant.num_positions(),
                              plant.num_velocities(), plant.num_actuators(),
                              constraints.countConstraintsWithoutSkipping(),
                              (is_quaternion) ? 1 : 0) {
  // If the MBP is in quaternion floating-base, demand that the quaternion
  // is located at the first four element of the generalized position
  if (is_quaternion) {
    map<string, int> positions_map = multibody::MakeNameToPositionsMap(plant);
    DRAKE_DEMAND(positions_map.at("base_qw") == 0);
    DRAKE_DEMAND(positions_map.at("base_qx") == 1);
    DRAKE_DEMAND(positions_map.at("base_qy") == 2);
    DRAKE_DEMAND(positions_map.at("base_qz") == 3);
  }
}

template <typename T>
DirconDynamicConstraint<T>::DirconDynamicConstraint(
    const MultibodyPlant<T>& plant, DirconKinematicDataSet<T>& constraints,
    int num_positions, int num_velocities, int num_inputs,
    int num_kinematic_constraints_wo_skipping, int num_quat_slack)
    : solvers::NonlinearConstraint<T>(
          num_positions + num_velocities,
          1 + 2 * (num_positions + num_velocities) + (2 * num_inputs) +
              (4 * num_kinematic_constraints_wo_skipping) + num_quat_slack,
          Eigen::VectorXd::Zero(num_positions + num_velocities),
          Eigen::VectorXd::Zero(num_positions + num_velocities),
          "dynamics_constraint"),
      plant_(plant),
      constraints_(&constraints),
      num_states_{num_positions + num_velocities},
      num_inputs_{num_inputs},
      num_kinematic_constraints_wo_skipping_{
          num_kinematic_constraints_wo_skipping},
      num_positions_{num_positions},
      num_velocities_{num_velocities},
      num_quat_slack_{num_quat_slack},
      context_(plant_.CreateDefaultContext()) {}

// The format of the input to the eval() function is the
// tuple { timestep, state 0, state 1, input 0, input 1, force 0, force 1},
// which has a total length of 1 + 2*num_states + 2*num_inputs +
// dim*num_constraints.
template <typename T>
void DirconDynamicConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  DRAKE_ASSERT(x.size() == 1 + (2 * num_states_) + (2 * num_inputs_) +
                               4 * (num_kinematic_constraints_wo_skipping_) +
                               num_quat_slack_);

  // Extract our input variables:
  // h - current time (knot) value
  // x0, x1 state vector at time steps k, k+1
  // u0, u1 input vector at time steps k, k+1
  const T h = x(0);
  const VectorX<T> x0 = x.segment(1, num_states_);
  const VectorX<T> x1 = x.segment(1 + num_states_, num_states_);
  const VectorX<T> u0 = x.segment(1 + (2 * num_states_), num_inputs_);
  const VectorX<T> u1 =
      x.segment(1 + (2 * num_states_) + num_inputs_, num_inputs_);
  const VectorX<T> l0 = x.segment(1 + 2 * (num_states_ + num_inputs_),
                                  num_kinematic_constraints_wo_skipping_);
  const VectorX<T> l1 = x.segment(1 + 2 * (num_states_ + num_inputs_) +
                                      num_kinematic_constraints_wo_skipping_,
                                  num_kinematic_constraints_wo_skipping_);
  const VectorX<T> lc =
      x.segment(1 + 2 * (num_states_ + num_inputs_) +
                    2 * num_kinematic_constraints_wo_skipping_,
                num_kinematic_constraints_wo_skipping_);
  const VectorX<T> vc =
      x.segment(1 + 2 * (num_states_ + num_inputs_) +
                    3 * num_kinematic_constraints_wo_skipping_,
                num_kinematic_constraints_wo_skipping_);
  const VectorX<T> gamma = x.tail(num_quat_slack_);

  multibody::SetContext<T>(plant_, x0, u0, context_.get());
  constraints_->updateData(*context_, l0);
  const VectorX<T> xdot0 = constraints_->getXDot();

  multibody::SetContext<T>(plant_, x1, u1, context_.get());
  constraints_->updateData(*context_, l1);
  const VectorX<T> xdot1 = constraints_->getXDot();

  // Cubic interpolation to get xcol and xdotcol.
  const VectorX<T> xcol = 0.5 * (x0 + x1) + h / 8 * (xdot0 - xdot1);
  const VectorX<T> xdotcol = -1.5 * (x0 - x1) / h - .25 * (xdot0 + xdot1);
  const VectorX<T> ucol = 0.5 * (u0 + u1);

  multibody::SetContext<T>(plant_, xcol, ucol, context_.get());
  constraints_->updateData(*context_, lc);
  auto g = constraints_->getXDot();
  VectorX<T> vc_in_qdot_space(num_positions_);
  plant_.MapVelocityToQDot(*context_,
                           constraints_->getJWithoutSkipping().transpose() * vc,
                           &vc_in_qdot_space);
  g.head(num_positions_) += vc_in_qdot_space;

  // The slack variable allows the quaternion to stay on a unit sphere.
  if (num_quat_slack_ > 0) {
    g.head(4) += xcol.head(4) * gamma;
  }

  *y = xdotcol - g;
}

template <typename T>
Binding<Constraint> AddDirconConstraint(
    std::shared_ptr<DirconDynamicConstraint<T>> constraint,
    const Eigen::Ref<const VectorXDecisionVariable>& timestep,
    const Eigen::Ref<const VectorXDecisionVariable>& state,
    const Eigen::Ref<const VectorXDecisionVariable>& next_state,
    const Eigen::Ref<const VectorXDecisionVariable>& input,
    const Eigen::Ref<const VectorXDecisionVariable>& next_input,
    const Eigen::Ref<const VectorXDecisionVariable>& force,
    const Eigen::Ref<const VectorXDecisionVariable>& next_force,
    const Eigen::Ref<const VectorXDecisionVariable>& collocation_force,
    const Eigen::Ref<const VectorXDecisionVariable>& collocation_position_slack,
    MathematicalProgram* prog) {
  DRAKE_DEMAND(timestep.size() == 1);
  DRAKE_DEMAND(state.size() == constraint->num_states());
  DRAKE_DEMAND(next_state.size() == constraint->num_states());
  DRAKE_DEMAND(input.size() == constraint->num_inputs());
  DRAKE_DEMAND(next_input.size() == constraint->num_inputs());
  DRAKE_DEMAND(force.size() ==
               constraint->num_kinematic_constraints_wo_skipping());
  DRAKE_DEMAND(next_force.size() ==
               constraint->num_kinematic_constraints_wo_skipping());
  DRAKE_DEMAND(collocation_force.size() ==
               constraint->num_kinematic_constraints_wo_skipping());
  DRAKE_DEMAND(collocation_position_slack.size() ==
               constraint->num_kinematic_constraints_wo_skipping());
  return prog->AddConstraint(
      constraint, {timestep, state, next_state, input, next_input, force,
                   next_force, collocation_force, collocation_position_slack});
}

template <typename T>
DirconKinematicConstraint<T>::DirconKinematicConstraint(
    const MultibodyPlant<T>& plant, DirconKinematicDataSet<T>& constraints,
    DirconKinConstraintType type)
    : DirconKinematicConstraint(
          plant, constraints,
          std::vector<bool>(constraints.countConstraints(), false), type,
          plant.num_positions(), plant.num_velocities(), plant.num_actuators(),
          constraints.countConstraints(),
          constraints.countConstraintsWithoutSkipping()) {}

template <typename T>
DirconKinematicConstraint<T>::DirconKinematicConstraint(
    const MultibodyPlant<T>& plant, DirconKinematicDataSet<T>& constraints,
    std::vector<bool> is_constraint_relative, DirconKinConstraintType type)
    : DirconKinematicConstraint(plant, constraints, is_constraint_relative,
                                type, plant.num_positions(),
                                plant.num_velocities(), plant.num_actuators(),
                                constraints.countConstraints(),
                                constraints.countConstraintsWithoutSkipping()) {
}

template <typename T>
DirconKinematicConstraint<T>::DirconKinematicConstraint(
    const MultibodyPlant<T>& plant, DirconKinematicDataSet<T>& constraints,
    std::vector<bool> is_constraint_relative, DirconKinConstraintType type,
    int num_positions, int num_velocities, int num_inputs,
    int num_kinematic_constraints, int num_kinematic_constraints_wo_skipping)
    : solvers::NonlinearConstraint<T>(
          type * num_kinematic_constraints,
          num_positions + num_velocities + num_inputs +
              num_kinematic_constraints_wo_skipping +
              std::count(is_constraint_relative.begin(),
                         is_constraint_relative.end(), true),
          VectorXd::Zero(type * num_kinematic_constraints),
          VectorXd::Zero(type * num_kinematic_constraints),
          "kinematics_constraint"),
      plant_(plant),
      constraints_(&constraints),
      num_states_{num_positions + num_velocities},
      num_inputs_{num_inputs},
      num_kinematic_constraints_{num_kinematic_constraints},
      num_kinematic_constraints_wo_skipping_{
          num_kinematic_constraints_wo_skipping},
      num_positions_{num_positions},
      num_velocities_{num_velocities},
      type_{type},
      is_constraint_relative_{is_constraint_relative},
      n_relative_{
          static_cast<int>(std::count(is_constraint_relative.begin(),
                                      is_constraint_relative.end(), true))},
      context_(plant_.CreateDefaultContext()) {
  // Set sparsity pattern and relative map
  std::vector<std::pair<int, int>> sparsity;
  // Acceleration constraints are dense in decision variables
  for (int i = 0; i < num_kinematic_constraints_; i++) {
    for (int j = 0; j < this->num_vars(); j++) {
      sparsity.push_back({i, j});
    }
  }

  // Velocity constraint depends on q and v only
  if (type_ == kAll || type == kAccelAndVel) {
    for (int i = 0; i < num_kinematic_constraints_; i++) {
      for (int j = 0; j < num_states_; j++) {
        sparsity.push_back({i + num_kinematic_constraints_, j});
      }
    }
  }

  // Position constraint only depends on q and any offset variables
  // Set relative map in the same loop
  if (type == kAll) {
    relative_map_ = MatrixXd::Zero(num_kinematic_constraints_, n_relative_);
    int k = 0;

    for (int i = 0; i < num_kinematic_constraints_; i++) {
      for (int j = 0; j < num_positions_; j++) {
        sparsity.push_back({i + 2 * num_kinematic_constraints_, j});
      }

      if (is_constraint_relative_[i]) {
        relative_map_(i, k) = 1;
        // ith constraint depends on kth offset variable
        sparsity.push_back({i + 2 * num_kinematic_constraints_,
                            num_states_ + num_inputs_ +
                                num_kinematic_constraints_wo_skipping + k});

        k++;
      }
    }
  }

  this->SetGradientSparsityPattern(sparsity);
}

template <typename T>
void DirconKinematicConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  DRAKE_ASSERT(x.size() == num_states_ + num_inputs_ +
                               num_kinematic_constraints_wo_skipping_ +
                               n_relative_);

  // Extract our input variables:
  // h - current time (knot) value
  // x0, x1 state vector at time steps k, k+1
  // u0, u1 input vector at time steps k, k+1
  const VectorX<T> state = x.segment(0, num_states_);
  const VectorX<T> input = x.segment(num_states_, num_inputs_);
  const VectorX<T> force = x.segment(num_states_ + num_inputs_,
                                     num_kinematic_constraints_wo_skipping_);
  const VectorX<T> offset = x.segment(
      num_states_ + num_inputs_ + num_kinematic_constraints_wo_skipping_,
      n_relative_);
  multibody::SetContext<T>(plant_, state, input, context_.get());
  constraints_->updateData(*context_, force);
  switch (type_) {
    case kAll:
      *y = VectorX<T>(3 * num_kinematic_constraints_);
      *y << constraints_->getCDDot(), constraints_->getCDot(),
           constraints_->getC() + relative_map_ * offset;
      break;
    case kAccelAndVel:
      *y = VectorX<T>(2 * num_kinematic_constraints_);
      *y << constraints_->getCDDot(), constraints_->getCDot();
      break;
    case kAccelOnly:
      *y = VectorX<T>(1 * num_kinematic_constraints_);
      *y << constraints_->getCDDot();
      break;
  }
}

template <typename T>
DirconImpactConstraint<T>::DirconImpactConstraint(
    const MultibodyPlant<T>& plant, DirconKinematicDataSet<T>& constraints)
    : DirconImpactConstraint(plant, constraints, plant.num_positions(),
                             plant.num_velocities(),
                             constraints.countConstraintsWithoutSkipping()) {}

template <typename T>
DirconImpactConstraint<T>::DirconImpactConstraint(
    const MultibodyPlant<T>& plant, DirconKinematicDataSet<T>& constraints,
    int num_positions, int num_velocities,
    int num_kinematic_constraints_wo_skipping)
    : solvers::NonlinearConstraint<T>(num_velocities,
                                  num_positions + 2 * num_velocities +
                                      num_kinematic_constraints_wo_skipping,
                                  VectorXd::Zero(num_velocities),
                                  VectorXd::Zero(num_velocities),
                                  "impact_constraint"),
      plant_(plant),
      constraints_(&constraints),
      num_states_{num_positions + num_velocities},
      num_kinematic_constraints_wo_skipping_{
          num_kinematic_constraints_wo_skipping},
      num_positions_{num_positions},
      num_velocities_{num_velocities},
      context_(plant_.CreateDefaultContext()) {}

// The format of the input to the eval() function is the
// tuple { state 0, impulse, velocity 1},
template <typename T>
void DirconImpactConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  DRAKE_ASSERT(x.size() == 2 * num_velocities_ + num_positions_ +
                               num_kinematic_constraints_wo_skipping_);

  // Extract our input variables:
  // x0, state vector at time k^-
  // impulse, impulsive force at impact
  // v1, post-impact velocity at time k^+
  const VectorX<T> x0 = x.segment(0, num_states_);
  const VectorX<T> impulse =
      x.segment(num_states_, num_kinematic_constraints_wo_skipping_);
  const VectorX<T> v1 = x.segment(
      num_states_ + num_kinematic_constraints_wo_skipping_, num_velocities_);

  const VectorX<T> v0 = x0.tail(num_velocities_);

  // vp = vm + M^{-1}*J^T*Lambda
  const VectorX<T> u =
      VectorXd::Zero(plant_.num_actuators()).template cast<T>();

  multibody::SetContext<T>(plant_, x0, u, context_.get());

  constraints_->updateData(*context_, impulse);

  MatrixX<T> M(num_velocities_, num_velocities_);
  plant_.CalcMassMatrix(*context_, &M);

  *y =
      M * (v1 - v0) - constraints_->getJWithoutSkipping().transpose() * impulse;
}

template <typename T>
PointPositionConstraint<T>::PointPositionConstraint(
    const drake::multibody::MultibodyPlant<T>& plant,
    const std::string& body_name, const Eigen::Vector3d& point_wrt_body,
    const Eigen::Vector3d& fix_pos)
    : PointPositionConstraint(plant, body_name, point_wrt_body,
                              Eigen::Matrix3d::Identity(), fix_pos, fix_pos) {}

template <typename T>
PointPositionConstraint<T>::PointPositionConstraint(
    const drake::multibody::MultibodyPlant<T>& plant,
    const std::string& body_name, const Eigen::Vector3d& point_wrt_body,
    const Eigen::Matrix<double, Eigen::Dynamic, 3>& dir,
    const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
    const std::string& description)
    : solvers::NonlinearConstraint<T>(
          dir.rows(), plant.num_positions(), lb, ub,
          description.empty() ? body_name + "_pos_constraint" : description),
      plant_(plant),
      body_(plant.GetBodyByName(body_name)),
      point_wrt_body_(point_wrt_body.template cast<T>()),
      dir_(dir.template cast<T>()),
      context_(plant_.CreateDefaultContext()) {}

template <typename T>
void PointPositionConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  multibody::SetPositionsIfNew<T>(plant_, x, context_.get());

  drake::VectorX<T> pt(3);
  this->plant_.CalcPointsPositions(*context_, body_.body_frame(),
                                   point_wrt_body_, plant_.world_frame(), &pt);
  *y = dir_ * pt;
};

template <typename T>
PointVelocityConstraint<T>::PointVelocityConstraint(
    const drake::multibody::MultibodyPlant<T>& plant,
    const std::string& body_name, const Eigen::Vector3d& point_wrt_body,
    const Eigen::Vector3d& fix_pos)
    : PointVelocityConstraint(plant, body_name, point_wrt_body,
                              Eigen::Matrix3d::Identity(), fix_pos, fix_pos) {}

template <typename T>
PointVelocityConstraint<T>::PointVelocityConstraint(
    const drake::multibody::MultibodyPlant<T>& plant,
    const std::string& body_name, const Eigen::Vector3d& point_wrt_body,
    const Eigen::Matrix<double, Eigen::Dynamic, 3>& dir,
    const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
    const std::string& description)
    : solvers::NonlinearConstraint<T>(
          dir.rows(), plant.num_positions(), lb, ub,
          description.empty() ? body_name + "_vel_constraint" : description),
      plant_(plant),
      body_(plant.GetBodyByName(body_name)),
      point_wrt_body_(point_wrt_body.template cast<T>()),
      dir_(dir.template cast<T>()),
      context_(plant_.CreateDefaultContext()) {}

template <typename T>
void PointVelocityConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  multibody::SetPositionsAndVelocitiesIfNew<T>(plant_, x, context_.get());

  drake::MatrixX<T> J(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      *context_, drake::multibody::JacobianWrtVariable::kV, body_.body_frame(),
      point_wrt_body_, plant_.world_frame(), plant_.world_frame(), &J);

  *y = dir_ * J * x.tail(plant_.num_velocities());
};

// Explicitly instantiates on the most common scalar types.
template class QuaternionNormConstraint<double>;
template class QuaternionNormConstraint<AutoDiffXd>;
template class DirconDynamicConstraint<double>;
template class DirconDynamicConstraint<AutoDiffXd>;
template class DirconKinematicConstraint<double>;
template class DirconKinematicConstraint<AutoDiffXd>;
template class DirconImpactConstraint<double>;
template class DirconImpactConstraint<AutoDiffXd>;
template class PointPositionConstraint<double>;
template class PointPositionConstraint<AutoDiffXd>;
template class PointVelocityConstraint<double>;
template class PointVelocityConstraint<AutoDiffXd>;

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
