#include "systems/trajectory_optimization/dircon/dircon_opt_constraints.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

using multibody::KinematicEvaluatorSet;
using solvers::NonlinearConstraint;

using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

using Eigen::VectorXd;

template <typename T>
QuaternionNormConstraint<T>::QuaternionNormConstraint()
    : NonlinearConstraint<T>(1, 4, VectorXd::Zero(1), 
          VectorXd::Zero(1), "quaternion_norm_constraint") {}
template <typename T>
void QuaternionNormConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  (*y).resize(1);
  *y << x.norm() - 1;
}

template <typename T>
DirconCollocationConstraint<T>::DirconCollocationConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators, Context<T>* context_0,
    Context<T>* context_1, int mode_index, int knot_index)
    : NonlinearConstraint<T>(
          plant.num_positions() + plant.num_velocities(),
          1 + 2 * (plant.num_positions() + plant.num_velocities()
              + plant.num_actuators())
              + (4 * evaluators.count_full())
              + multibody::QuaternionStartIndices(plant).size(),
          VectorXd::Zero(plant.num_positions() + plant.num_velocities()),
          VectorXd::Zero(plant.num_positions() + plant.num_velocities()),
          "collocation[" + std::to_string(mode_index) + "]["
              + std::to_string(knot_index) + "]"),
      plant_(plant),
      evaluators_(evaluators),
      context_0_(context_0),
      context_1_(context_1),
      context_col_(plant.CreateDefaultContext()),
      quat_start_indices_(multibody::QuaternionStartIndices(plant)),
      n_x_(plant.num_positions() + plant.num_velocities()),
      n_u_(plant.num_actuators()),
      n_l_(evaluators.count_full()) {}

/// The format of the input to the eval() function is in the order
///   - timestep h
///   - x0, state at time k
///   - x1, state at time k+1
///   - u0, control input at time k
///   - u1, control input at time k+1
///   - l0, constraint force at time k
///   - l1, constraint force at time k+1
///   - lc, constraint force at collocation point between k and k+1
///   - gamma, velocity slack at collocation point between k and k+1
///   - quat_slack, quaternion slack at collocation point between k and k+1
template <typename T>
void DirconCollocationConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  DRAKE_ASSERT(x.size() == 1 + 2 * (n_x_ + n_u_) + 4* n_l_ +
      quat_start_indices_.size())
  const T& h = x(0);
  const auto& x0 = x.segment(1, n_x_);
  const auto& x1 = x.segment(1 + n_x_, n_x_);
  const auto& u0 = x.segment(1 + 2 * n_x_, n_u_);
  const auto& u1 = x.segment(1 + 2 * n_x_ + n_u_, n_u_);
  const auto& l0 = x.segment(1 + 2 * (n_x_ + n_u_), n_l_);
  const auto& l1 = x.segment(1 + 2 * (n_x_ + n_u_) + n_l_, n_l_);
  const auto& lc = x.segment(1 + 2 * (n_x_ + n_u_) + 2* n_l_, n_l_);
  const auto& gamma = x.segment(1 + 2 * (n_x_ + n_u_) + 3* n_l_, n_l_);
  const auto& quat_slack = x.segment(1 + 2 * (n_x_ + n_u_) + 4* n_l_,
      quat_start_indices_.size());

  // Evaluate dynamics at k and k+1
  multibody::setContext<T>(plant_, x0, u0, context_0_);
  multibody::setContext<T>(plant_, x1, u1, context_1_);
  const auto& xdot0 = evaluators_.CalcTimeDerivatives(*context_0_, l0);
  const auto& xdot1 = evaluators_.CalcTimeDerivatives(*context_1_, l1);

  // Cubic interpolation to get xcol and xdotcol.
  const auto& xcol = 0.5 * (x0 + x1) + h / 8 * (xdot0 - xdot1);
  const auto& xdotcol = -1.5 * (x0 - x1) / h - .25 * (xdot0 + xdot1);
  const auto& ucol = 0.5 * (u0 + u1);

  // Evaluate dynamics at colocation point
  multibody::setContext<T>(plant_, xcol, ucol, context_col_.get());
  auto g = evaluators_.CalcTimeDerivatives(*context_col_, lc);
  
  // Add velocity slack contribution, J^T * gamma
  VectorX<T> gamma_in_qdot_space(plant_.num_positions());
  plant_.MapVelocityToQDot(*context_col_,
      evaluators_.EvalFullJacobian(*context_col_).transpose() * gamma,
      &gamma_in_qdot_space);
  g.head(plant_.num_positions()) += gamma_in_qdot_space;

  // Add quaternion slack contribution, quat * slack
  for (uint i = 0; i < quat_start_indices_.size(); i++) {
    g.segment(quat_start_indices_.at(i), 4) +=
        g.segment(quat_start_indices_.at(i), 4) * quat_slack(i);
  }

  *y = xdotcol - g;
}

// template <typename T>
// Binding<Constraint> AddDirconConstraint(
//     std::shared_ptr<DirconDynamicConstraint<T>> constraint,
//     const Eigen::Ref<const VectorXDecisionVariable>& timestep,
//     const Eigen::Ref<const VectorXDecisionVariable>& state,
//     const Eigen::Ref<const VectorXDecisionVariable>& next_state,
//     const Eigen::Ref<const VectorXDecisionVariable>& input,
//     const Eigen::Ref<const VectorXDecisionVariable>& next_input,
//     const Eigen::Ref<const VectorXDecisionVariable>& force,
//     const Eigen::Ref<const VectorXDecisionVariable>& next_force,
//     const Eigen::Ref<const VectorXDecisionVariable>& collocation_force,
//     const Eigen::Ref<const VectorXDecisionVariable>& collocation_position_slack,
//     MathematicalProgram* prog) {
//   DRAKE_DEMAND(timestep.size() == 1);
//   DRAKE_DEMAND(state.size() == constraint->num_states());
//   DRAKE_DEMAND(next_state.size() == constraint->num_states());
//   DRAKE_DEMAND(input.size() == constraint->num_inputs());
//   DRAKE_DEMAND(next_input.size() == constraint->num_inputs());
//   DRAKE_DEMAND(force.size() ==
//                constraint->num_kinematic_constraints_wo_skipping());
//   DRAKE_DEMAND(next_force.size() ==
//                constraint->num_kinematic_constraints_wo_skipping());
//   DRAKE_DEMAND(collocation_force.size() ==
//                constraint->num_kinematic_constraints_wo_skipping());
//   DRAKE_DEMAND(collocation_position_slack.size() ==
//                constraint->num_kinematic_constraints_wo_skipping());
//   return prog->AddConstraint(
//       constraint, {timestep, state, next_state, input, next_input, force,
//                    next_force, collocation_force, collocation_position_slack});
// }

// template <typename T>
// DirconKinematicConstraint<T>::DirconKinematicConstraint(
//     const MultibodyPlant<T>& plant, DirconKinematicDataSet<T>& constraints,
//     DirconKinConstraintType type)
//     : DirconKinematicConstraint(
//           plant, constraints,
//           std::vector<bool>(constraints.countConstraints(), false), type,
//           plant.num_positions(), plant.num_velocities(), plant.num_actuators(),
//           constraints.countConstraints(),
//           constraints.countConstraintsWithoutSkipping()) {}

// template <typename T>
// DirconKinematicConstraint<T>::DirconKinematicConstraint(
//     const MultibodyPlant<T>& plant, DirconKinematicDataSet<T>& constraints,
//     std::vector<bool> is_constraint_relative, DirconKinConstraintType type)
//     : DirconKinematicConstraint(plant, constraints, is_constraint_relative,
//                                 type, plant.num_positions(),
//                                 plant.num_velocities(), plant.num_actuators(),
//                                 constraints.countConstraints(),
//                                 constraints.countConstraintsWithoutSkipping()) {
// }

// template <typename T>
// DirconKinematicConstraint<T>::DirconKinematicConstraint(
//     const MultibodyPlant<T>& plant, DirconKinematicDataSet<T>& constraints,
//     std::vector<bool> is_constraint_relative, DirconKinConstraintType type,
//     int num_positions, int num_velocities, int num_inputs,
//     int num_kinematic_constraints, int num_kinematic_constraints_wo_skipping)
//     : solvers::NonlinearConstraint<T>(
//           type * num_kinematic_constraints,
//           num_positions + num_velocities + num_inputs +
//               num_kinematic_constraints_wo_skipping +
//               std::count(is_constraint_relative.begin(),
//                          is_constraint_relative.end(), true),
//           VectorXd::Zero(type * num_kinematic_constraints),
//           VectorXd::Zero(type * num_kinematic_constraints),
//           "kinematics_constraint"),
//       plant_(plant),
//       constraints_(&constraints),
//       num_states_{num_positions + num_velocities},
//       num_inputs_{num_inputs},
//       num_kinematic_constraints_{num_kinematic_constraints},
//       num_kinematic_constraints_wo_skipping_{
//           num_kinematic_constraints_wo_skipping},
//       num_positions_{num_positions},
//       num_velocities_{num_velocities},
//       type_{type},
//       is_constraint_relative_{is_constraint_relative},
//       n_relative_{
//           static_cast<int>(std::count(is_constraint_relative.begin(),
//                                       is_constraint_relative.end(), true))},
//       context_(plant_.CreateDefaultContext()) {
//   // Set sparsity pattern and relative map
//   std::vector<std::pair<int, int>> sparsity;
//   // Acceleration constraints are dense in decision variables
//   for (int i = 0; i < num_kinematic_constraints_; i++) {
//     for (int j = 0; j < this->num_vars(); j++) {
//       sparsity.push_back({i, j});
//     }
//   }

//   // Velocity constraint depends on q and v only
//   if (type_ == kAll || type == kAccelAndVel) {
//     for (int i = 0; i < num_kinematic_constraints_; i++) {
//       for (int j = 0; j < num_states_; j++) {
//         sparsity.push_back({i + num_kinematic_constraints_, j});
//       }
//     }
//   }

//   // Position constraint only depends on q and any offset variables
//   // Set relative map in the same loop
//   if (type == kAll) {
//     relative_map_ = MatrixXd::Zero(num_kinematic_constraints_, n_relative_);
//     int k = 0;

//     for (int i = 0; i < num_kinematic_constraints_; i++) {
//       for (int j = 0; j < num_positions_; j++) {
//         sparsity.push_back({i + 2 * num_kinematic_constraints_, j});
//       }

//       if (is_constraint_relative_[i]) {
//         relative_map_(i, k) = 1;
//         // ith constraint depends on kth offset variable
//         sparsity.push_back({i + 2 * num_kinematic_constraints_,
//                             num_states_ + num_inputs_ +
//                                 num_kinematic_constraints_wo_skipping + k});

//         k++;
//       }
//     }
//   }

//   this->SetGradientSparsityPattern(sparsity);
// }

// template <typename T>
// void DirconKinematicConstraint<T>::EvaluateConstraint(
//     const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
//   DRAKE_ASSERT(x.size() == num_states_ + num_inputs_ +
//                                num_kinematic_constraints_wo_skipping_ +
//                                n_relative_);

//   // Extract our input variables:
//   // h - current time (knot) value
//   // x0, x1 state vector at time steps k, k+1
//   // u0, u1 input vector at time steps k, k+1
//   const VectorX<T> state = x.segment(0, num_states_);
//   const VectorX<T> input = x.segment(num_states_, num_inputs_);
//   const VectorX<T> force = x.segment(num_states_ + num_inputs_,
//                                      num_kinematic_constraints_wo_skipping_);
//   const VectorX<T> offset = x.segment(
//       num_states_ + num_inputs_ + num_kinematic_constraints_wo_skipping_,
//       n_relative_);
//   multibody::setContext(plant_, state, input, context_.get());
//   constraints_->updateData(*context_, force);
//   switch (type_) {
//     case kAll:
//       *y = VectorX<T>(3 * num_kinematic_constraints_);
//       *y << constraints_->getCDDot(), constraints_->getCDot(),
//           constraints_->getC() + relative_map_ * offset;
//       break;
//     case kAccelAndVel:
//       *y = VectorX<T>(2 * num_kinematic_constraints_);
//       *y << constraints_->getCDDot(), constraints_->getCDot();
//       break;
//     case kAccelOnly:
//       *y = VectorX<T>(1 * num_kinematic_constraints_);
//       *y << constraints_->getCDDot();
//       break;
//   }
// }

// template <typename T>
// DirconImpactConstraint<T>::DirconImpactConstraint(
//     const MultibodyPlant<T>& plant, DirconKinematicDataSet<T>& constraints)
//     : DirconImpactConstraint(plant, constraints, plant.num_positions(),
//                              plant.num_velocities(),
//                              constraints.countConstraintsWithoutSkipping()) {}

// template <typename T>
// DirconImpactConstraint<T>::DirconImpactConstraint(
//     const MultibodyPlant<T>& plant, DirconKinematicDataSet<T>& constraints,
//     int num_positions, int num_velocities,
//     int num_kinematic_constraints_wo_skipping)
//     : solvers::NonlinearConstraint<T>(num_velocities,
//                                   num_positions + 2 * num_velocities +
//                                       num_kinematic_constraints_wo_skipping,
//                                   VectorXd::Zero(num_velocities),
//                                   VectorXd::Zero(num_velocities),
//                                   "impact_constraint"),
//       plant_(plant),
//       constraints_(&constraints),
//       num_states_{num_positions + num_velocities},
//       num_kinematic_constraints_wo_skipping_{
//           num_kinematic_constraints_wo_skipping},
//       num_positions_{num_positions},
//       num_velocities_{num_velocities},
//       context_(plant_.CreateDefaultContext()) {}

// // The format of the input to the eval() function is the
// // tuple { state 0, impulse, velocity 1},
// template <typename T>
// void DirconImpactConstraint<T>::EvaluateConstraint(
//     const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
//   DRAKE_ASSERT(x.size() == 2 * num_velocities_ + num_positions_ +
//                                num_kinematic_constraints_wo_skipping_);

//   // Extract our input variables:
//   // x0, state vector at time k^-
//   // impulse, impulsive force at impact
//   // v1, post-impact velocity at time k^+
//   const VectorX<T> x0 = x.segment(0, num_states_);
//   const VectorX<T> impulse =
//       x.segment(num_states_, num_kinematic_constraints_wo_skipping_);
//   const VectorX<T> v1 = x.segment(
//       num_states_ + num_kinematic_constraints_wo_skipping_, num_velocities_);

//   const VectorX<T> v0 = x0.tail(num_velocities_);

//   // vp = vm + M^{-1}*J^T*Lambda
//   const VectorX<T> u =
//       VectorXd::Zero(plant_.num_actuators()).template cast<T>();

//   multibody::setContext(plant_, x0, u, context_.get());

//   constraints_->updateData(*context_, impulse);

//   MatrixX<T> M(num_velocities_, num_velocities_);
//   plant_.CalcMassMatrix(*context_, &M);

//   *y =
//       M * (v1 - v0) - constraints_->getJWithoutSkipping().transpose() * impulse;
// }

// template <typename T>
// PointPositionConstraint<T>::PointPositionConstraint(
//     const drake::multibody::MultibodyPlant<T>& plant,
//     const std::string& body_name, const Eigen::Vector3d& point_wrt_body,
//     const Eigen::Vector3d& fix_pos)
//     : PointPositionConstraint(plant, body_name, point_wrt_body,
//                               Eigen::Matrix3d::Identity(), fix_pos, fix_pos) {}

// template <typename T>
// PointPositionConstraint<T>::PointPositionConstraint(
//     const drake::multibody::MultibodyPlant<T>& plant,
//     const std::string& body_name, const Eigen::Vector3d& point_wrt_body,
//     const Eigen::Matrix<double, Eigen::Dynamic, 3>& dir,
//     const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
//     const std::string& description)
//     : solvers::NonlinearConstraint<T>(
//           dir.rows(), plant.num_positions(), lb, ub,
//           description.empty() ? body_name + "_pos_constraint" : description),
//       plant_(plant),
//       body_(plant.GetBodyByName(body_name)),
//       point_wrt_body_(point_wrt_body.template cast<T>()),
//       dir_(dir.template cast<T>()),
//       context_(plant_.CreateDefaultContext()) {}

// template <typename T>
// void PointPositionConstraint<T>::EvaluateConstraint(
//     const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
//   plant_.SetPositions(context_.get(), x);

//   VectorX<T> pt(3);
//   this->plant_.CalcPointsPositions(*context_, body_.body_frame(),
//                                    point_wrt_body_, plant_.world_frame(), &pt);
//   *y = dir_ * pt;
// };

// template <typename T>
// PointVelocityConstraint<T>::PointVelocityConstraint(
//     const drake::multibody::MultibodyPlant<T>& plant,
//     const std::string& body_name, const Eigen::Vector3d& point_wrt_body,
//     const Eigen::Vector3d& fix_pos)
//     : PointVelocityConstraint(plant, body_name, point_wrt_body,
//                               Eigen::Matrix3d::Identity(), fix_pos, fix_pos) {}

// template <typename T>
// PointVelocityConstraint<T>::PointVelocityConstraint(
//     const drake::multibody::MultibodyPlant<T>& plant,
//     const std::string& body_name, const Eigen::Vector3d& point_wrt_body,
//     const Eigen::Matrix<double, Eigen::Dynamic, 3>& dir,
//     const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
//     const std::string& description)
//     : solvers::NonlinearConstraint<T>(
//           dir.rows(), plant.num_positions(), lb, ub,
//           description.empty() ? body_name + "_vel_constraint" : description),
//       plant_(plant),
//       body_(plant.GetBodyByName(body_name)),
//       point_wrt_body_(point_wrt_body.template cast<T>()),
//       dir_(dir.template cast<T>()),
//       context_(plant_.CreateDefaultContext()) {}

// template <typename T>
// void PointVelocityConstraint<T>::EvaluateConstraint(
//     const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
//   plant_.SetPositionsAndVelocities(context_.get(), x);

//   drake::MatrixX<T> J(3, plant_.num_velocities());
//   plant_.CalcJacobianTranslationalVelocity(
//       *context_, drake::multibody::JacobianWrtVariable::kV, body_.body_frame(),
//       point_wrt_body_, plant_.world_frame(), plant_.world_frame(), &J);

//   *y = dir_ * J * x.tail(plant_.num_velocities());
// };

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::trajectory_optimization::QuaternionNormConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::trajectory_optimization::DirconCollocationConstraint)

// Explicitly instantiates on the most common scalar types.
// template class QuaternionNormConstraint<double>;
// template class QuaternionNormConstraint<AutoDiffXd>;
// template class DirconDynamicConstraint<double>;
// template class DirconDynamicConstraint<AutoDiffXd>;
// template class DirconKinematicConstraint<double>;
// template class DirconKinematicConstraint<AutoDiffXd>;
// template class DirconImpactConstraint<double>;
// template class DirconImpactConstraint<AutoDiffXd>;
// template class PointPositionConstraint<double>;
// template class PointPositionConstraint<AutoDiffXd>;
// template class PointVelocityConstraint<double>;
// template class PointVelocityConstraint<AutoDiffXd>;

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
