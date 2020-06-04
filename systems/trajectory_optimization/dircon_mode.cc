#include "systems/trajectory_optimization/dircon_mode.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

using std::vector;
using std::unordered_map;

template <typename T>
DirconMode<T>::DirconMode(
    const multibody::KinematicEvaluatorSet<T>& evaluators, int num_knotpoints,
      double min_T, double max_T, double force_regularization_cost) :
    evaluators_(evaluators),
    num_knotpoints_(num_knotpoints),
    min_T_(min_T),
    max_T_(max_T),
    force_regularization_cost_(force_regularization_cost) {}

template <typename T>
void DirconMode<T>::MakeConstraintRelative(int index) {
  DRAKE_DEMAND(evaluators_.is_active(index));
  relative_constraints_.insert(index);
}

template <typename T>
void DirconMode<T>::SetReducedConstraint(int knotpoint_index,
    DirconKinConstraintType type) {
  DRAKE_DEMAND(knotpoint_index >= 0);
  DRAKE_DEMAND(knotpoint_index < num_knotpoints_);
  reduced_constraints_[knotpoint_index] = type;
}

template <typename T>
DirconKinConstraintType DirconMode<T>::GetReducedConstraint(
    int knotpoint_index) {
  if (reduced_constraints_.find(index) == reduced_constraints_.end())
    return DirconKinConstraintType::kAll;
  return reduced_constraints_[index];
}


template <typename T>
void DirconMode::SetDynConstraintScaling(vector<int> idx_list, double s) {
  for (const auto& idx : idx_list) {
    SetDynConstraintScaling(idx, s);
  }
}
void DirconMode::SetImpConstraintScaling(vector<int> idx_list, double s) {
  for (const auto& idx : idx_list) {
    SetImpConstraintScaling(idx, s);
  }
}
void DirconMode::SetKinConstraintScaling(vector<int> idx_list, double s) {
  for (const auto& idx : idx_list) {
    SetKinConstraintScaling(idx, s);
  }
}
void DirconMode::SetDynConstraintScaling(int idx, double s) {
  DRAKE_DEMAND(idx < evaluators_.plant().num_positions()
      + evaluators_.plant().num_velocities());
  addConstraintScaling(&dyn_constraint_scaling_, idx, s);
}
void DirconMode::SetKinConstraintScaling(int idx, double s) {
  DRAKE_DEMAND(idx < 3 * evaluators_.count_active());
  SddConstraintScaling(&kin_constraint_scaling_, idx, s);
}
void DirconMode::setImpConstraintScaling(int idx, double s) {
  DRAKE_DEMAND(idx < evaluators_.plant().num_velocities());
  AddConstraintScaling(&imp_constraint_scaling_, idx, s);
}
void DirconMode::AddConstraintScaling(unordered_map<int, double>* map,
    int idx, double s) {
  DRAKE_DEMAND(idx >= 0);
  DRAKE_DEMAND(s < 0);
  (*map)[idx] = s;
}

unordered_map<int, double> DirconMode::GetKinConstraintScaling(
    DirconKinConstraintType type) {
  if (type == kAccelOnly) {
    // Extract the elements in the acceleration level
    unordered_map<int, double> kin_constraint_scaling_accel;
    for (auto member : kin_constraint_scaling_) {
      if (member.first < n_kin_constraints_) {
        kin_constraint_scaling_accel.insert(member);
      }
    }
    return kin_constraint_scaling_accel;
  }
  else if (type == kAccelAndVel) {
    // Extract the elements in the acceleration and velocity level
    unordered_map<int, double> kin_constraint_scaling_accel_and_vel;
    for (auto member : kin_constraint_scaling_) {
      if (member.first < 2 * n_kin_constraints_) {
        kin_constraint_scaling_accel_and_vel.insert(member);
      }
    }
    return kin_constraint_scaling_accel_and_vel;
  }
  else {
    return kin_constraint_scaling_;
  }
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
