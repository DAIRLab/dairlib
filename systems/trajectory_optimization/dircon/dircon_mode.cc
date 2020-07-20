#include "systems/trajectory_optimization/dircon/dircon_mode.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

using drake::multibody::MultibodyPlant;
using std::vector;
using std::unordered_map;

template <typename T>
DirconMode<T>::DirconMode(
    const multibody::KinematicEvaluatorSet<T>& evaluators, int num_knotpoints,
      double min_T, double max_T, double force_regularization_) :
    evaluators_(evaluators),
    num_knotpoints_(num_knotpoints),
    min_T_(min_T),
    max_T_(max_T),
    force_regularization_(force_regularization_) {}

template <typename T>
void DirconMode<T>::MakeConstraintRelative(int evaluator_index,
    int constraint_index) {
  DRAKE_DEMAND(constraint_index >= 0);
  DRAKE_DEMAND(constraint_index <=
      evaluators_.get_evaluator(evaluator_index).num_full());
  int active_constraint_index = evaluators_.get_evaluator(
      evaluator_index).full_index_to_active_index(constraint_index);
  if (active_constraint_index != -1) {
    int total_index = evaluators_.evaluator_active_start(evaluator_index)
        + active_constraint_index;
    relative_constraints_.insert(total_index);
  }
}

template <typename T>
void DirconMode<T>::SkipQuaternionConstraint(int knotpoint_index) {
  skip_quaternion_.insert(knotpoint_index);
}

template <typename T>
bool DirconMode<T>::IsSkipQuaternionConstraint(int knotpoint_index) const {
  return skip_quaternion_.find(knotpoint_index) != skip_quaternion_.end();
}

template <typename T>
void DirconMode<T>::set_constraint_type(int knotpoint_index,
    KinematicConstraintType type) {
  DRAKE_DEMAND(knotpoint_index >= 0);
  DRAKE_DEMAND(knotpoint_index < num_knotpoints_);
  reduced_constraints_[knotpoint_index] = type;
}

template <typename T>
KinematicConstraintType DirconMode<T>::get_constraint_type(
    int knotpoint_index) const {
  if (reduced_constraints_.find(knotpoint_index) == reduced_constraints_.end())
    return KinematicConstraintType::kAll;
  return reduced_constraints_.at(knotpoint_index);
}


template <typename T>
void DirconMode<T>::SetImpactScale(int velocity_index, double scale) {
  DRAKE_DEMAND(velocity_index >= 0);
  DRAKE_DEMAND(velocity_index < evaluators_.plant().num_velocities());
  impact_scale_[velocity_index] = scale;
}

template <typename T>
void DirconMode<T>::SetDynamicsScale(int state_index, double scale) {
  DRAKE_DEMAND(state_index >= 0);
  DRAKE_DEMAND(state_index < evaluators_.plant().num_velocities()
      + evaluators_.plant().num_positions());
  dynamics_scale_[state_index] = scale;
}


template <typename T>
void DirconMode<T>::SetKinPositionScale(int evaluator_index,
    int constraint_index, double scale) {
  SetKinScale(&kin_position_scale_, evaluator_index, constraint_index, scale);
}

template <typename T>
void DirconMode<T>::SetKinVelocityScale(int evaluator_index,
    int constraint_index, double scale) {
  SetKinScale(&kin_velocity_scale_, evaluator_index, constraint_index, scale);
}

template <typename T>
void DirconMode<T>::SetKinAccelerationScale(int evaluator_index,
    int constraint_index, double scale) {
  SetKinScale(&kin_acceleration_scale_, evaluator_index, constraint_index,
      scale);
}

template <typename T>
void DirconMode<T>::SetImpactScale(std::vector<int> velocity_indices,
    double scale) {
  for (const auto& idx : velocity_indices) {
    SetImpactScale(idx, scale);
  }
}

template <typename T>
void DirconMode<T>::SetDynamicsScale(std::vector<int> state_indices,
    double scale) {
  for (const auto& idx : state_indices) {
    SetDynamicsScale(idx, scale);
  }
}

template <typename T>
void DirconMode<T>::SetKinPositionScale(std::vector<int> evaluator_indices,
      std::vector<int> constraint_indices, double scale) {
  for (const auto& evaluator_index : evaluator_indices) {
    for (const auto& constraint_index : constraint_indices) {
      SetKinPositionScale(evaluator_index, constraint_index, scale);
    }    
  }
}

template <typename T>
void DirconMode<T>::SetKinAccelerationScale(std::vector<int> evaluator_indices,
      std::vector<int> constraint_indices, double scale) {
  for (const auto& evaluator_index : evaluator_indices) {
    for (const auto& constraint_index : constraint_indices) {
      SetKinAccelerationScale(evaluator_index, constraint_index, scale);
    }    
  }
}

template <typename T>
void DirconMode<T>::SetKinVelocityScale(std::vector<int> evaluator_indices,
      std::vector<int> constraint_indices, double scale) {
  for (const auto& evaluator_index : evaluator_indices) {
    for (const auto& constraint_index : constraint_indices) {
      SetKinVelocityScale(evaluator_index, constraint_index, scale);
    }    
  }
}


template <typename T>
void DirconMode<T>::SetKinScale(std::unordered_map<int, double>* scale_map,
    int evaluator_index, int constraint_index, double scale) {
  DRAKE_DEMAND(evaluator_index >= 0);
  DRAKE_DEMAND(evaluator_index < evaluators_.num_evaluators());
  DRAKE_DEMAND(constraint_index >= 0);
  DRAKE_DEMAND(constraint_index <=
      evaluators_.get_evaluator(evaluator_index).num_full());
  int active_constraint_index = evaluators_.get_evaluator(
      evaluator_index).full_index_to_active_index(constraint_index);
  if (active_constraint_index != -1) {
    int total_index = evaluators_.evaluator_active_start(evaluator_index) + 
        active_constraint_index;
    (*scale_map)[total_index] = scale;
  }
}

template <typename T>
DirconModeSequence<T>::DirconModeSequence(const MultibodyPlant<T>& plant)
    : plant_(plant) {}

template <typename T>
DirconModeSequence<T>::DirconModeSequence(DirconMode<T>* mode)
    : plant_(mode->evaluators().plant()) {
  AddMode(mode);
}

template <typename T>
void DirconModeSequence<T>::AddMode(DirconMode<T>* mode) {
  DRAKE_DEMAND(&mode->plant() == &plant_);
  modes_.push_back(mode);
}

template <typename T>
int DirconModeSequence<T>::count_knotpoints() const {
  int sum = 1 - modes_.size(); // subtract shared knotpoints
  for (const auto& mode : modes_) {
    sum += mode->num_knotpoints();
  }
  return sum;
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::trajectory_optimization::DirconMode)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::trajectory_optimization::DirconModeSequence)
