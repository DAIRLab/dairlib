#include "systems/trajectory_optimization/dircon_options.h"

using std::unordered_map;
using std::vector;

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

DirconOptions::DirconOptions(int n_kin_constraints) {
  n_kin_constraints_ = n_kin_constraints;
  is_constraints_relative_ = vector<bool>(n_kin_constraints_, false);
  start_constraint_type_ = DirconKinConstraintType::kAll;
  end_constraint_type_ = DirconKinConstraintType::kAll;
  force_cost_ = 1.0e-4;
}
DirconOptions::DirconOptions(int n_constraints,
                             const drake::multibody::MultibodyPlant<double>& plant)
    : DirconOptions(n_constraints) {
  n_v_ = plant.num_velocities();
  n_x_ = plant.num_positions() + plant.num_velocities();
}
DirconOptions::DirconOptions(
    int n_constraints,
    const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant)
    : DirconOptions(n_constraints) {
  n_v_ = plant.num_velocities();
  n_x_ = plant.num_positions() + plant.num_velocities();
}

void DirconOptions::setDynConstraintScaling(double scale, int row_start,
                                            int row_end) {
  DRAKE_DEMAND(row_end < n_x_);
  addConstraintScaling(&dyn_constraint_scaling_, scale, row_start, row_end);
}
void DirconOptions::setKinConstraintScaling(double scale, int row_start,
                                            int row_end) {
  DRAKE_DEMAND(row_end < 3 * n_kin_constraints_);
  for (int i = row_start; i <= row_end; i++) {
    if (i < n_kin_constraints_) {
      addConstraintScaling(&kin_constraint_scaling_accel_vel_pos_, scale, i, i);
      addConstraintScaling(&kin_constraint_scaling_accel_vel_, scale, i, i);
      addConstraintScaling(&kin_constraint_scaling_accel_, scale, i, i);
    } else if (i < 2 * n_kin_constraints_) {
      addConstraintScaling(&kin_constraint_scaling_accel_vel_pos_, scale, i, i);
      addConstraintScaling(&kin_constraint_scaling_accel_vel_, scale, i, i);
    } else {
      addConstraintScaling(&kin_constraint_scaling_accel_vel_pos_, scale, i, i);
    }
  }
}
void DirconOptions::setImpConstraintScaling(double scale, int row_start,
                                            int row_end) {
  DRAKE_DEMAND(row_end < n_v_);
  addConstraintScaling(&imp_constraint_scaling_, scale, row_start, row_end);
}
void DirconOptions::addConstraintScaling(std::unordered_map<int, double>* list,
                                         double scale, int row_start,
                                         int row_end) {
  DRAKE_DEMAND(0 <= row_start);
  DRAKE_DEMAND(row_start <= row_end);
  for (int i = row_start; i <= row_end; i++) {
    // Check if the scaling has been set already
    for (const auto& member : *list) {
      DRAKE_DEMAND(i != member.first);
    }
    // Add scaling
    list->insert(std::pair<int, double>(i, scale));
  }
}

const unordered_map<int, double>& DirconOptions::getDynConstraintScaling() {
  return dyn_constraint_scaling_;
}
const unordered_map<int, double>& DirconOptions::getImpConstraintScaling() {
  return imp_constraint_scaling_;
}
const unordered_map<int, double>& DirconOptions::getKinConstraintScaling() {
  return getKinConstraintScaling(kAll);
}
const unordered_map<int, double>&
DirconOptions::getKinConstraintScalingStart() {
  return getKinConstraintScaling(start_constraint_type_);
}
const unordered_map<int, double>& DirconOptions::getKinConstraintScalingEnd() {
  return getKinConstraintScaling(end_constraint_type_);
}
const unordered_map<int, double>& DirconOptions::getKinConstraintScaling(
    DirconKinConstraintType type) {
  DRAKE_DEMAND((type == kAccelOnly) || (type == kAccelAndVel) ||
               (type == kAll));
  if (type == kAccelOnly) {
    return kin_constraint_scaling_accel_;
  } else if (type == kAccelAndVel) {
    return kin_constraint_scaling_accel_vel_;
  } else {
    return kin_constraint_scaling_accel_vel_pos_;
  }
}

void DirconOptions::setAllConstraintsRelative(bool relative) {
  for (int i = 0; i < n_kin_constraints_; i++) {
    is_constraints_relative_[i] = relative;
  }
}

void DirconOptions::setConstraintRelative(int index, bool relative) {
  is_constraints_relative_[index] = relative;
}

void DirconOptions::setStartType(DirconKinConstraintType type) {
  start_constraint_type_ = type;
}

void DirconOptions::setEndType(DirconKinConstraintType type) {
  end_constraint_type_ = type;
}

void DirconOptions::setForceCost(double force_cost) {
  force_cost_ = force_cost;
}

int DirconOptions::getNumConstraints() { return n_kin_constraints_; }

bool DirconOptions::getSingleConstraintRelative(int index) {
  return is_constraints_relative_[index];
}

std::vector<bool> DirconOptions::getConstraintsRelative() {
  return is_constraints_relative_;
}

DirconKinConstraintType DirconOptions::getStartType() {
  return start_constraint_type_;
}

DirconKinConstraintType DirconOptions::getEndType() {
  return end_constraint_type_;
}

double DirconOptions::getForceCost() { return force_cost_; }

int DirconOptions::getNumRelative() {
  return static_cast<int>(std::count(is_constraints_relative_.begin(),
                                     is_constraints_relative_.end(), true));
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
