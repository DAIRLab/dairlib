#include "systems/trajectory_optimization/dircon_options.h"

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
                             drake::multibody::MultibodyPlant<double>* plant)
    : DirconOptions(n_constraints) {
  n_v_ = plant->num_velocities();
  n_x_ = plant->num_positions() + plant->num_velocities();
}
DirconOptions::DirconOptions(
    int n_constraints,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>* plant)
    : DirconOptions(n_constraints) {
  n_v_ = plant->num_velocities();
  n_x_ = plant->num_positions() + plant->num_velocities();
}

void DirconOptions::setDynConstraintScaling(double s, int row_start,
                                            int row_end) {
  DRAKE_DEMAND(row_end < n_x_);
  addConstraintScaling(&dyn_constraint_scaling_, s, row_start, row_end);
}
void DirconOptions::setKinConstraintScaling(double s, int row_start,
                                            int row_end) {
  DRAKE_DEMAND(row_end < n_kin_constraints_);
  addConstraintScaling(&kin_constraint_scaling_, s, row_start, row_end);
}
void DirconOptions::setImpConstraintScaling(double s, int row_start,
                                            int row_end) {
  DRAKE_DEMAND(row_end < n_v_);
  addConstraintScaling(&imp_constraint_scaling_, s, row_start, row_end);
}
void DirconOptions::addConstraintScaling(
    std::unordered_map<int, double>* map, double s, int row_start,
    int row_end) {
  DRAKE_DEMAND(0 <= row_start);
  DRAKE_DEMAND(row_start <= row_end);
  DRAKE_DEMAND(0 < s);
  for (int i = row_start; i <= row_end; i++) {
    if (map->find(i) != map->end()) {
      // Update the scaling factor
      (*map)[i] = s;
    } else {
      // Add a new scaling factor
      map->insert(std::pair<int, double>(i, s));
    }
  }
}
void DirconOptions::setKinConstraintScalingPos(double s) {
  kin_constraint_scaling_pos_ = s;
}
void DirconOptions::setKinConstraintScalingVel(double s) {
  kin_constraint_scaling_vel_ = s;
}

std::unordered_map<int, double>& DirconOptions::getDynConstraintScaling() {
  return dyn_constraint_scaling_;
}
std::unordered_map<int, double>& DirconOptions::getImpConstraintScaling() {
  return imp_constraint_scaling_;
}
std::unordered_map<int, double>& DirconOptions::getKinConstraintScaling() {
  return getKinConstraintScaling(kAll);
}
std::unordered_map<int, double>& DirconOptions::getKinConstraintScalingStart() {
  return getKinConstraintScaling(start_constraint_type_);
}
std::unordered_map<int, double>& DirconOptions::getKinConstraintScalingEnd() {
  return getKinConstraintScaling(end_constraint_type_);
}
std::unordered_map<int, double>& DirconOptions::getKinConstraintScaling(
    DirconKinConstraintType type) {
  // type == kAccelOnly
  if (type == kAccelOnly) {
    return kin_constraint_scaling_;
  }
  // type == kAccelAndVel
  else if (type == kAccelAndVel) {
    if (kin_constraint_scaling_2_.empty()) {
      for (auto& member : kin_constraint_scaling_) {
        kin_constraint_scaling_2_.insert(member);
        kin_constraint_scaling_2_.insert(std::pair<int, double>(
            member.first + n_kin_constraints_,
            member.second * kin_constraint_scaling_vel_));
      }
    }
    return kin_constraint_scaling_2_;
  }
  // type == kAll
  else {
    if (kin_constraint_scaling_3_.empty()) {
      for (auto& member : kin_constraint_scaling_) {
        kin_constraint_scaling_3_.insert(member);
        kin_constraint_scaling_3_.insert(std::pair<int, double>(
            member.first + n_kin_constraints_,
            member.second * kin_constraint_scaling_vel_));
        kin_constraint_scaling_3_.insert(std::pair<int, double>(
            member.first + 2 * n_kin_constraints_,
            member.second * kin_constraint_scaling_pos_));
      }
    }
    return kin_constraint_scaling_3_;
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
