#include "dircon_options.h"

using std::vector;

namespace drake{
namespace systems{
namespace trajectory_optimization{

DirconOptions::DirconOptions(int n_constraints) {
  n_constraints_ = n_constraints;
  is_constraints_relative_ = vector<bool>(n_constraints_);
  for (int i=0; i < n_constraints_; i++) {
    is_constraints_relative_[i] = false;
  }
  start_constraint_type_ = DirconKinConstraintType::kAll;
  end_constraint_type_ = DirconKinConstraintType::kAll;
  force_cost_ = 1.0e-6;
}

void DirconOptions::setAllConstraintsRelative(bool relative) {
  for (int i=0; i < n_constraints_; i++) {
    is_constraints_relative_[i] = relative;
  }
}

void DirconOptions::setConstraintRelative(int index, bool relative) {
  is_constraints_relative_[index]  = relative;
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

int DirconOptions::getNumConstraints() {
  return n_constraints_;
}

bool DirconOptions::getSingleConstraintRelative(int index) {
  return is_constraints_relative_[index];
}

DirconKinConstraintType DirconOptions::getStartType() {
  return start_constraint_type_;
}

DirconKinConstraintType DirconOptions::getEndType() {
  return end_constraint_type_;
}

double DirconOptions::getForceCost() {
  return force_cost_;
}

}
}
}