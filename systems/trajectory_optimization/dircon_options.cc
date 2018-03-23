#include "dircon_options.h"

using std::vector;

namespace drake{
DirconOptions::DirconOptions(int n_constraints) {
  n_constraints_ = n_constraints;
  is_constraints_relative_ = vector<bool>(n_constraints_);
  for (int i=0; i < n_constraints_; i++) {
    is_constraints_relative_[i] = false;
  }
  bool constrain_start_ = true;
  bool constrain_end_ = true;
  bool constrain_phi_start_ = true;
  bool constrain_phi_end_ = true;
  accel_cost_ = 0;
}

void DirconOptions::setAllConstraintsRelative(bool relative) {
  for (int i=0; i < n_constraints_; i++) {
    is_constraints_relative_[i] = relative;
  }
}

void DirconOptions::setConstraintRelative(int index, bool relative) {
  is_constraints_relative_[index]  = relative;
}

void DirconOptions::setConstraintStart(bool constrain_start) {
  constrain_start_ = constrain_start;
}

void DirconOptions::setConstraintEnd(bool constrain_end) {
  constrain_end_ = constrain_end;
}

void DirconOptions::setConstraintPhiStart(bool constrain_phi_start) {
  constrain_phi_start_ = constrain_phi_start;
}

void DirconOptions::setConstraintPhiEnd(bool constrain_phi_end) {
  constrain_phi_end_ = constrain_phi_end;
}

void DirconOptions::setAccelCost(double accel_cost) {
  accel_cost_ = accel_cost;
}

int DirconOptions::getNumConstraints() {
  return n_constraints_;
}

bool DirconOptions::getSingleConstraintRelative(int index) {
  return is_constraints_relative_[index];
}

bool DirconOptions::getConstrainStart() {
  return constrain_start_;
}

bool DirconOptions::getConstrainEnd() {
  return constrain_end_;
}

bool DirconOptions::getConstrainPhiStart() {
  return constrain_phi_start_;
}

bool DirconOptions::getConstrainPhiEnd() {
  return constrain_phi_end_;

}
double DirconOptions::getAccelCost() {
  return accel_cost_;
}
}