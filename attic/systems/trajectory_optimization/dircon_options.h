#pragma once

#include <vector>

#include "attic/systems/trajectory_optimization/dircon_opt_constraints.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

class DirconOptions {
 private:
  int n_constraints_;
  std::vector<bool> is_constraints_relative_;
  DirconKinConstraintType start_constraint_type_;
  DirconKinConstraintType end_constraint_type_;
  double force_cost_;

 public:
  explicit DirconOptions(int n_constraints);

  void setAllConstraintsRelative(bool relative);
  void setConstraintRelative(int index, bool relative);
  void setStartType(DirconKinConstraintType type);
  void setEndType(DirconKinConstraintType type);
  void setForceCost(double force_cost);

  int getNumConstraints();
  bool getSingleConstraintRelative(int index);
  std::vector<bool> getConstraintsRelative();
  DirconKinConstraintType getStartType();
  DirconKinConstraintType getEndType();
  double getForceCost();
  int getNumRelative();
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
