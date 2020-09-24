#pragma once

#include <vector>
#include <unordered_map>

#include "systems/trajectory_optimization/dircon_opt_constraints.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

class DirconOptions {
 public:
  explicit DirconOptions(int n_constraints);
  DirconOptions(int n_constraints,
                const drake::multibody::MultibodyPlant<double>& plant);
  DirconOptions(
      int n_constraints,
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant);

  // Setters/getters for constraint scaling
  /// The impact constraint is for the impact at the beginning of the mode
  void setDynConstraintScaling(int idx, double s);
  void setImpConstraintScaling(int idx, double s);
  void setKinConstraintScaling(int idx, double s);
  void setDynConstraintScaling(std::vector<int> idx_list, double s);
  void setImpConstraintScaling(std::vector<int> idx_list, double s);
  void setKinConstraintScaling(std::vector<int> idx_list, double s);
  const std::unordered_map<int, double>& getDynConstraintScaling();
  const std::unordered_map<int, double>& getImpConstraintScaling();
  std::unordered_map<int, double> getKinConstraintScaling();
  std::unordered_map<int, double> getKinConstraintScalingStart();
  std::unordered_map<int, double> getKinConstraintScalingEnd();

  // Setters/getters for relativity of kinematic constraint
  void setAllConstraintsRelative(bool relative);
  void setConstraintRelative(int index, bool relative);
  bool getSingleConstraintRelative(int index);
  std::vector<bool> getConstraintsRelative();
  int getNumRelative();

  // Setters/getters for start type and end type of kinematic constraint
  void setStartType(DirconKinConstraintType type);
  void setEndType(DirconKinConstraintType type);
  DirconKinConstraintType getStartType();
  DirconKinConstraintType getEndType();

  // Getter for size of kinematic constraint
  int getNumConstraints();

  // Setter/getter for force cost
  void setForceCost(double force_cost);
  double getForceCost();

 private:
  // methods for constraint scaling
  static void addConstraintScaling(std::unordered_map<int, double>* list,
                                   int idx, double s);
  std::unordered_map<int, double> getKinConstraintScaling(
      DirconKinConstraintType type);

  // Constraint scaling
  std::unordered_map<int, double> dyn_constraint_scaling_;
  std::unordered_map<int, double> imp_constraint_scaling_;
  std::unordered_map<int, double> kin_constraint_scaling_;
  int n_v_ = -1;
  int n_x_ = -1;

  // Kinematic constraints
  int n_kin_constraints_;
  std::vector<bool> is_constraints_relative_;
  DirconKinConstraintType start_constraint_type_;
  DirconKinConstraintType end_constraint_type_;

  // Force cost
  double force_cost_;
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
