#pragma once

#include <vector>

#include "systems/trajectory_optimization/dircon_opt_constraints.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

class DirconOptions {
 public:
  explicit DirconOptions(int n_constraints);
  DirconOptions(int n_constraints,
                drake::multibody::MultibodyPlant<double>* plant);
  DirconOptions(int n_constraints,
                drake::multibody::MultibodyPlant<drake::AutoDiffXd>* plant);

  // Setters/getters for constraint scaling
  /// The impact constraint is for the impact at the beginning of the mode
  void setDynConstraintScaling(double scale, int row_start, int row_end);
  void setImpConstraintScaling(double scale, int row_start, int row_end);
  void setKinConstraintScaling(double scale, int row_start, int row_end);
  const std::unordered_map<int, double>& getDynConstraintScaling();
  const std::unordered_map<int, double>& getImpConstraintScaling();
  const std::unordered_map<int, double>& getKinConstraintScaling();
  const std::unordered_map<int, double>& getKinConstraintScalingStart();
  const std::unordered_map<int, double>& getKinConstraintScalingEnd();

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

  // Setter/getter for is_single_periodic_end_node_ flag
  void setSinglePeriodicEndNode(bool is_single_periodic_end_node);
  bool isSinglePeriodicEndNode() const { return is_single_periodic_end_node_; }

  // Getter for size of kinematic constraint
  int getNumConstraints();

  // Setter/getter for force cost
  void setForceCost(double force_cost);
  double getForceCost();

 private:
  // methods for constraint scaling
  static void addConstraintScaling(std::unordered_map<int, double>* list,
                                   double scale, int row_start, int row_end);
  const std::unordered_map<int, double>& getKinConstraintScaling(
      DirconKinConstraintType type);

  // Constraint scaling
  std::unordered_map<int, double> dyn_constraint_scaling_;
  std::unordered_map<int, double> imp_constraint_scaling_;
  std::unordered_map<int, double> kin_constraint_scaling_accel_;
  std::unordered_map<int, double> kin_constraint_scaling_accel_vel_;
  std::unordered_map<int, double> kin_constraint_scaling_accel_vel_pos_;
  int n_v_ = -1;
  int n_x_ = -1;

  // Kinematic constraints
  int n_kin_constraints_;
  std::vector<bool> is_constraints_relative_;
  DirconKinConstraintType start_constraint_type_;
  DirconKinConstraintType end_constraint_type_;

  // Force cost
  double force_cost_;

  // is_single_periodic_end_node_ is set true when
  //  1. the mode with this flag being set to true is the last mode of the
  //  hybrid system,
  //  2. the # of nodes in this mode is 1,
  //  3. and the user want to impose the constraint on the state at this node
  //  himself/herself.
  // If is_single_periodic_end_node_ is true, Dircon will impose impact
  //  constraint but not kinematics constraint.
  // One use case:
  //  When getting periodic walking gait of a bipedal robot, the user only
  //  implement one stride in Dircon and impose a periodic (and mirror)
  //  constraint between the states at the first node and the end node. In this
  //  case, since the state at the end node (both position and velocity) is
  //  constrained by the first node (and hence the kinematic constraint at the
  //  first node as well), there is no need to impose kinematic constraint on
  //  the end node.
  bool is_single_periodic_end_node_ = false;
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
