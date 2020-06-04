#pragma once
#include <map>
#include <set>

#include "multibody/kinematic/kinematic_evaluator_set.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

enum DirconKinConstraintType { kAll = 3, kAccelAndVel = 2, kAccelOnly = 1 };

template <typename T>
class DirconMode {
 public:
  explicit DirconMode(
      const multibody::KinematicEvaluatorSet<T>& evaluators, int num_knotpoints,
      double min_T = 0, double max_T = std::numeric_limits<double>::infinity(),
      double force_regularization_cost_ = 1.0e-4);

  void MakeConstraintRelative(int index);
  
  const std::set<int>& RelativeConstraints() { return relative_constraints_; };
  
  int CountRelativeConstraints() { return relative_constraints_.size(); };

  void SetReducedConstraint(int knotpoint_index, DirconKinConstraintType type);

  DirconKinConstraintType GetReducedConstraint(int knotpoint_index);

  // Setters/getters for constraint scaling
  /// The impact constraint is for the impact at the beginning of the mode
  void SetDynConstraintScaling(int idx, double s);
  void SetImpConstraintScaling(int idx, double s);
  void SetKinPositionConstraintScaling(int idx, double s);
  void SetKinVelocityConstraintScaling(int idx, double s);
  void SetKinAccelConstraintScaling(int idx, double s);
  void SetDynConstraintScaling(std::vector<int> idx_list, double s);
  void SetImpConstraintScaling(std::vector<int> idx_list, double s);
  void SetKinPositionConstraintScaling(std::vector<int> idx_list, double s);
  void SetKinVelocityConstraintScaling(std::vector<int> idx_list, double s);
  void SetKinAccelConstraintScaling(std::vector<int> idx_list, double s);

  const std::unordered_map<int, double>& GetDynConstraintScaling() {
    return dyn_constraint_scaling_;
  };
  const std::unordered_map<int, double>& GetImpConstraintScaling() {
    return imp_constraint_scaling_;
  };
  std::unordered_map<int, double> GetKinConstraintScaling();
  std::unordered_map<int, double> GetKinConstraintScalingStart();
  std::unordered_map<int, double> GetKinConstraintScalingEnd();


  // Getter for size of kinematic constraint
  int getNumConstraints();

  double getForceCost();

 private:
  // methods for constraint scaling
  static void addConstraintScaling(std::unordered_map<int, double>* list,
                                   int idx, double s);
  std::unordered_map<int, double> getKinConstraintScaling(
      DirconKinConstraintType type);

  const multibody::KinematicEvaluatorSet<T>& evaluators_;
  int num_knotpoints_;
  double max_T_;
  double min_T_;
  std::set<int> relative_constraints_;
  std::unordered_map<int, DirconKinConstraintType> reduced_constraints_;
  double force_regularization_cost_;

  // Constraint scaling
  std::unordered_map<int, double> dyn_constraint_scaling_;
  std::unordered_map<int, double> imp_constraint_scaling_;
  std::unordered_map<int, double> kin_constraint_scaling_;
  int n_v_ = -1;
  int n_x_ = -1;
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
