#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "examples/Cassie/kinematic_centroidal_planner/simple_models/slip_lifter.h"
#include "examples/Cassie/kinematic_centroidal_planner/simple_models/slip_reducer.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/nonlinear_constraint.h"
#include "solvers/nonlinear_cost.h"

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/constraint.h"

class SlipReductionConstraint
    : public dairlib::solvers::NonlinearConstraint<double> {
 public:
  SlipReductionConstraint(const drake::multibody::MultibodyPlant<double>& plant,
                          std::shared_ptr<SlipReducer> reducing_function,
                          int n_slip_feet, int n_complex_feet, int knot_index);

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override;

  std::shared_ptr<SlipReducer> reducing_function_;
  const int slip_dim_;
  const int complex_dim_;
};

class SlipGrfReductionConstrain
    : public dairlib::solvers::NonlinearConstraint<double> {
 public:
  SlipGrfReductionConstrain(
      const drake::multibody::MultibodyPlant<double>& plant,
      std::shared_ptr<SlipReducer> reducing_function, int n_slip_feet,
      int n_complex_feet, int knot_index);

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override;

  std::shared_ptr<SlipReducer> reducing_function_;
  const int n_slip_feet_;
  const int n_complex_feet_;
};

template <typename T>
class SlipDynamicsConstraint : public dairlib::solvers::NonlinearConstraint<T> {
 public:
  SlipDynamicsConstraint(double r0, double k, double b, T m, int n_feet,
                         std::vector<bool> contact_mask0,
                         std::vector<bool> contact_mask1, double dt,
                         int knot_index);

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

  drake::VectorX<T> CalcTimeDerivativesWithForce(
      const drake::VectorX<T>& com_position, const drake::VectorX<T>& com_vel,
      const drake::VectorX<T>& contact_loc, const drake::VectorX<T>& slip_force,
      const std::vector<bool>& contact_mask) const;

  const double r0_;
  const double k_;
  const double b_;
  const T m_;
  const int n_feet_;
  const std::vector<bool> contact_mask0_;
  const std::vector<bool> contact_mask1_;
  const double dt_;
};