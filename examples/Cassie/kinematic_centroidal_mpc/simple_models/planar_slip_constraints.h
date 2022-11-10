#pragma  once

#include <drake/multibody/plant/multibody_plant.h>
#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"
#include "solvers/nonlinear_constraint.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include <drake/multibody/inverse_kinematics/com_position_constraint.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include "examples/Cassie/kinematic_centroidal_mpc/simple_models/planar_slip_lifter.h"

template <typename T>
class PlanarSlipReductionConstraint : public dairlib::solvers::NonlinearConstraint<T> {

 public:
  PlanarSlipReductionConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                                drake::systems::Context<T>* context,
                                const std::vector<dairlib::multibody::WorldPointEvaluator<T>>& slip_feet,
                                int complex_state_size, int knot_index);

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

  const T m_;
  drake::systems::Context<T>* context_;
  const drake::multibody::MultibodyPlant<T>& plant_;
  int nx_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<T>> slip_feet_;
  const int kSLIP_DIM = 2;
  const Eigen::Vector2i slip_index_{0,2};
};

class PlanarSlipLiftingConstraint : public dairlib::solvers::NonlinearConstraint<double> {

 public:
  PlanarSlipLiftingConstraint(const drake::multibody::MultibodyPlant<double>& plant,
                              drake::systems::Context<double>* context,
                              const std::vector<dairlib::multibody::WorldPointEvaluator<double>>& slip_contact_points,
                              const std::vector<dairlib::multibody::WorldPointEvaluator<double>>& complex_contact_points,
                              const std::map<int, std::vector<int>>& simple_foot_index_to_complex_foot_index,
                              const drake::VectorX<double>& nominal_stand,
                              double k,
                              double r0,
                              const std::vector<double>& stance_widths, int knot_index);

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override;

  PlanarSlipLifter lifting_function_;
  const int slip_dim_;
  const int complex_dim_;
};