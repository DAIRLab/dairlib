#pragma once
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

class PlanarSlipReducer {
 public:
  PlanarSlipReducer(const drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* context,
                   const std::vector<dairlib::multibody::WorldPointEvaluator<double>>& slip_contact_points,
                   const std::vector<dairlib::multibody::WorldPointEvaluator<double>>& complex_contact_points,
                   const std::map<int, std::vector<int>>& simple_foot_index_to_complex_foot_index,
                   double k,
                   double r0);

  void Reduce(const Eigen::Ref<const drake::VectorX<double>> &complex_state,
            drake::VectorX<double> *slip_state) const;

  drake::VectorX<double> Reduce(const Eigen::Ref<const drake::VectorX<double>> &complex_state) const;

  drake::VectorX<double> ReduceGrf(const Eigen::Ref<const drake::VectorX<double>> &complex_com,
                                   const Eigen::Ref<const drake::VectorX<double>> &slip_contact_pos,
                                   const Eigen::Ref<const drake::VectorX<double>> &complex_grf) const;
 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable drake::systems::Context<double>* context_;
  const double k_;
  const double r0_;
  const double m_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<double>> slip_contact_points_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<double>> complex_contact_points_;
  const std::map<int, std::vector<int>> simple_foot_index_to_complex_foot_index_;
  const int n_q_;
  const int n_v_;

  const int kSLIP_DIM = 3;
};

