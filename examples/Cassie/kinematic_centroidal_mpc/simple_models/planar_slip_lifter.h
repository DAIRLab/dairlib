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

class PlanarSlipLifter {
 public:
  PlanarSlipLifter(const drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* context,
                   const std::vector<dairlib::multibody::WorldPointEvaluator<double>>& slip_contact_points,
                   const std::vector<dairlib::multibody::WorldPointEvaluator<double>>& complex_contact_points,
                   const std::map<int, std::vector<int>>& simple_foot_index_to_complex_foot_index,
                   const drake::VectorX<double>& nominal_stand,
                   double k,
                   double r0,
                   const std::vector<double>& stance_widths);

  void Lift(const Eigen::Ref<const drake::VectorX<double>> &slip_state,
            drake::VectorX<double> *complex_state) const;

  drake::VectorX<double> Lift(const Eigen::Ref<const drake::VectorX<double>> &slip_state) const;

 private:
  /*!
   * @brief given the center of mass position and slip feet position solve for the generalized position of the full robot
   * @note assumes identity orientation
   * @note implements numerical ik (can be slow)
   * @param com_position center of mass position in the world
   * @param slip_feet_positions [2*n_slip_feet] locations of the planar slip feet
   * @return the generalized positions
   */
  drake::VectorX<double> LiftGeneralizedPosition(const drake::Vector3<double>& com_position, const drake::VectorX<double>& slip_feet_positions) const;

  /*!
   * @brief Given a generalized position calculate the generalized velocity that is the least squares solution to tracking the
   * linear momentum and foot velocity with 0 floating base angular velocity.
   * @note Jacobian for momentum is calculated numerically
   * @param generalized_pos
   * @param linear_momentum
   * @param com_pos
   * @param slip_feet_velocities [2*n_slip_feet] the foot velocity
   * @return generalized velocities
   */
  drake::VectorX<double> LiftGeneralizedVelocity(const drake::VectorX<double>& generalized_pos,
                                            const drake::Vector3<double>& linear_momentum,
                                            const drake::Vector3<double>& com_pos,
                                            const drake::VectorX<double>& slip_feet_velocities) const;

  drake::VectorX<double> LiftContactPos(const drake::VectorX<double>& generalized_position) const;

  drake::VectorX<double> LiftContactVel(const drake::VectorX<double>& generalized_pos, const drake::VectorX<double>& generalized_vel) const;

  drake::VectorX<double> LiftGrf(const drake::VectorX<double>& com_pos,
                            const drake::VectorX<double>& slip_feet_pos,
                            const drake::VectorX<double>& complex_contact_point_pos) const;


  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable drake::systems::Context<double>* context_;
  mutable drake::multibody::InverseKinematics ik_;
  const double m_;
  const double k_;
  const double r0_;
  const std::vector<double> stance_widths_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<double>> slip_contact_points_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<double>> complex_contact_points_;
  const std::map<int, std::vector<int>> simple_foot_index_to_complex_foot_index_;
  const int n_q_;
  const int n_v_;

  drake::solvers::VectorXDecisionVariable com_vars_;

  const int kSLIP_DIM = 2;
  const Eigen::VectorXi slip_index_ = {0,2};
};

