#pragma once
#include <drake/multibody/inverse_kinematics/com_position_constraint.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/nonlinear_constraint.h"

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

/*!
 * @brief This class lifts the slip state and control to an equivalent kinematic
 * centroidal state and control. It assumes that there is zero orientation and
 * angular momentum
 *
 * @note Currently this class assumes the slip contact points or [ltoe, rtoe]
 * and the complex contact points are [ltoe, lheel, rtoe, rheel]
 */
class SlipLifter {
 public:
  /*!
   * @brief Constructor
   * @param plant
   * @param context
   * @param slip_contact_points vector of world point evaluators for slip
   * contact points
   * @param complex_contact_points vector of world point evaluators for complex
   * contact points
   * @param simple_foot_index_to_complex_foot_index map from slip contact point
   * index to vector of complex contact points on the same feet
   * @param nominal_stand
   * @param k
   * @param b
   * @param r0
   * @param contact_mask vector of booleans describing which slip contact points
   * are in stance
   */
  SlipLifter(const drake::multibody::MultibodyPlant<double>& plant,
             drake::systems::Context<double>* context,
             const std::vector<dairlib::multibody::WorldPointEvaluator<double>>&
                 slip_contact_points,
             const std::vector<dairlib::multibody::WorldPointEvaluator<double>>&
                 complex_contact_points,
             const std::map<int, std::vector<int>>&
                 simple_foot_index_to_complex_foot_index,
             const drake::VectorX<double>& nominal_stand, double k, double b,
             double r0, const std::vector<bool>& contact_mask);

  /// Input is of the form:
  ///     slip_com
  ///     slip_velocity
  ///     slip_contact_pos
  ///     slip_contact_vel
  ///     slip_force
  /// Output is of the form:
  ///     complex_com
  ///     complex_ang_momentum
  ///     complex_lin_momentum
  ///     complex_contact_pos
  ///     complex_contact_vel
  ///     complex_contact_force
  ///     complex_gen_pos
  ///     complex_gen_vel
  void Lift(const Eigen::Ref<const drake::VectorX<double>>& slip_state,
            drake::VectorX<double>* complex_state) const;

  drake::VectorX<double> Lift(
      const Eigen::Ref<const drake::VectorX<double>>& slip_state) const;

 private:
  /*!
   * @brief given the center of mass position and slip feet position solve for
   * the generalized position of the full robot
   * @note assumes identity orientation
   * @note implements numerical ik (can be slow)
   * @param com_position center of mass position in the world
   * @param slip_feet_positions [3*n_slip_feet] locations of the slip
   * feet
   * @return the generalized positions
   */
  drake::VectorX<double> LiftGeneralizedPosition(
      const drake::Vector3<double>& com_position,
      const drake::VectorX<double>& slip_feet_positions) const;

  /*!
   * @brief Given a generalized position calculate the generalized velocity that
   * is the least squares solution to tracking the linear momentum and foot
   * velocity with 0 floating base angular velocity.
   * @note Jacobian for momentum is calculated numerically
   * @param generalized_pos
   * @param linear_momentum
   * @param com_pos
   * @param slip_feet_velocities [2*n_slip_feet] the foot velocity
   * @return generalized velocities
   */
  drake::VectorX<double> LiftGeneralizedVelocity(
      const drake::VectorX<double>& generalized_pos,
      const drake::Vector3<double>& linear_momentum,
      const drake::Vector3<double>& com_pos,
      const drake::VectorX<double>& slip_feet_velocities) const;

  drake::VectorX<double> LiftContactPos(
      const drake::VectorX<double>& generalized_position) const;

  drake::VectorX<double> LiftContactVel(
      const drake::VectorX<double>& generalized_pos,
      const drake::VectorX<double>& generalized_vel) const;

  /*!
   * @brief Lifts the grf by evenly distributing the slip grf into the
   * equivalent toe and heel such that the toe and heel grf are equal and
   * parallel
   * @param com_pos
   * @param com_vel
   * @param slip_feet_pos
   * @param slip_force
   * @param complex_contact_point_pos
   * @return
   */
  drake::VectorX<double> LiftGrf(
      const drake::VectorX<double>& com_pos,
      const drake::VectorX<double>& com_vel,
      const drake::VectorX<double>& slip_feet_pos,
      const drake::VectorX<double>& slip_force,
      const drake::VectorX<double>& complex_contact_point_pos) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable drake::systems::Context<double>* context_;
  mutable drake::multibody::InverseKinematics ik_;
  const double m_;
  const double k_;
  const double b_;
  const double r0_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<double>>
      slip_contact_points_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<double>>
      complex_contact_points_;
  const std::map<int, std::vector<int>>
      simple_foot_index_to_complex_foot_index_;
  const int n_q_;
  const int n_v_;
  const std::vector<bool> slip_contact_mask_;
  drake::solvers::VectorXDecisionVariable com_vars_;

  const int kSLIP_DIM = 3;
};
