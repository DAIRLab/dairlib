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

template <typename T>
class PlanarSlipReductionConstraint : public dairlib::solvers::NonlinearConstraint<T> {

 public:
  PlanarSlipReductionConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                                drake::systems::Context<T>* context,
                                const std::vector<dairlib::multibody::WorldPointEvaluator<T>>& slip_feet,
                                int complex_state_size,
                                T m, int knot_index);

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

  const T m_;
  drake::systems::Context<T>* context_;
  const drake::multibody::MultibodyPlant<T>& plant_;
  int nx_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<T>>& slip_feet_;
  const int kSLIP_DIM = 2;
};

template <typename T>
class PlanarSlipLifter {
 public:
  PlanarSlipLifter(const drake::multibody::MultibodyPlant<T>& plant,
                   drake::systems::Context<T>* context,
                   const std::vector<dairlib::multibody::WorldPointEvaluator<T>>& slip_contact_points,
                   const std::vector<dairlib::multibody::WorldPointEvaluator<T>>& complex_contact_points,
                   const std::map<int, std::vector<int>>& simple_foot_index_to_complex_foot_index,
                   const drake::VectorX<T>& nominal_stand,
                   T m,
                   double k,
                   double r0,
                   const drake::VectorX<T>& stance_widths);

 private:
  /*!
   * @brief given the center of mass position and slip feet position solve for the generalized position of the full robot
   * @note assumes identity orientation
   * @note implements numerical ik (can be slow)
   * @param com_position center of mass position in the world
   * @param slip_feet_positions [2*n_slip_feet] locations of the planar slip feet
   * @return the generalized positions
   */
  drake::VectorX<T> LiftGeneralizedPosition(const drake::Vector3<T>& com_position, const drake::VectorX<T>& slip_feet_positions);

  /*!
   * @brief Given a generalized position calculate the generalized velocity that is the least squares solution to tracking the
   * centroidal momentum and foot velocity.
   * @note Assumes 0 angular momentum
   * @note Jacobian for momentum is calculated numerically
   * @param generalized_pos
   * @param linear_momentum
   * @param com_pos
   * @param slip_feet_velocities [2*n_slip_feet] the foot velocity
   * @return generalized velocities
   */
  drake::VectorX<T> LiftGeneralizedVelocity(const drake::VectorX<T>& generalized_pos,
                                            const drake::Vector3<T>& linear_momentum,
                                            const drake::Vector3<T>& com_pos,
                                            const drake::VectorX<T>& slip_feet_velocities);

  drake::VectorX<T> LiftContactPos(const drake::VectorX<T>& generalized_position);

  drake::VectorX<T> LiftContactVel(const drake::VectorX<T>& generalized_state);

  drake::VectorX<T> LiftGrf(const drake::VectorX<T>& com_pos,
                            const drake::VectorX<T>& slip_feet_pos,
                            const drake::VectorX<T>& complex_contact_point_pos);


  const drake::multibody::MultibodyPlant<T>& plant_;
  drake::systems::Context<T>* context_;
  drake::multibody::InverseKinematics ik_;
  const T m_;
  const double k_;
  const double r0_;
  const drake::VectorX<T>& stance_widths_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<T>>& slip_contact_points_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<T>>& complex_contact_points_;
  const std::map<int, std::vector<int>>& simple_foot_index_to_complex_foot_index_;
  const int n_q_;
  const int n_v_;

  drake::solvers::VectorXDecisionVariable com_vars_;
};
