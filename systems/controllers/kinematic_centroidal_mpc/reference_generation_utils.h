#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include "multibody/kinematic/world_point_evaluator.h"
#include "systems/controllers/kinematic_centroidal_mpc/gait.h"

/*!
 * @brief given an initial com and velocity of com emrt w, calculate a com trajectory
 * @param current_com
 * @param vel_ewrt_w velocity of the com ewrt. Given a zoh between time points, must be the same length as time_points
 * @param time_points time values correspeding to velocities, trajectory start at the first, and end at the last, must be the same length as vel_ewrt_w
 * @return trajectory of com velocity
 */
drake::trajectories::PiecewisePolynomial<double> GenerateComTrajectory(const Eigen::Vector3d& current_com,
                                                                       const std::vector<Eigen::Vector3d>& vel_ewrt_w,
                                                                       const std::vector<double>& time_points);

/*!
 * @brief Constructs a trajectory for the generalized positions of a constant joint state and floating base position from com trajectory
 * @param nominal_stand [nq] generalized state
 * @param p_ScmBase_W vector from com to floating base in world frame
 * @param com_traj trajectory for the center of mass
 * @param base_pos_start index where the floating base position starts in generalized state
 * @return trajectory of generalized position
 */
drake::trajectories::PiecewisePolynomial<double> GenerateGeneralizedPosTrajectory(const Eigen::VectorXd& nominal_stand,
                                                                                  const Eigen::Vector3d& p_ScmBase_W,
                                                                                  const drake::trajectories::PiecewisePolynomial<double>& com_traj,
                                                                                  int base_pos_start);

/*!
 * @brief constructs a trajectory for the generalized velocity where the joint velocity is 0 and floating base val from com trajectory
 * @param com_traj
 * @param n_v number of velocity states
 * @param base_vel_start index where base vel starts
 * @return trajectory of generalized velocity
 */
drake::trajectories::PiecewisePolynomial<double> GenerateGeneralizedVelTrajectory(const drake::trajectories::PiecewisePolynomial<double>& com_traj,
                                                                                  int n_v,
                                                                                  int base_vel_start);

/*!
 * @brief given a vector of gaits and time points corresponding to when the gaits are active generate a mode sequence.
 *      The transition between gaits can be awkward depending on where the gaits are in phase space during the transition
 *      {TODO SRL} make transition clean by attempting to shift phase so mode sequence lines up
 * @param gait_sequence vector of gaits
 * @param time_points vector of time points when each gait is active
 * @return trajectory of mode status start at time_point(0) and ending at time_point.end()-1
 */
drake::trajectories::PiecewisePolynomial<double> GenerateModeSequence(const std::vector<Gait>& gait_sequence,
                                                                      const std::vector<double>& time_points);

/*!
 * @brief given a trajectory which describes the mode, and the mass of the robot calculate a nominal grf trajectory where the weight of the robot
 *          is distributed over the active contact points
 * @param mode_trajectory
 * @param m
 * @return trajectory of grf for reference
 */
drake::trajectories::PiecewisePolynomial<double> GenerateGrfReference(const drake::trajectories::PiecewisePolynomial<double>& mode_trajectory,
                                                                      double m);

/*!
 * @brief Calculate trajectory of world point evaluators from generalized state trajectory. This assumes first order hold
 * between knot points in state trajectories.
 * TODO If we start using more complicated state references with this function sample time more coarsely
 * @param plant
 * @param contacts vector of world point evaluators
 * @param q_traj generalized position trajectory
 * @param v_traj generalized velocity trajectory
 * @return trajectory of contact points stacked [contact1_pos, contact1_vel, ... contact_n_pos, contact_n_vel]
 */
drake::trajectories::PiecewisePolynomial<double> GenerateContactPointReference(const drake::multibody::MultibodyPlant<double> &plant,
                                                                               const std::vector<dairlib::multibody::WorldPointEvaluator<
                                                                                   double>> &contacts,
                                                                               const drake::trajectories::PiecewisePolynomial<
                                                                                   double> &q_traj,
                                                                               const drake::trajectories::PiecewisePolynomial<
                                                                                   double> &v_traj);