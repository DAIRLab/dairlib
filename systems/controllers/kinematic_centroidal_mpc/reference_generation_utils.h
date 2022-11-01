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
 * @param base_rt_com_ewrt_w vector from com to base in world frame
 * @param com_traj trajectory for the center of mass
 * @param base_pos_start index where the base position starts in generalized state
 * @return trajectory of generalized state
 */
drake::trajectories::PiecewisePolynomial<double> GenerateGeneralizedPosTrajectory(const Eigen::VectorXd& nominal_stand,
                                                                                  const Eigen::Vector3d& base_rt_com_ewrt_w,
                                                                                  const drake::trajectories::PiecewisePolynomial<double>& com_traj,
                                                                                  int base_pos_start);

/*!
 * @brief constructs a trajectory for the generalized velocity where the joint velocity is 0 and floating base val from com trajectory
 * @param com_traj
 * @param n_v number of velocity states
 * @param base_vel_start index where base vel starts
 * @return
 */
drake::trajectories::PiecewisePolynomial<double> GenerateGeneralizedVelTrajectory(const drake::trajectories::PiecewisePolynomial<double>& com_traj,
                                                                                  int n_v,
                                                                                  int base_vel_start);

/*!
 * @brief given a vector of gaits and time points corresponding to when the gaits are active
 * @param gait_sequence vector of gaits
 * @param time_points vector of time points when each gait is active
 * @return trajectory of mode status start at time_point(0) and ending at time_point.end()-1
 */
drake::trajectories::PiecewisePolynomial<double> GenerateModeSequence(const std::vector<Gait>& gait_sequence,
                                                                      const std::vector<double>& time_points);