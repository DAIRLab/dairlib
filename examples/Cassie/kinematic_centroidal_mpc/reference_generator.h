
#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <Eigen/Core>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include "multibody/kinematic/world_point_evaluator.h"

struct Mode{
  double start_phase;
  double end_phase;
  drake::VectorX<bool> contact_status;
};

struct Gait{
  double period;
  std::vector<Mode> gait_pattern;

  drake::trajectories::PiecewisePolynomial<double> ToTrajectory(double current_time, double end_time) const;
};

Eigen::VectorXd GenerateNominalStand(const drake::multibody::MultibodyPlant<double> &plant,
                                     double pelvis_height,
                                     double stance_width,
                                     bool visualize = false);

/*!
 * @brief Constructs a com trajectory given world frame velocity
 * @param current_com current xyz location of com
 * @param vel_ewrt_w vector of velocity commands
 * @param time_points vector of times for velocity commands, start time is first entry, final time of spline is last entry
 * @return spline describing com position
 */
drake::trajectories::PiecewisePolynomial<double> GenerateComTrajectory(const Eigen::Vector3d& current_com,
                                                              const std::vector<Eigen::Vector3d>& vel_ewrt_w,
                                                              const std::vector<double>& time_points);


/*!
 * @brief Creates a trajectory in generalized position given a nominal state and com trajectory
 * @param nominal_stand nominal generalized state of the robot
 * @param base_rt_com_ewrt_w offset from base to com
 * @param com_traj trajectory of center of mass
 * @param base_pos_start index in generalized state where base position starts
 * @return center of mass trajectory
 */
drake::trajectories::PiecewisePolynomial<double> GenerateGeneralizedPosTrajectory(const Eigen::VectorXd& nominal_stand,
                                                                                  const Eigen::Vector3d& base_rt_com_ewrt_w,
                                                                                  const drake::trajectories::PiecewisePolynomial<double>& com_traj,
                                                                                  int base_pos_start);

drake::trajectories::PiecewisePolynomial<double> GenerateGeneralizedVelTrajectory(const drake::trajectories::PiecewisePolynomial<double>& com_traj,
                                                                                  int n_v,
                                                                                  int base_vel_start);

int FindCurrentMode(const Gait& active_gait, double time_now);

drake::trajectories::PiecewisePolynomial<double> GenerateModeSequence(const std::vector<Gait>& gait_sequence,
                                                                      const std::vector<double>& time_points);

/*!
 * @brief Calculates the next time a index of the zoh trajectory goes to a specific value
 * @param trajectory piecewise polynomial trajectory that is a zoh
 * @param current_time the time to start the search
 * @param index index of the trajectory output to track
 * @param value value searching for
 * @return current time if trajectory(current_time)[index] = value,
 *         the first time greater than current time and the value matches,
 *         If the trajectory never goes to value, returns trajectory.end_time()
 */
double TimeNextValue(const drake::trajectories::PiecewisePolynomial<double>& trajectory,
                     double current_time,
                     int index,
                     double value);


drake::trajectories::PiecewisePolynomial<double> GenerateGrfReference(const drake::trajectories::PiecewisePolynomial<double>& mode_trajectory,
                                                                      double m);

drake::trajectories::PiecewisePolynomial<double> GenerateContactPointReference(const drake::multibody::MultibodyPlant<double> &plant,
                                                                               const std::vector<dairlib::multibody::WorldPointEvaluator<
                                                                                   double>> &contacts,
                                                                               const drake::trajectories::PiecewisePolynomial<
                                                                                   double> &q_traj,
                                                                               const drake::trajectories::PiecewisePolynomial<
                                                                                   double> &v_traj);