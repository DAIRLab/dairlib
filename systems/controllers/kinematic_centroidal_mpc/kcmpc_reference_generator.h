#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include "multibody/kinematic/world_point_evaluator.h"

/*!
 * @brief Struct for a given mode as part of a mode sequence
 */
struct Mode{
  double start_phase;
  double end_phase;
  drake::VectorX<bool> contact_status; /// Vector describing which contacts are active
};

/*!
 * @brief Struct for defining a gait consisting of a vector of Modes, and a period for the whole gait.
 */
struct Gait{
  double period;
  std::vector<Mode> gait_pattern; /// Vector of modes. Start phase of the first mode must be 0, end phase of the last mode
                                  /// must be 1, and no time gaps between start and end of sequential modes

  /*!
   * @brief converts the gait into a trajectory of of the contact timing
   * @param current_time time for the trajectory to start
   * @param end_time time for the trajectory to end
   * @return the trajectory
   */
  drake::trajectories::PiecewisePolynomial<double> ToTrajectory(double current_time, double end_time) const;

  /*!
   * @brief Checks to make sure the gait is valid, Asserts if not valid
   */
  void Is_Valid() const;
};

/*!
 * @brief struct consisting of a collection of times and samples used for passing inputs to reference generator
 * @tparam T
 */
template <typename T>
struct KnotPoints{
  std::vector<double> times;
  std::vector<T> samples;
};

class KcmpcReferenceGenerator{
 public:

  /*!
   * @brief constructor for reference generator
   * @param plant
   * @param nominal_stand nominal stand of size [nq]
   * @param contacts Vector of world point evalator describing contacts
   */
  KcmpcReferenceGenerator(const drake::multibody::MultibodyPlant<double>& plant,
                          const Eigen::VectorXd& nominal_stand,
                          const std::vector<dairlib::multibody::WorldPointEvaluator<double>>& contacts);

  /*!
   * @brief specify com velocity at specific times
   * @param com_knot_points
   */
  void SetComKnotPoints(const KnotPoints<Eigen::Vector3d>& com_knot_points);

  /*!
   * @brief specify gait at specific times
   * @param gait_knot_points
   */
  void SetGaitSequence(const KnotPoints<Gait>& gait_knot_points);

  /*!
   * @brief construct references assuming initial com is the from the nominal stand
   */
  void Build();

  /*!
   * @brief Constructs references based on the time of the knot points
   * @param com initial location of the com
   */
  void Build(const Eigen::Vector3d& com);

  drake::trajectories::PiecewisePolynomial<double> com_trajectory_;
  drake::trajectories::PiecewisePolynomial<double> q_trajectory_;
  drake::trajectories::PiecewisePolynomial<double> v_trajectory_;
  drake::trajectories::PiecewisePolynomial<double> contact_sequence_;
  drake::trajectories::PiecewisePolynomial<double> grf_traj_;
  drake::trajectories::PiecewisePolynomial<double> contact_traj_;

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<double>> contacts_;
  Eigen::VectorXd nominal_stand_;
  KnotPoints<Eigen::Vector3d> com_vel_knot_points_;
  KnotPoints<Gait> gait_knot_points_;
  double m_;
  Eigen::Vector3d base_rt_com_ewrt_w;
  std::unique_ptr<drake::systems::Context<double>> context_;

};