#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "multibody/kinematic/world_point_evaluator.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/gait.h"

/*!
 * @brief struct consisting of a collection of times and samples used for
 * passing inputs to reference generator
 * @tparam T
 */
template <typename T>
struct KnotPoints {
  std::vector<double> times;
  std::vector<T> samples;
};

class KcmpcReferenceGenerator {
 public:
  /*!
   * @brief constructor for reference generator
   * @param plant
   * @param nominal_stand nominal stand of size [nq]
   * @param contacts Vector of world point evalator describing contacts
   */
  KcmpcReferenceGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const std::vector<dairlib::multibody::WorldPointEvaluator<double>>&
          contacts);

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
   * @brief return gait sequence
   * @param gait_knot_points
   */
  static std::vector<double> GenerateTimePoints(
      const std::vector<double>& duration_scaling,
      std::vector<Gait> gait_knot_points);

  /*!
   * @brief construct references assuming initial com is the from the nominal
   * stand
   */
  void Generate();

  /*!
   * @brief Set the nominal reference configuration q_ref
   * @param com initial location of the com
   */
  void SetNominalReferenceConfiguration(const Eigen::VectorXd& q_ref) {
    q_ref_ = q_ref;
  }

  drake::trajectories::PiecewisePolynomial<double> com_trajectory_;
  drake::trajectories::PiecewisePolynomial<double> q_trajectory_;
  drake::trajectories::PiecewisePolynomial<double> v_trajectory_;
  drake::trajectories::PiecewisePolynomial<double> contact_sequence_;
  drake::trajectories::PiecewisePolynomial<double> grf_traj_;
  drake::trajectories::PiecewisePolynomial<double> contact_traj_;
  drake::trajectories::PiecewisePolynomial<double> momentum_trajectory_;

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<double>> contacts_;
  Eigen::VectorXd q_ref_;
  KnotPoints<Eigen::Vector3d> com_vel_knot_points_;
  KnotPoints<Gait> gait_knot_points_;
  double m_;
  Eigen::Vector3d p_ScmBase_W_;
};
