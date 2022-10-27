#pragma once

#include <drake/multibody/plant/multibody_plant.h>
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

template <typename T>
struct KnotPoints{
  std::vector<double> times;
  std::vector<T> samples;
};

class KcmpcReferenceGenerator{
 public:
  KcmpcReferenceGenerator(const drake::multibody::MultibodyPlant<double>& plant,
                          const Eigen::VectorXd& nominal_stand,
                          const std::vector<dairlib::multibody::WorldPointEvaluator<double>>& contacts);


  void SetComKnotPoints(const KnotPoints<Eigen::Vector3d>& com_knot_points);

  void SetGaitSequence(const KnotPoints<Gait>& gait_knot_points);

  void Build();

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
  bool built_ = false;
  double m_;
  Eigen::Vector3d base_rt_com_ewrt_w;
  std::unique_ptr<drake::systems::Context<double>> context_;

};