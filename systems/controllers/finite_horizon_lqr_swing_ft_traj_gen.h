#pragma once
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"

#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers {

/// FiniteHorizonLqrSwingFootTrajGenerator generates a trajectory for the
/// swing foot to determine the suitable swing foot acceleration in the xy
/// directions. A trajectory is then calculated such that y_error = 0,
/// ydot_error = 0, and y_ddot = -K(t)(y - yd) where yd is the xy coordinates
/// of the next stance foot location. A cubic spline is used for the swing foot
/// z coordinate

typedef struct SwingFootTajGenOptions {
  std::string floating_base_body_name = "";
  double double_support_duration;
  double mid_foot_height;
  double desired_final_foot_height ;
  double desired_final_vertical_foot_velocity;
  double max_com_to_x_footstep_dist;
  double footstep_offset;
  double center_line_offset;
  bool wrt_com_in_local_frame = false;
} SwingFootTajGenOptions;

class FiniteHorizonLqrSwingFootTrajGenerator :
     public drake::systems::LeafSystem<double> {
 public:
  FiniteHorizonLqrSwingFootTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::systems::LinearSystem<double>& double_integrator,
      const std::vector<int> left_right_support_fsm_states,
      const std::vector<double> left_right_support_durations,
      const std::vector<std::pair<const Eigen::Vector3d,
                            const drake::multibody::Frame<double>&>> pts,
      const SwingFootTajGenOptions opts);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
  const {
    return this->get_input_port(liftoff_time_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_foot_target() const {
    return this->get_input_port(foot_target_port_);
  }

  static drake::systems::LinearSystem<double> MakeDoubleIntegratorSystem();

 private:

  Eigen::Vector4d CalcSwingFootState(
      const Eigen::VectorXd& x,
      const std::pair<
      const Eigen::Vector3d, const drake::multibody::Frame<double>&> pt) const;

  drake::trajectories::PiecewisePolynomial<double> CreateSplineForSwingFoot(
      const double start_time_of_this_interval,
      const double end_time_of_this_interval,
      const double timestamp,
      const Eigen::Vector4d& init_swing_foot_xy_state,
      const Eigen::Vector2d& u,
      double stance_foot_height) const;

  double CalcStanceFootHeight(const Eigen::VectorXd& x,
      const std::pair<
      const Eigen::Vector3d, const drake::multibody::Frame<double>&> pt) const;

  void CalcTrajs(const drake::systems::Context<double>& context,
                 drake::trajectories::Trajectory<double>* traj) const;

  // MBP parameters
  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  const drake::systems::LinearSystem<double>& double_integrator_;
  mutable std::unique_ptr<drake::systems::Context<double>> double_integrator_context_;

  // Footstep parameters
  const SwingFootTajGenOptions opts_;
  const std::vector<int> left_right_support_fsm_states_;
  const std::vector<double> left_right_support_durations_;
  const std::vector<std::pair<const Eigen::Vector3d,
                              const drake::multibody::Frame<double>&>> pts_;

  // maps
  std::map<int, std::pair<const Eigen::Vector3d,
                          const drake::multibody::Frame<double>&>>
      swing_foot_map_;
  std::map<int, std::pair<const Eigen::Vector3d,
                          const drake::multibody::Frame<double>&>>
      stance_foot_map_;
  std::map<int, double> duration_map_;

  // ports
  int state_port_;
  int fsm_port_;
  int liftoff_time_port_;
  int foot_target_port_;

  // LQR parameters
  Eigen::Matrix<double, 4, 4> Q_ =  Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 4> Qf_ = 5 * Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 2, 2> R_ = 0.1 * Eigen::Matrix2d::Identity();
};
}