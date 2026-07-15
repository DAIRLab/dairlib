#pragma once

#include <string>
#include <vector>

#include <dairlib/lcmt_elastoplastic_network.hpp>
#include <dairlib/lcmt_saved_traj.hpp>
#include <drake/geometry/meshcat.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/discrete_values.h>

#include "lcm/lcm_trajectory.h"
#include "multibody/multipose_visualizer.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

// If the user does not provide a time vector, the default fallback is the epoch
// timestamp (seconds since 1970). However, using these large absolute times
// with Drake's cubic spline constructor will throw an exception. To avoid this,
// we generate a synthetic time vector with values spaced 1 second apart.
inline Eigen::VectorXd PopulateTimeVectorOfLcmTrajectoryIfUnspecified(
    const Eigen::VectorXd& time_vector) {
  Eigen::VectorXd new_time_vector = time_vector;
  if (time_vector[0] > 1e9) {
    for (int i = 0; i < time_vector.size(); ++i) {
      new_time_vector[i] = 1.0 * i;
    }
    return new_time_vector;
  }
  return time_vector;
};

/// Receives the output of an MPC planner as a lcmt_timestamped_saved_traj,
/// and draws it through meshcat.
class LcmTrajectoryDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmTrajectoryDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                               std::string trajectory_name,
                               const std::string& system_name = "");

  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_input_port_);
  }

  void SetLineColor(drake::geometry::Rgba rgba) { rgba_ = rgba; }
  void SetLineWidth(double line_width) { line_width_ = line_width; }

  void SetNumSamples(int N) {
    DRAKE_DEMAND(N > 1);
    N_ = N;
  }

 private:
  void OutputTrajectory(const drake::systems::Context<double>& context,
                        drake::trajectories::Trajectory<double>* traj) const;

  drake::systems::EventStatus DrawTrajectory(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex trajectory_input_port_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  const std::string system_name_;
  const std::string trajectory_name_;
  drake::geometry::Rgba rgba_ = drake::geometry::Rgba(0.1, 0.1, 0.1, 1.0);
  double line_width_ = 100;
  int N_ = 5;
};

/// Receives the output of an MPC planner as a lcmt_timestamped_saved_traj,
/// and draws the object pose through meshcat.
class LcmPoseDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmPoseDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                         const std::string& model_file,
                         const std::string& translation_trajectory_name,
                         const std::string& orientation_trajectory_name,
                         const std::string& system_name = "", int num_poses = 5,
                         bool add_transparency = true,
                         const Eigen::VectorXd& rgb = Eigen::VectorXd(),
                         const std::string& weld_frame_to_world = "");

  explicit LcmPoseDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                         const std::string& model_file,
                         const std::string& joint_trajectory_name,
                         const std::string& system_name = "", int num_poses = 5,
                         bool add_transparency = true,
                         const Eigen::VectorXd& rgb = Eigen::VectorXd(),
                         const std::string& weld_frame_to_world = "");

  explicit LcmPoseDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                         std::vector<std::string> model_files,
                         std::vector<std::string> translation_trajectory_names,
                         std::vector<std::string> orientation_trajectory_names,
                         const std::string& system_name = "", int num_poses = 5,
                         bool add_transparency = true,
                         const Eigen::VectorXd& rgb = Eigen::VectorXd(),
                         const std::string& weld_frame_to_world = "");

  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_input_port_);
  }

 private:
  void OutputTrajectory(const drake::systems::Context<double>& context,
                        drake::trajectories::Trajectory<double>* traj) const;

  drake::systems::EventStatus DrawTrajectory(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::EventStatus DrawTrajectoryFromJoints(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::EventStatus DrawTrajectoryObjects(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex trajectory_input_port_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  const std::string translation_trajectory_name_;
  const std::string orientation_trajectory_name_;
  const std::string joint_trajectory_name_;

  std::vector<std::string> translation_trajectory_names_;
  std::vector<std::string> orientation_trajectory_names_;
  std::vector<std::unique_ptr<multibody::MultiposeVisualizer>>
      multipose_visualizers_;
  const int N_;
};

/// Receives the output of an MPC planner as a lcmt_timestamped_saved_traj,
/// and draws it through meshcat.
class LcmForceDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmForceDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                          std::string force_trajectory_name,
                          std::string actor_trajectory_name,
                          std::string lcs_force_trajectory_name,
                          const std::string& system_name = "");

  const drake::systems::InputPort<double>& get_input_port_actor_trajectory()
      const {
    return this->get_input_port(actor_trajectory_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_robot_time() const {
    return this->get_input_port(robot_time_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_force_trajectory()
      const {
    return this->get_input_port(force_trajectory_input_port_);
  }

 private:
  drake::systems::EventStatus DrawForce(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
  drake::systems::EventStatus DrawForces(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex actor_trajectory_input_port_;
  drake::systems::InputPortIndex robot_time_input_port_;
  drake::systems::InputPortIndex force_trajectory_input_port_;

  drake::systems::DiscreteStateIndex actor_last_update_time_index_;
  drake::systems::DiscreteStateIndex forces_last_update_time_index_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  const drake::geometry::Cylinder cylinder_ =
      drake::geometry::Cylinder(0.002, 1.0);
  const drake::geometry::MeshcatCone arrowhead_ =
      drake::geometry::MeshcatCone(0.004, 0.004, 0.004);
  const std::string force_path_ = "c3_forces";
  const std::string actor_trajectory_name_;
  const std::string force_trajectory_name_;
  const std::string lcs_force_trajectory_name_;
  drake::geometry::Rgba actor_force_color_ =
      drake::geometry::Rgba(1, 0, 1, 1.0);
  drake::geometry::Rgba contact_force_color_ =
      drake::geometry::Rgba(0.949, 0.757, 0.0, 1.0);
  const double radius_ = 0.002;
  const double newtons_per_meter_ = 40;
};

/// Receives the target (and optionally actual and final target) C3 states via
/// LCM and draws them through meshcat.
class LcmC3TargetDrawer : public drake::systems::LeafSystem<double> {
 public:
  /// Constructor that assumes the robot is a 3 DoF end effector and there is
  /// one floating base (7 DoF) object.
  explicit LcmC3TargetDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                             bool draw_tray = true, bool draw_ee = false);
  /// Constructor that assumes the robot is a 3 DoF end effector and there are
  /// multiple floating base (7 DoF) objects.
  explicit LcmC3TargetDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                             int num_objects, bool draw_tray = true,
                             bool draw_ee = false);
  /// Constructor that loads the (1) robot and (1) object models from files,
  /// which determine the number of robot and object DoFs automatically.
  explicit LcmC3TargetDrawer(
      const std::shared_ptr<drake::geometry::Meshcat>&,
      const std::string& object_model_file, const std::string& robot_model_file,
      const std::string& weld_frame_to_world = "",
      const drake::math::RigidTransformd& object_world_offset =
          drake::math::RigidTransformd(),
      const drake::math::RigidTransformd& robot_world_offset =
          drake::math::RigidTransformd(),
      const Eigen::VectorXd& object_rgb = Eigen::VectorXd(),
      const Eigen::VectorXd& robot_rgb = Eigen::VectorXd(),
      const bool& include_actual = false,
      const bool& include_final_target = false);
  /// Constructor that assumes the robot is a 3 DoF end effector and there are
  /// multiple deformable network nodes (3 DoF each).
  explicit LcmC3TargetDrawer(
      const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
      const int& num_nodes, const std::string& node_model_file,
      const std::string& robot_model_file,
      const std::string& weld_frame_to_world = "",
      const drake::math::RigidTransformd& object_world_offset =
          drake::math::RigidTransformd(),
      const drake::math::RigidTransformd& robot_world_offset =
          drake::math::RigidTransformd(),
      const Eigen::VectorXd& actual_rgb = Eigen::VectorXd(),
      const Eigen::VectorXd& target_rgb = Eigen::VectorXd(),
      const Eigen::VectorXd& final_target_rgb = Eigen::VectorXd(),
      const bool& include_actual = false, const bool& include_target = false,
      const bool& include_final_target = false);

  const drake::systems::InputPort<double>&
  get_input_port_c3_state_final_target() const {
    return this->get_input_port(c3_state_final_target_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_c3_state_target()
      const {
    return this->get_input_port(c3_state_target_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_c3_state_actual()
      const {
    return this->get_input_port(c3_state_actual_input_port_);
  }

  const drake::systems::InputPort<double>&
  get_input_port_lcmt_elastoplastic_network() const {
    return this->get_input_port(lcmt_elastoplastic_network_input_port_);
  }

 private:
  drake::systems::EventStatus DrawC3State(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::EventStatus DrawC3StateMulti(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::EventStatus DrawC3StateGeneric(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::EventStatus DrawC3StateDeformableNetwork(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void DrawDeformableNetworkState(
      multibody::MultiposeVisualizer* multi_pose_visualizer,
      const dairlib::lcmt_elastoplastic_network* elastoplastic_model,
      const Eigen::VectorXd& node_locations, const std::string& meshcat_prefix,
      const drake::geometry::Rgba& color, const double& timestamp) const;

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;

  drake::systems::InputPortIndex c3_state_final_target_input_port_;
  drake::systems::InputPortIndex c3_state_target_input_port_;
  drake::systems::InputPortIndex c3_state_actual_input_port_;
  drake::systems::InputPortIndex lcmt_elastoplastic_network_input_port_;
  bool draw_tray_;
  bool draw_ee_;
  int num_objects_;
  int num_nodes_;

  drake::systems::DiscreteStateIndex last_update_time_index_;

  const drake::geometry::Cylinder cylinder_for_tray_ =
      drake::geometry::Cylinder(0.005, 0.1);
  const drake::geometry::Cylinder cylinder_for_ee_ =
      drake::geometry::Cylinder(0.0025, 0.05);
  const drake::geometry::Cylinder cylinder_for_deformable_ =
      drake::geometry::Cylinder(0.002, 1.0);
  const std::string c3_state_path_ = "c3_state";
  const std::string c3_final_target_object_path_ =
      "c3_state/c3_final_target_object";
  const std::string c3_target_object_path_ = "c3_state/c3_target_object";
  const std::string c3_actual_object_path_ = "c3_state/c3_actual_object";
  const std::string c3_target_ee_path_ = "c3_state/c3_target_ee";
  const std::string c3_actual_ee_path_ = "c3_state/c3_actual_ee";

  drake::geometry::Rgba actual_color_;
  drake::geometry::Rgba target_color_;
  drake::geometry::Rgba final_target_color_;

  std::vector<std::string> c3_state_paths_;
  std::vector<std::string> c3_final_target_object_paths_;
  std::vector<std::string> c3_target_object_paths_;
  std::vector<std::string> c3_actual_object_paths_;
  std::vector<std::string> c3_target_ee_paths_;
  std::vector<std::string> c3_actual_ee_paths_;

  // Optionally used only if using constructor with model files:
  std::unique_ptr<multibody::MultiposeVisualizer>
      object_pose_actual_visualizer_ = nullptr;
  std::unique_ptr<multibody::MultiposeVisualizer>
      robot_pose_actual_visualizer_ = nullptr;
  std::unique_ptr<multibody::MultiposeVisualizer>
      object_pose_target_visualizer_ = nullptr;
  std::unique_ptr<multibody::MultiposeVisualizer>
      robot_pose_target_visualizer_ = nullptr;
  std::unique_ptr<multibody::MultiposeVisualizer>
      object_pose_final_visualizer_ = nullptr;
  std::unique_ptr<multibody::MultiposeVisualizer> robot_pose_final_visualizer_ =
      nullptr;
};

/// Receives a C3 plan via LCM and draws the planned trajectory through meshcat.
class LcmC3PlanDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmC3PlanDrawer(
      const std::shared_ptr<drake::geometry::Meshcat>& meshcat, const int& N,
      const std::string& object_model_file, const std::string& robot_model_file,
      const std::string& weld_frame_to_world = "",
      const drake::math::RigidTransformd& object_world_offset =
          drake::math::RigidTransformd(),
      const drake::math::RigidTransformd& robot_world_offset =
          drake::math::RigidTransformd(),
      const Eigen::VectorXd& object_rgb = Eigen::VectorXd(),
      const Eigen::VectorXd& robot_rgb = Eigen::VectorXd(),
      const bool& show_object = true, const bool& show_robot = true);

  /// Constructor that assumes the robot is a 3 DoF end effector and there are
  /// multiple deformable network nodes (3 DoF each), drawn across the full C3
  /// plan horizon. Consumes the raw `c3::lcmt_output` message (the LCM type
  /// published by C3OutputGenerator), not `dairlib::lcmt_c3_output`.
  explicit LcmC3PlanDrawer(
      const std::shared_ptr<drake::geometry::Meshcat>& meshcat, const int& N,
      const int& num_nodes, const std::string& node_model_file,
      const std::string& robot_model_file,
      const std::string& weld_frame_to_world = "",
      const drake::math::RigidTransformd& object_world_offset =
          drake::math::RigidTransformd(),
      const drake::math::RigidTransformd& robot_world_offset =
          drake::math::RigidTransformd(),
      const Eigen::VectorXd& object_rgb = Eigen::VectorXd(),
      const Eigen::VectorXd& robot_rgb = Eigen::VectorXd(),
      const bool& show_object = true, const bool& show_robot = true);

  const drake::systems::InputPort<double>& get_input_port_c3_plan() const {
    return this->get_input_port(c3_plan_input_port_);
  }

  const drake::systems::InputPort<double>&
  get_input_port_lcmt_elastoplastic_network() const {
    return this->get_input_port(lcmt_elastoplastic_network_input_port_);
  }

 private:
  drake::systems::EventStatus DrawC3Plan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::EventStatus DrawC3PlanDeformableNetwork(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  const int N_;
  int num_nodes_ = 0;
  drake::systems::InputPortIndex c3_plan_input_port_;
  drake::systems::InputPortIndex lcmt_elastoplastic_network_input_port_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  const drake::geometry::Cylinder cylinder_for_deformable_ =
      drake::geometry::Cylinder(0.002, 1.0);
  std::unique_ptr<multibody::MultiposeVisualizer> object_plan_visualizer_ =
      nullptr;
  std::unique_ptr<multibody::MultiposeVisualizer> robot_plan_visualizer_ =
      nullptr;
  // One visualizer per horizon step (deformable-network overload only), each
  // rooted at its own meshcat path so nodes + connections for a given step
  // can be toggled together as a single group.
  std::vector<std::unique_ptr<multibody::MultiposeVisualizer>>
      object_plan_step_visualizers_;
  std::vector<drake::geometry::Rgba> object_plan_step_colors_;
};

}  // namespace systems
}  // namespace dairlib
