#pragma once

#include <string>
#include <vector>

#include <drake/geometry/meshcat.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/discrete_values.h>

#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "multibody/multipose_visualizer.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {
namespace systems {
namespace visualization {

/**
 * @class PositionTrajectoryDrawer
 * @brief Receives the output of an MPC planner as a drake::Trajectory and draws
 * it through Meshcat.
 *
 * This system subscribes to a trajectory input port and visualizes the
 * trajectory as a line in Meshcat. The color and number of samples for the line
 * can be configured.
 *
 * @param meshcat Shared pointer to the Meshcat visualizer.
 * @param trajectory_name Name for the trajectory in the Meshcat scene tree.
 */
class PositionTrajectoryDrawer : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor.
   * @param meshcat Shared pointer to the Meshcat visualizer.
   * @param trajectory_name Name for the trajectory in the Meshcat scene tree.
   */
  explicit PositionTrajectoryDrawer(
      const std::shared_ptr<drake::geometry::Meshcat>&,
      std::string trajectory_name);

  /**
   * @brief Returns the input port for the trajectory.
   */
  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_input_port_);
  }

  /**
   * @brief Sets the color of the trajectory line.
   * @param rgba The color as a Drake Rgba object.
   */
  void SetLineColor(drake::geometry::Rgba rgba) { rgba_ = rgba; }

  /**
   * @brief Sets the number of samples to draw along the trajectory.
   * @param N Number of samples (must be > 1).
   */
  void SetNumSamples(int N) {
    DRAKE_DEMAND(N > 1);
    N_ = N;
  }

 private:
  /**
   * @brief Outputs the trajectory for visualization.
   * @param context The system context.
   * @param traj The trajectory to output.
   */
  void OutputTrajectory(const drake::systems::Context<double>& context,
                        drake::trajectories::Trajectory<double>* traj) const;

  /**
   * @brief Draws the trajectory in Meshcat.
   * @param context The system context.
   * @param discrete_state The system's discrete state.
   */
  drake::systems::EventStatus DrawTrajectory(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex trajectory_input_port_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  const std::string trajectory_name_;
  drake::geometry::Rgba rgba_ = drake::geometry::Rgba(0.1, 0.1, 0.1, 1.0);
  int N_ = 5;
};

/**
 * @class PoseTrajectoryDrawer
 * @brief Receives the output of an MPC planner as a drake::Trajectory and draws
 * the object pose through Meshcat.
 *
 * This system visualizes a sequence of object poses (with optional orientation)
 * in Meshcat, using a provided model file for the object.
 *
 * @param meshcat Shared pointer to the Meshcat visualizer.
 * @param model_file Path to the model file for visualization.
 * @param trajectory_name Name for the trajectory in the Meshcat scene tree.
 * @param include_orientation Whether to include orientation in the
 * visualization.
 * @param num_poses Number of poses to visualize.
 * @param add_transparency Whether to add transparency to the visualized
 * objects.
 */
class PoseTrajectoryDrawer : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor.
   * @param meshcat Shared pointer to the Meshcat visualizer.
   * @param model_file Path to the model file for visualization.
   * @param trajectory_name Name for the trajectory in the Meshcat scene tree.
   * @param include_orientation Whether to include orientation in the
   * visualization.
   * @param num_poses Number of poses to visualize.
   * @param add_transparency Whether to add transparency to the visualized
   * objects.
   */
  explicit PoseTrajectoryDrawer(
      const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
      const std::string& model_file, const std::string& trajectory_name,
      bool include_orientation, int num_poses = 5,
      bool add_transparency = true);

  /**
   * @brief Returns the input port for the translation trajectory.
   */
  const drake::systems::InputPort<double>&
  get_input_port_translation_trajectory() const {
    return this->get_input_port(translation_trajectory_input_port_);
  }

  /**
   * @brief Returns the input port for the orientation trajectory.
   */
  const drake::systems::InputPort<double>&
  get_input_port_orientation_trajectory() const {
    return this->get_input_port(orientation_trajectory_input_port_);
  }

 private:
  /**
   * @brief Outputs the trajectory for visualization.
   * @param context The system context.
   * @param traj The trajectory to output.
   */
  void OutputTrajectory(const drake::systems::Context<double>& context,
                        drake::trajectories::Trajectory<double>* traj) const;

  /**
   * @brief Draws the trajectory in Meshcat.
   * @param context The system context.
   * @param discrete_state The system's discrete state.
   */
  drake::systems::EventStatus DrawTrajectory(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex translation_trajectory_input_port_;
  drake::systems::InputPortIndex orientation_trajectory_input_port_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  const std::string trajectory_name_;
  bool include_orientation_;
  std::unique_ptr<multibody::MultiposeVisualizer> multipose_visualizer_;
  const int N_;
};

/**
 * @class LcmForceDrawer
 * @brief Receives the output of an MPC planner as a drake::Trajectory and draws
 * forces through Meshcat.
 *
 * This system visualizes force vectors (e.g., contact or actuator forces) as
 * arrows in Meshcat, using trajectory and force input ports.
 *
 * @param meshcat Shared pointer to the Meshcat visualizer.
 * @param trajectory_name Name for the trajectory in the Meshcat scene tree.
 */
class LcmForceDrawer : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor.
   * @param meshcat Shared pointer to the Meshcat visualizer.
   * @param trajectory_name Name for the trajectory in the Meshcat scene tree.
   */
  explicit LcmForceDrawer(
      const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
      std::string trajectory_name);

  /**
   * @brief Returns the input port for the position trajectory.
   */
  const drake::systems::InputPort<double>& get_input_port_position_trajectory()
      const {
    return this->get_input_port(position_trajectory_input_port_);
  }

  /**
   * @brief Returns the input port for the input trajectory.
   */
  const drake::systems::InputPort<double>& get_input_port_input_trajectory()
      const {
    return this->get_input_port(input_trajectory_input_port_);
  }

  /**
   * @brief Returns the input port for the robot time.
   */
  const drake::systems::InputPort<double>& get_input_port_robot_time() const {
    return this->get_input_port(robot_time_input_port_);
  }

  /**
   * @brief Returns the input port for the force trajectory.
   */
  const drake::systems::InputPort<double>& get_input_port_force_trajectory()
      const {
    return this->get_input_port(force_trajectory_input_port_);
  }

 private:
  /**
   * @brief Draws a single force in Meshcat.
   * @param context The system context.
   * @param discrete_state The system's discrete state.
   */
  drake::systems::EventStatus DrawForce(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  /**
   * @brief Draws multiple forces in Meshcat.
   * @param context The system context.
   * @param discrete_state The system's discrete state.
   */
  drake::systems::EventStatus DrawForces(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
  drake::systems::InputPortIndex position_trajectory_input_port_;
  drake::systems::InputPortIndex input_trajectory_input_port_;
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
  const std::string trajectory_name_;
  drake::geometry::Rgba actor_force_color_ =
      drake::geometry::Rgba(1, 0, 1, 1.0);
  drake::geometry::Rgba contact_force_color_ =
      drake::geometry::Rgba(0.949, 0.757, 0.0, 1.0);
  const double radius_ = 0.002;
  const double newtons_per_meter_ = 40;
};

/**
 * @class LcmC3TargetDrawer
 * @brief Receives the output of an MPC planner as a drake::Trajectory and draws
 * C3 target and actual states through Meshcat.
 *
 * This system visualizes the target and actual states of a C3 object (e.g.,
 * tray or end-effector) as cylinders in Meshcat, for debugging and
 * demonstration.
 *
 * @param meshcat Shared pointer to the Meshcat visualizer.
 * @param draw_tray Whether to draw the tray.
 * @param draw_ee Whether to draw the end-effector.
 */
class LcmC3TargetDrawer : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor.
   * @param meshcat Shared pointer to the Meshcat visualizer.
   * @param draw_tray Whether to draw the tray.
   * @param draw_ee Whether to draw the end-effector.
   */
  explicit LcmC3TargetDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                             bool draw_tray = true, bool draw_ee = false);

  /**
   * @brief Returns the input port for the C3 state target.
   */
  const drake::systems::InputPort<double>& get_input_port_c3_state_target()
      const {
    return this->get_input_port(c3_state_target_input_port_);
  }

  /**
   * @brief Returns the input port for the C3 state actual.
   */
  const drake::systems::InputPort<double>& get_input_port_c3_state_actual()
      const {
    return this->get_input_port(c3_state_actual_input_port_);
  }

 private:
  /**
   * @brief Draws the C3 state in Meshcat.
   * @param context The system context.
   * @param discrete_state The system's discrete state.
   */
  drake::systems::EventStatus DrawC3State(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;

  drake::systems::InputPortIndex c3_state_target_input_port_;
  drake::systems::InputPortIndex c3_state_actual_input_port_;

  bool draw_tray_;
  bool draw_ee_;

  drake::systems::DiscreteStateIndex last_update_time_index_;

  const drake::geometry::Cylinder cylinder_for_tray_ =
      drake::geometry::Cylinder(0.005, 0.1);
  const drake::geometry::Cylinder cylinder_for_ee_ =
      drake::geometry::Cylinder(0.0025, 0.05);
  const std::string c3_state_path_ = "c3_state";
  const std::string c3_target_object_path_ = "c3_state/c3_target_object";
  const std::string c3_actual_object_path_ = "c3_state/c3_actual_object";
  const std::string c3_target_ee_path_ = "c3_state/c3_target_ee";
  const std::string c3_actual_ee_path_ = "c3_state/c3_actual_ee";
};

}  // namespace visualization
}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib