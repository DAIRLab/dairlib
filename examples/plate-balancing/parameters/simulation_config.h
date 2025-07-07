#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {

/**
 * @brief Simulation configuration for the plate balancing example.
 *
 * Contains model paths, simulation timing, initial conditions, and
 * visualization options.
 */
struct SimulationConfig {
  std::string franka_model;        ///< Path to the Franka robot model file.
  std::string end_effector_model;  ///< Path to the end effector model file.
  std::string tray_model;          ///< Path to the tray model file.
  std::string object_model;        ///< Path to the object model file.

  double dt;                       ///< Simulation timestep.
  double realtime_rate;            ///< Real-time rate for simulation.
  double actuator_delay;           ///< Delay for actuator commands.
  double franka_publish_rate;      ///< Publish rate for Franka state.
  double tray_publish_rate;        ///< Publish rate for tray state.
  double object_publish_rate;      ///< Publish rate for object state.
  double visualizer_publish_rate;  ///< Publish rate for visualizer.

  bool visualize_drake_sim;  ///< Whether to visualize the Drake simulation.
  bool publish_efforts;      ///< Whether to publish actuator efforts.
  bool publish_object_velocities;  ///< Whether to publish object velocities.

  Eigen::VectorXd q_init_franka;  ///< Initial joint positions for Franka.
  std::vector<Eigen::VectorXd> q_init_tray;  ///< Initial positions for tray.
  std::vector<Eigen::VectorXd>
      q_init_object;                      ///< Initial positions for object.
  Eigen::VectorXd tool_attachment_frame;  ///< Tool attachment frame.

  std::vector<Eigen::VectorXd> world_x_limits;  ///< X limits for the world.
  std::vector<Eigen::VectorXd> world_y_limits;  ///< Y limits for the world.
  std::vector<Eigen::VectorXd> world_z_limits;  ///< Z limits for the world.

  std::vector<double> external_force_scaling;  ///< Scaling for external forces.

  bool visualize_pose_trace;             ///< Visualize pose trace.
  bool visualize_center_of_mass_plan;    ///< Visualize center of mass plan.
  bool visualize_c3_forces;              ///< Visualize C3 forces.
  bool visualize_c3_object_state;        ///< Visualize C3 object state.
  bool visualize_c3_end_effector_state;  ///< Visualize C3 end effector state.
  bool visualize_workspace;              ///< Visualize workspace.

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(tray_model));
    a->Visit(DRAKE_NVP(object_model));

    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(franka_publish_rate));
    a->Visit(DRAKE_NVP(tray_publish_rate));
    a->Visit(DRAKE_NVP(object_publish_rate));
    a->Visit(DRAKE_NVP(visualizer_publish_rate));

    a->Visit(DRAKE_NVP(visualize_drake_sim));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(publish_object_velocities));

    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_tray));
    a->Visit(DRAKE_NVP(q_init_object));
    a->Visit(DRAKE_NVP(tool_attachment_frame));

    a->Visit(DRAKE_NVP(world_x_limits));
    a->Visit(DRAKE_NVP(world_y_limits));
    a->Visit(DRAKE_NVP(world_z_limits));
    a->Visit(DRAKE_NVP(external_force_scaling));

    a->Visit(DRAKE_NVP(visualize_pose_trace));
    a->Visit(DRAKE_NVP(visualize_center_of_mass_plan));
    a->Visit(DRAKE_NVP(visualize_c3_forces));
    a->Visit(DRAKE_NVP(visualize_c3_object_state));
    a->Visit(DRAKE_NVP(visualize_c3_end_effector_state));
    a->Visit(DRAKE_NVP(visualize_workspace));
  }
};

}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib