#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {
namespace systems {

/**
 * @brief Generates position trajectories for the end effector.
 *
 * This system outputs a position trajectory for the end effector,
 * switching between a neutral pose, a remote-controlled pose, or a provided
 * trajectory based on radio input.
 */
class EndEffectorTrajectoryGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor. Declares input/output ports and sets the neutral pose.
   * @param neutral_pose The default position for the end effector.
   */
  EndEffectorTrajectoryGenerator(const Eigen::Vector3d& neutral_pose);

  /**
   * @brief Returns the input port for the position trajectory.
   */
  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_port_);
  }

  /**
   * @brief Returns the input port for the radio signal.
   */
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  /**
   * @brief Sets the neutral pose and scaling for remote control.
   * @param neutral_pose The neutral pose for the end effector.
   * @param x_scale Scaling for x input.
   * @param y_scale Scaling for y input.
   * @param z_scale Scaling for z input.
   */
  void SetRemoteControlParameters(const Eigen::Vector3d& neutral_pose,
                                  double x_scale, double y_scale,
                                  double z_scale);

 private:
  /**
   * @brief Calculates the output trajectory based on radio input and trajectory
   * input.
   */
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  drake::systems::InputPortIndex
      trajectory_port_;                        ///< Input port for trajectory.
  drake::systems::InputPortIndex radio_port_;  ///< Input port for radio signal.

  Eigen::Vector3d neutral_pose_ = {0.55, 0, 0.40};  ///< Default neutral pose.
  double x_scale_;  ///< Scaling for x remote input.
  double y_scale_;  ///< Scaling for y remote input.
  double z_scale_;  ///< Scaling for z remote input.
};

}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib
