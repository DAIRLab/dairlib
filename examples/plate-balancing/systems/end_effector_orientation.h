#pragma once

#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {
namespace systems {

/**
 * @brief Generates orientation trajectories for the end effector.
 *
 * This system outputs a quaternion trajectory for the end effector,
 * switching between a neutral pose and a provided trajectory based on
 * radio input and a tracking flag.
 */
class EndEffectorOrientationTrajectoryGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor. Declares input/output ports.
   */
  EndEffectorOrientationTrajectoryGenerator();

  /**
   * @brief Returns the input port for the orientation trajectory.
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
   * @brief Sets whether to track the orientation trajectory.
   * @param track_orientation If true, tracks the provided trajectory;
   * otherwise, outputs neutral orientation.
   */
  void SetTrackOrientation(bool track_orientation) {
    track_orientation_ = track_orientation;
  }

 private:
  /**
   * @brief Calculates the output orientation trajectory based on radio input
   * and tracking flag.
   */
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  bool track_orientation_;  ///< Whether to track the provided orientation
                            ///< trajectory.

  drake::systems::InputPortIndex
      trajectory_port_;                        ///< Input port for trajectory.
  drake::systems::InputPortIndex radio_port_;  ///< Input port for radio signal.
};

}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib
