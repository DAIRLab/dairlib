#pragma once

#include <map>
#include <string>
#include <vector>

#include "dairlib/lcmt_robot_output.hpp"
#include "systems/framework/output_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {
namespace lcmt_systems {

/**
 * @class RobotOutputConsumer
 * @brief Receives the output of an LcmSubscriberSystem that subscribes to the
 * Robot output channel with LCM type lcmt_robot_output, and outputs the
 * robot states as a OutputVector.
 */
class RobotOutputConsumer : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor using the full plant.
   * @param plant The MultibodyPlant describing the robot.
   */
  explicit RobotOutputConsumer(
      const drake::multibody::MultibodyPlant<double>& plant);

  /**
   * @brief Constructor for a specific model instance.
   * @param plant The MultibodyPlant describing the robot.
   * @param model_instance The model instance index.
   */
  explicit RobotOutputConsumer(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::multibody::ModelInstanceIndex model_instance);

  /**
   * @brief Convenience function to initialize an lcmt_robot_output subscriber
   * with positions and velocities which are all zero except for the quaternion
   * positions, which are all 1, 0, 0, 0.
   * @param plant The MultibodyPlant describing the robot.
   * @param context The context of a
   * drake::LcmSubscriberSystem<lcmt_robot_output>.
   */
  void InitializeSubscriberPositions(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>& context) const;

 private:
  /**
   * @brief Copies the output from the context to the OutputVector.
   * @param context The system context.
   * @param output The output vector to populate.
   */
  void CopyOutput(const drake::systems::Context<double>& context,
                  OutputVector<double>* output) const;

  drake::multibody::ModelInstanceIndex
      model_instance_;       /**< Model instance index */
  int positions_start_idx_;  /**< Start index for positions */
  int velocities_start_idx_; /**< Start index for velocities */
  int num_positions_;        /**< Number of positions */
  int num_velocities_;       /**< Number of velocities */
  int num_efforts_;          /**< Number of efforts */
  std::map<std::string, int>
      position_index_map_; /**< Map from position name to index */
  std::map<std::string, int>
      velocity_index_map_; /**< Map from velocity name to index */
  std::map<std::string, int>
      effort_index_map_; /**< Map from effort name to index */
};

/**
 * @class RobotOutputGenerator
 * @brief Converts a OutputVector object to LCM type lcmt_robot_output.
 */
class RobotOutputGenerator : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor for a specific model instance.
   * @param plant The MultibodyPlant describing the robot.
   * @param model_instance_index The model instance index.
   * @param publish_efforts Whether to publish efforts.
   * @param publish_imu Whether to publish IMU data.
   */
  explicit RobotOutputGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::multibody::ModelInstanceIndex model_instance_index,
      const bool publish_efforts = false, const bool publish_imu = false);

  /**
   * @brief Constructor using the full plant.
   * @param plant The MultibodyPlant describing the robot.
   * @param publish_efforts Whether to publish efforts.
   * @param publish_imu Whether to publish IMU data.
   */
  explicit RobotOutputGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      const bool publish_efforts = false, const bool publish_imu = false);

  /**
   * @brief
   * @return The input port for state.
   */
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_);
  }

  /**
   * @brief
   * @return The input port for effort.
   */
  const drake::systems::InputPort<double>& get_input_port_effort() const {
    DRAKE_DEMAND(publish_efforts_);
    return this->get_input_port(effort_input_port_);
  }

  /**
   * @brief
   * @return The input port for IMU.
   */
  const drake::systems::InputPort<double>& get_input_port_imu() const {
    return this->get_input_port(imu_input_port_);
  }

 private:
  /**
   * @brief
   * @param context The system context.
   * @param output The LCM message to populate.
   */
  void Output(const drake::systems::Context<double>& context,
              dairlib::lcmt_robot_output* output) const;

  int positions_start_idx_;  /**< Start index for positions */
  int velocities_start_idx_; /**< Start index for velocities */
  int num_positions_;        /**< Number of positions */
  int num_velocities_;       /**< Number of velocities */
  int num_efforts_;          /**< Number of efforts */
  std::vector<std::string>
      ordered_position_names_; /**< Ordered position names */
  std::vector<std::string>
      ordered_velocity_names_;                    /**< Ordered velocity names */
  std::vector<std::string> ordered_effort_names_; /**< Ordered effort names */
  std::map<std::string, int>
      position_index_map_; /**< Map from position name to index */
  std::map<std::string, int>
      velocity_index_map_; /**< Map from velocity name to index */
  std::map<std::string, int>
      effort_index_map_;       /**< Map from effort name to index */
  int state_input_port_ = -1;  /**< Index of state input port */
  int effort_input_port_ = -1; /**< Index of effort input port */
  int imu_input_port_ = -1;    /**< Index of IMU input port */
  bool publish_efforts_;       /**< Whether to publish efforts */
  bool publish_imu_;           /**< Whether to publish IMU data */
};

}  // namespace lcmt_systems
}  // namespace systems
}  // namespace dairlib