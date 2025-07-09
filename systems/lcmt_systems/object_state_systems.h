#pragma once

#include <map>
#include <string>
#include <vector>

#include "dairlib/lcmt_object_state.hpp"
#include "systems/framework/state_vector.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace systems {
namespace lcmt_systems {

/**
 * @file This file contains classes dealing with sending/receiving
 * LCM messages related to an object.
 */

/**
 * @class ObjectStateConsumer
 * @brief Receives the output of an LcmSubscriberSystem that subscribes to the
 * object state channel with LCM type lcmt_object_state, and outputs the
 * object state as a StateVector.
 */
class ObjectStateConsumer : public drake::systems::LeafSystem<double> {
 public:
    /**
     * @brief Constructor using the given MultibodyPlant.
     * @param plant The MultibodyPlant describing the object.
     */
    explicit ObjectStateConsumer(
            const drake::multibody::MultibodyPlant<double>& plant);

    /**
     * @brief Constructor using the given MultibodyPlant and model instance.
     * @param plant The MultibodyPlant describing the object.
     * @param model_instance The model instance index.
     */
    explicit ObjectStateConsumer(
            const drake::multibody::MultibodyPlant<double>& plant,
            drake::multibody::ModelInstanceIndex model_instance);

    /**
     * @brief Convenience function to initialize an lcmt_object_state subscriber with
     * positions and velocities which are all zero except for the quaternion
     * positions, which are all 1, 0, 0, 0.
     * @param plant The MultibodyPlant describing the object.
     * @param context The context of a drake::LcmSubscriberSystem<lcmt_object_state>.
     */
    void InitializeSubscriberPositions(
            const drake::multibody::MultibodyPlant<double>& plant,
            drake::systems::Context<double>& context) const;

 private:
    /**
     * @brief Copies the output from the context to the StateVector output.
     * @param context The system context.
     * @param output The output StateVector.
     */
    void CopyOutput(const drake::systems::Context<double>& context,
                                    StateVector<double>* output) const;

    drake::multibody::ModelInstanceIndex model_instance_; /**< Model instance index */
    int positions_start_idx_; /**< Start index for positions */
    int velocities_start_idx_; /**< Start index for velocities */
    int num_positions_; /**< Number of positions */
    int num_velocities_; /**< Number of velocities */
    std::map<std::string, int> position_index_map_; /**< Map from position name to index */
    std::map<std::string, int> velocity_index_map_; /**< Map from velocity name to index */
};

/**
 * @class ObjectStateGenerator
 * @brief Converts a StateVector object to LCM type lcmt_object_state.
 */
class ObjectStateGenerator : public drake::systems::LeafSystem<double> {
 public:
    /**
     * @brief Constructor using the given MultibodyPlant, with option to publish velocities.
     * @param plant The MultibodyPlant describing the object.
     * @param publish_velocities Whether to publish velocities.
     * @param model_instance_index The model instance index.
     */
    explicit ObjectStateGenerator(
            const drake::multibody::MultibodyPlant<double>& plant, bool publish_velocities = true,
            drake::multibody::ModelInstanceIndex model_instance_index =
            drake::multibody::default_model_instance());

    /**
 * @brief 
     * @param plant The MultibodyPlant describing the object.
     */
    explicit ObjectStateGenerator(
            const drake::multibody::MultibodyPlant<double>& plant);

    /**
     * @brief 
     * @return The input port for the state.
     */
    const drake::systems::InputPort<double>& get_input_port_state() const {
        return this->get_input_port(state_input_port_);
    }

 private:
    /**
     * @brief 
     * @param context The system context.
     * @param output The output lcmt_object_state message.
     */
    void Output(const drake::systems::Context<double>& context,
                            dairlib::lcmt_object_state* output) const;

    drake::multibody::ModelInstanceIndex model_instance_; /**< Model instance index */
    int positions_start_idx_; /**< Start index for positions */
    int velocities_start_idx_; /**< Start index for velocities */
    int num_positions_; /**< Number of positions */
    int num_velocities_; /**< Number of velocities */
    std::vector<std::string> ordered_position_names_; /**< Ordered position names */
    std::vector<std::string> ordered_velocity_names_; /**< Ordered velocity names */
    std::map<std::string, int> position_index_map_; /**< Map from position name to index */
    std::map<std::string, int> velocity_index_map_; /**< Map from velocity name to index */
    drake::systems::InputPortIndex state_input_port_; /**< State input port index */
    bool publish_velocities_; /**< Whether to publish velocities */
};

} // namespace lcmt_systems
} // namespace systems
} // namespace dairlib