#pragma once

#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {
namespace systems {

/**
 * @brief Generates external spatial forces for a specified body.
 *
 * This system outputs a spatial force vector for a body, with force values
 * determined by radio input and scaling parameters.
 */
class ExternalForceGenerator : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor. Declares input/output ports for force generation.
   * @param body_index The index of the body to apply the force to.
   */
  ExternalForceGenerator(drake::multibody::BodyIndex body_index);

  /// Input/output port accessors
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_spatial_force()
      const {
    return this->get_output_port(spatial_force_port_);
  }

  /**
   * @brief Sets scaling for remote control force input.
   * @param x_scale Scaling for x force.
   * @param y_scale Scaling for y force.
   * @param z_scale Scaling for z force.
   */
  void SetRemoteControlParameters(double x_scale, double y_scale,
                                  double z_scale);

 private:
  /**
   * @brief Calculates the spatial force output based on radio input.
   */
  void CalcSpatialForce(
      const drake::systems::Context<double>& context,
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>*
          spatial_forces) const;

  drake::systems::InputPortIndex radio_port_;  ///< Input port for radio.
  drake::systems::OutputPortIndex
      spatial_force_port_;                  ///< Output port for spatial force.
  drake::multibody::BodyIndex body_index_;  ///< Body index to apply force.
  double x_scale_ = 0;                      ///< Scaling for x force.
  double y_scale_ = 0;                      ///< Scaling for y force.
  double z_scale_ = 0;                      ///< Scaling for z force.
};

}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib
