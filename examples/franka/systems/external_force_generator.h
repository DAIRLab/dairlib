#pragma once

#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

class ExternalForceGenerator : public drake::systems::LeafSystem<double> {
 public:
  ExternalForceGenerator(drake::multibody::BodyIndex body_index);

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_spatial_force()
      const {
    return this->get_output_port(spatial_force_port_);
  }

  void SetRemoteControlParameters(double x_scale, double y_scale,
                                  double z_scale);

 private:
  void CalcSpatialForce(const drake::systems::Context<double>& context,
                        std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* spatial_forces) const;

  drake::systems::InputPortIndex radio_port_;
  drake::systems::OutputPortIndex spatial_force_port_;
  drake::multibody::BodyIndex body_index_;
  double x_scale_ = 0;
  double y_scale_ = 0;
  double z_scale_ = 0;
};

}  // namespace dairlib
