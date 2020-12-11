//
// Created by brian on 12/7/20.
//

#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/externally_applied_spatial_force.h>

namespace dairlib {
using drake::multibody::MultibodyPlant;
using drake::multibody::ExternallyAppliedSpatialForce;
using drake::systems::Context;
using Eigen::Vector3d;

    class CassieDisturbanceGenerator : public drake::systems::LeafSystem<double> {
    public:
        CassieDisturbanceGenerator(const MultibodyPlant<double> &plant, Vector3d& f);

        const drake::systems::OutputPort<double>& get_applied_force_output_port() const {
            return this->get_output_port(applied_force_output_port_);
        }
        const drake::systems::InputPort<double>& get_state_input_port() const {
            return this->get_input_port(controller_input_port_);
        }

    private:
        Vector3d force_;
        drake::systems::EventStatus DiscreteVariableUpdate(
                const Context<double>& context,
                drake::systems::DiscreteValues<double> discrete_state) const;

        void MakeAppliedExternalForce(
                const Context<double>& context,
                ExternallyAppliedSpatialForce<double>* force) const;
        int applied_force_output_port_;
        int controller_input_port_;
    };

}