//
// Created by brian on 12/7/20.
//

#include "cassie_disturbance_generator.h"
#include <cassie_utils.h>
#include <utility>
#include <dairlib/lcmt_osc_output.hpp>

using dairlib::CassieDisturbanceGenerator;
using drake::multibody::ExternallyAppliedSpatialForce;

CassieDisturbanceGenerator::CassieDisturbanceGenerator(
        const MultibodyPlant<double>& plant, Vector3d& f) {
    force_ = std::move(f);
    applied_force_output_port_ = this->DeclareAbstractOutputPort(
            "ExternallyAppliedSpatialForce",
            &CassieDisturbanceGenerator::MakeAppliedExternalForce).get_index();
    controller_input_port_ = this->DeclareAbstractInputPort("lcmt_osc_output",
                                                            drake::Value<dairlib::lcmt_osc_output>{}).get_index();
}

drake::systems::EventStatus CassieDisturbanceGenerator::DiscreteVariableUpdate(
        const drake::systems::Context<double> &context,
        drake::systems::DiscreteValues<double> discrete_state) const {

}


void CassieDisturbanceGenerator::MakeAppliedExternalForce(
            const Context<double>& context,
            ExternallyAppliedSpatialForce<double>* force ) const {
    auto osc_output = (dairlib::lcmt_osc_output*)this->EvalAbstractInput(context, controller_input_port_);

}

