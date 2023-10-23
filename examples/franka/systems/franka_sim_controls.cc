#include <iostream>

#include "common/find_resource.h"
#include "examples/franka/systems/franka_sim_controls.h"
#include "multibody/multibody_utils.h"
#include <dairlib/lcmt_radio_out.hpp>

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;

using Eigen::VectorXd;
using systems::OutputVector;
using systems::TimestampedVector;

namespace systems {

FrankaSimControls::FrankaSimControls(const MultibodyPlant<double>& sim_plant,
                                     Context<double>* sim_context,
                                     ModelInstanceIndex franka_index,
                                     ModelInstanceIndex plate_index,
                                     ModelInstanceIndex box_index,
                                     VectorXd& q_franka_default,
                                     VectorXd& q_plate_default,
                                     VectorXd& q_box_default)
    : sim_plant_(sim_plant),
      sim_context_(sim_context),
      franka_index_(franka_index),
      plate_index_(plate_index),
      box_index_(box_index),
      q_franka_default_(q_franka_default),
      q_plate_default_(q_plate_default),
      q_box_default_(q_box_default){
  this->set_name("franka_sim_control");
  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();
  DeclareForcedDiscreteUpdateEvent(&FrankaSimControls::UpdateSimState);
}

drake::systems::EventStatus FrankaSimControls::UpdateSimState(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);

  if (radio_out->channel[15] > 0) {
    this->ResetSim(context);
  }
}

void FrankaSimControls::ResetSim(
    const drake::systems::Context<double>& context) const {
  sim_plant_.SetPositions(sim_context_, franka_index_, q_franka_default_);
  sim_plant_.SetPositions(sim_context_, plate_index_, q_plate_default_);
  sim_plant_.SetPositions(sim_context_, box_index_, q_box_default_);
}

//void FrankaSimControls::DropBox(
//    const drake::systems::Context<double>& context) const {
//  plant.SetPositions(franka_context_, q);
//}

}  // namespace systems
}  // namespace dairlib
