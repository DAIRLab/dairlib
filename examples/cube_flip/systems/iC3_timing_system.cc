#include "iC3_timing_system.h"

#include <iostream>
#include <utility>

#include "common/find_resource.h"
#include "dairlib/lcmt_radio_out.hpp"



namespace dairlib {

using drake::systems::DiscreteValues;

iC3TimingSystem::iC3TimingSystem(iC3Options ic3_options, FrankaPlateControllerParams franka_controller_params)
    : ic3_options_(ic3_options),
      franka_controller_params_(franka_controller_params),
      example_idx_(0)
  {
		
  this->set_name("ic3_timing_system");

  radio_port_ =
    this->DeclareAbstractInputPort("lcmt_radio_out",
                                    drake::Value<lcmt_radio_out>{})
        .get_index();
          

  index_output_port_ =
    this->DeclareVectorOutputPort(
            "target_port",
            BasicVector<double>(1),
            &iC3TimingSystem::OutputIndex)
        .get_index();

  t0_idx_ = this->DeclareDiscreteState(1); 
	called_ = false;      

  this->DeclarePerStepDiscreteUpdateEvent(
    &iC3TimingSystem::SetFirstCallTime);
}

iC3TimingSystem::iC3TimingSystem(iC3Options ic3_options, TrifingerControllerParams trifinger_controller_params)
    : ic3_options_(ic3_options),
      trifinger_controller_params_(trifinger_controller_params),
      example_idx_(1)
  {
		
  this->set_name("ic3_timing_system");

  radio_port_ =
    this->DeclareAbstractInputPort("lcmt_radio_out",
                                    drake::Value<lcmt_radio_out>{})
        .get_index();
          

  index_output_port_ =
    this->DeclareVectorOutputPort(
            "target_port",
            BasicVector<double>(1),
            &iC3TimingSystem::OutputIndex)
        .get_index();

  t0_idx_ = this->DeclareDiscreteState(1); 
	called_ = false;    
  
  this->DeclarePerStepDiscreteUpdateEvent(
    &iC3TimingSystem::SetFirstCallTime);
}


void iC3TimingSystem::OutputIndex(
  const Context<double>& context, BasicVector<double>* index) const {

  double delay;
  if (example_idx_ == 0) {
    delay = franka_controller_params_.time_to_wait;
  } else if (example_idx_ == 1) {
    delay = trifinger_controller_params_.time_to_wait;
  }

  double t0 = context.get_discrete_state(t0_idx_).GetAtIndex(0);
  double curr_time = context.get_time() - t0;  
  double ic3_dt = ic3_options_.dt;
  int ic3_timestep = (curr_time - delay) / ic3_dt;

  VectorXd index_vector(1);
  
  // Set negative time if in teleop
  if (!called_) {
    index_vector(0) = -999;
  } else {
    index_vector(0) = ic3_timestep;
  }

  index->get_mutable_value() = index_vector;
}


drake::systems::EventStatus iC3TimingSystem::SetFirstCallTime(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {

  const auto* radio_out =
      this->EvalInputValue<lcmt_radio_out>(context, radio_port_);
  
  bool is_teleop = false;

  // HARDCODED, only do teleop if plate example
  if (example_idx_ == 0) {
    is_teleop = radio_out->channel[14];
  }


  auto& vec = discrete_state->get_mutable_vector(t0_idx_);
  if (!called_ && !is_teleop) {  
    vec.SetAtIndex(0, context.get_time());
		called_ = true;
  }

  if (is_teleop) {
    called_ = false;
  }
  return drake::systems::EventStatus::Succeeded();
}

} // namespace dairlib