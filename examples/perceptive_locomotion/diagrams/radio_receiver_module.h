#pragma once

// dairlib
#include "dairlib/lcmt_radio_out.hpp"
#include "dairlib/lcmt_cassie_out.hpp"

// drake
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/lcm/drake_lcm_interface.h"

namespace dairlib {
namespace perceptive_locomotion {

class VelocityCommander : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VelocityCommander);
  VelocityCommander() {
    this->DeclareAbstractInputPort(
        "lcmt_radio_output", drake::Value<lcmt_radio_out>());
    this->DeclareVectorOutputPort(
        "vdes", 2, &VelocityCommander::CalcOutput);
  };
 private:
  void CalcOutput(const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* out) const {
    constexpr double vel_scale_trans_sagittal = 0.8;
    constexpr double vel_scale_trans_lateral = -0.35;

    const auto& radio_out =
        this->EvalInputValue<dairlib::lcmt_radio_out>(context, 0);

    out->SetAtIndex(0, vel_scale_trans_sagittal * radio_out->channel[0]);
    out->SetAtIndex(1, vel_scale_trans_lateral * radio_out->channel[1]);
  }
};

class RadioReceiverModule : public drake::systems::Diagram<double> {
 public:
  RadioReceiverModule(const std::string& cassie_out_channel,
                      drake::lcm::DrakeLcmInterface* lcm);
};

}
}
