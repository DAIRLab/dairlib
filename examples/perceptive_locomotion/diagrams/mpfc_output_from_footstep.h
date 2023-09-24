#pragma once

#include "dairlib/lcmt_alip_mpc_output.hpp"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"


namespace dairlib::perceptive_locomotion {

class MpfcOutputFromFootstep : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MpfcOutputFromFootstep)

  MpfcOutputFromFootstep(double T_ss, double T_ds,
                         const drake::multibody::MultibodyPlant<double>& plant);
private:

  lcmt_fsm_info MakeFsmInfo(double t) const;
  void CalcOutput(const drake::systems::Context<double>& context,
                  lcmt_alip_mpc_output* output) const;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_footstep_;

  drake::systems::OutputPortIndex output_port_mpc_output_;

  const double Tss_;
  const double Tds_;


};

}
