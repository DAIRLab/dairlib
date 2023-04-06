#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "dairlib/lcmt_alip_mpc_output.hpp"

namespace dairlib::systems::controllers {
class AlipMpcOutputReceiver : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AlipMpcOutputReceiver);

  AlipMpcOutputReceiver();

  const drake::systems::OutputPort<double> &get_output_port_fsm() const {
    return this->get_output_port(fsm_output_port_);
  }
  const drake::systems::OutputPort<double> &get_output_port_footstep_target()
  const {
    return this->get_output_port(footstep_target_output_port_);
  }
  const drake::systems::OutputPort<double> &get_output_port_slope_parameters()
  const {
    return this->get_output_port(slope_parameters_output_port_);
  }
  const drake::systems::OutputPort<double> &get_output_port_pitch() const {
    return this->get_output_port(pitch_output_port_);
  }
  const drake::systems::OutputPort<double> &get_output_port_ankle_torque()
  const {
    return this->get_output_port(ankle_torque_output_port_);
  }
 private:
  void CopyFsm(const drake::systems::Context<double> &context,
               lcmt_fsm_info *fsm_info) const;
  void CopyFootstepTarget(const drake::systems::Context<double> &context,
                          drake::systems::BasicVector<double> *out) const;
  void CopySlopeParameters(const drake::systems::Context<double> &context,
                           drake::systems::BasicVector<double> *out) const;
  void CopyPitch(const drake::systems::Context<double> &context,
                 drake::systems::BasicVector<double> *out) const;
  void CopyAnkleTorque(const drake::systems::Context<double> &context,
                       lcmt_saved_traj *out) const;

  drake::systems::InputPortIndex alip_mpc_input_port_;
  drake::systems::OutputPortIndex fsm_output_port_;
  drake::systems::OutputPortIndex footstep_target_output_port_;
  drake::systems::OutputPortIndex slope_parameters_output_port_;
  drake::systems::OutputPortIndex pitch_output_port_;
  drake::systems::OutputPortIndex ankle_torque_output_port_;
};
}