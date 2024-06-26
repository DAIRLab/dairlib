#pragma once
#include "dairlib/lcmt_cf_mpfc_output.hpp"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/footstep_planning/contact_modes.h"
#include "systems/controllers/footstep_planning/alip_utils.h"

#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::systems::controllers {

class CFMPFCOutputReceiver : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CFMPFCOutputReceiver);

  /*!
   * Constructs the MPFC receiver, which converts the pmfc output message into
   * trajectories, etc. which are usable by downstream systems
   *
   * @param ordered_left_contact_names left foot contact point names, as they
   * appear in the OSC, in the order that they are declared to the MPC
   *
   * @param ordered_right_contact_names right foot contact names, following the
   * same convention as above
   */
  CFMPFCOutputReceiver(const drake::multibody::MultibodyPlant<double>& plant);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return get_input_port(input_port_state_);
  }
  const drake::systems::InputPort<double>& get_input_port_mpfc_output() const {
    return get_input_port(input_port_mpfc_output_);
  }
  const drake::systems::OutputPort<double>& get_output_port_fsm() const {
    return get_output_port(output_port_fsm_);
  }
  const drake::systems::OutputPort<double>& get_output_port_pitch() const {
    return get_output_port(output_port_pitch_);
  }
  const drake::systems::OutputPort<double>& get_output_port_footstep_target()
  const {
    return get_output_port(output_port_footstep_target_);
  }
  const drake::systems::OutputPort<double>& get_output_port_ankle_torque()
  const {
    return get_output_port(output_port_ankle_torque_);
  }
  const drake::systems::OutputPort<double>& get_output_port_r_traj() const {
    return get_output_port(output_port_r_traj_);
  }
 private:
  void CopyFsm(const drake::systems::Context<double>& context,
               lcmt_fsm_info* out) const;

  void CopyPitch(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* pitch) const;

  void CopyFootstepTarget(const drake::systems::Context<double>& context,
                          drake::systems::BasicVector<double>* target) const;

  void CalcInputTraj(const drake::systems::Context<double>& context,
                     drake::trajectories::PiecewisePolynomial<double>* traj) const;

  void CopyAnkleTorque(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* u) const;

  void CopyRTraj(const drake::systems::Context<double>& context,
                 drake::trajectories::Trajectory<double>* u) const;

  [[nodiscard]] const lcmt_cf_mpfc_output& get_mpfc_output(
      const drake::systems::Context<double>& context) const {
    return EvalAbstractInput(
        context, input_port_mpfc_output_)->get_value<lcmt_cf_mpfc_output>();
  }


  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_mpfc_output_;

  drake::systems::OutputPortIndex output_port_fsm_;
  drake::systems::OutputPortIndex output_port_pitch_;
  drake::systems::OutputPortIndex output_port_footstep_target_;
  drake::systems::OutputPortIndex output_port_ankle_torque_;
  drake::systems::OutputPortIndex output_port_r_traj_;

  drake::systems::CacheIndex traj_cache_;

  std::string floating_base_ = "pelvis";
};

}
