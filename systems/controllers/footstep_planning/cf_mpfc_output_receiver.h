#pragma once
#include "dairlib/lcmt_cf_mpfc_output.hpp"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/footstep_planning/contact_modes.h"

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
  CFMPFCOutputReceiver(std::vector<std::string> ordered_left_contact_names,
                       std::vector<std::string> ordered_right_contact_names,
                       const drake::multibody::MultibodyPlant<double>& plant);

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
  const drake::systems::OutputPort<double>& get_output_port_orientation()
  const {
    return get_output_port(output_port_orientation_);
  }
  const drake::systems::OutputPort<double>& get_output_port_com() const {
    return get_output_port(output_port_com_);
  }

 private:
  void CopyFsm(const drake::systems::Context<double>& context,
               lcmt_fsm_info* out) const;

  void CopyPitch(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* pitch) const;

  void CopyFootstepTarget(const drake::systems::Context<double>& context,
                          drake::systems::BasicVector<double>* target) const;

  void CopyOrientationTraj(const drake::systems::Context<double>& context,
                           drake::trajectories::Trajectory<double>* traj) const;

  void CalcOrientationTraj(const drake::systems::Context<double>& context,
                           drake::trajectories::PiecewiseQuaternionSlerp<double>* traj) const;

  void CopyComTraj(const drake::systems::Context<double>& context,
                   drake::trajectories::Trajectory<double>* traj) const;

  void CalcLcmTraj(const drake::systems::Context<double>& context,
                   LcmTrajectory* lcm_traj) const;

  void CalcComTraj(const drake::systems::Context<double>& context,
                   drake::trajectories::PiecewisePolynomial<double>* traj) const;

  void CalcForceTraj(const drake::systems::Context<double>& context,
                    drake::trajectories::PiecewisePolynomial<double>* traj) const;

  void CalcDesiredForce(const std::string& contact_name,
                        const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* f) const;

  void RegisterContact(const std::string& name);

  [[nodiscard]] const lcmt_cf_mpfc_output& get_mpfc_output(
      const drake::systems::Context<double>& context) const {
    return EvalAbstractInput(
        context, input_port_mpfc_output_)->get_value<lcmt_cf_mpfc_output>();
  }

  [[nodiscard]] bool mode_match (const std::string& contact_name, int fsm_state) const {
    if (std::find(left_contact_names_.begin(),
                  left_contact_names_.end(), contact_name) != left_contact_names_.end()) {
      return fsm_state == kLeft or fsm_state == kPostRightDouble;
    }
    return fsm_state == kRight or fsm_state == kPostLeftDouble;
  }

  const std::vector<std::string>& contacts_this_mode(int fsm_state) const {
    if (fsm_state == kRight or fsm_state == kPostLeftDouble) {
      return right_contact_names_;
    }
    return left_contact_names_;
  }

  const std::vector<std::string> left_contact_names_;
  const std::vector<std::string> right_contact_names_;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_mpfc_output_;

  drake::systems::OutputPortIndex output_port_fsm_;
  drake::systems::OutputPortIndex output_port_pitch_;
  drake::systems::OutputPortIndex output_port_footstep_target_;
  drake::systems::OutputPortIndex output_port_orientation_;
  drake::systems::OutputPortIndex output_port_com_;

  drake::systems::CacheIndex com_trajectory_cache_;
  drake::systems::CacheIndex wbo_trajectory_cache_;
  drake::systems::CacheIndex force_traj_cache_;
  drake::systems::CacheIndex lcm_traj_cache_;

  std::unordered_map<std::string, drake::systems::OutputPortIndex>
    desired_force_output_port_map_;
};

}
