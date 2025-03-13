#pragma once
#include "walking_utils.h"
#include "systems/controllers/id_mpc/id_mpc.h"

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::systems::controllers::id_mpc {

class WalkingReferenceSystem : public drake::systems::LeafSystem<double> {
 public:

  WalkingReferenceSystem(
      const ConstrainedDynamicsInfo& dynamics,
      drake::systems::Context<double> *plant_context,
      const GaitParams& params);

  [[nodiscard]] const drake::systems::InputPort<double>& get_input_port_state()
  const {
    return get_input_port(input_port_state_);
  }

  [[nodiscard]] const drake::systems::InputPort<double>& get_input_port_vdes()
  const {
    return get_input_port(input_port_vdes_);
  }

  void MakeDrivenByStandaloneSimulator(double update_period);

 private:

  drake::systems::EventStatus
  UnrestrictedUpdate(const drake::systems::Context<double>& context,
                drake::systems::State<double>* state) const;

  fsm_info CalcFSM(double t, const fsm_info& curr_fsm) const;

  std::vector<fsm_info::fsm_state> CalcGaitTiming(
      double t, const fsm_info& fsm, MPCReference* mpc_reference) const;

  void SetContactsAtKnot(
      int i, fsm_info::fsm_state state, MPCReference* ref) const;

  // Note that these methods assume the context is up-to-date with the
  // robot's current state
  // TODO (@Brian-Acosta) include yaw rate in the reference
  // TODO (@Brian-Acosta) add height lookup for position
  drake::trajectories::PiecewisePolynomial<double> CalcPositionTraj(
      fsm_info::fsm_state fsm_state, const Eigen::Vector2d& vdes,
      const std::vector<double>& breaks) const;
  drake::trajectories::PiecewisePolynomial<double> CalcOrientationTraj() const;
  drake::trajectories::PiecewisePolynomial<double> CalcVelocityTraj(
      const Eigen::Vector2d& vdes) const;

  drake::trajectories::PiecewisePolynomial<double> CalcInputTraj(
      const std::vector<double>& breaks,
      const std::vector<fsm_info::fsm_state> fsm_states) const;

  drake::trajectories::PiecewisePolynomial<double> CalcLambdaTraj(
      const std::vector<double>& breaks,
      const std::vector<fsm_info::fsm_state>& fsm_states,
      const std::vector<std::vector<std::string>>& active_contacts) const;

  drake::trajectories::PiecewisePolynomial<double> CalcSwingFootTraj(
      const std::vector<double>& breaks,
      const std::vector<fsm_info::fsm_state>& fsm_states,
      const std::vector<double> phases) const;

  std::vector<double> CalcSSPhaseVector(
      const fsm_info& fsm,
      const std::vector<double>& breaks,
      const std::vector<fsm_info::fsm_state>& fsm_vector) const;

  const ConstrainedDynamicsInfo& dynamics_;
  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::systems::Context<double> *plant_context_;
  GaitParams params_;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_vdes_;

  drake::systems::AbstractStateIndex reference_state_idx_;
  drake::systems::AbstractStateIndex fsm_info_idx_;

  int ds_intervals_;
  int ss_intervals_;
};

}