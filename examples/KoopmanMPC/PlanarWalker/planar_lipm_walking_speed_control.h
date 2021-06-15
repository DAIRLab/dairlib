#pragma once

#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace koopman_examples{



/// re-implements examples/Cassie/osc/WalkingSpeedControl for the planar case
class PlanarLipmWalkingSpeedControl : public drake::systems::LeafSystem<double> {
 public:
  PlanarLipmWalkingSpeedControl(const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context, double k_ff, double k_fb,
      std::string base_name, double swing_phase_duration=0);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_des_vel() const {
    return this->get_input_port(v_des_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
  const {
    return this->get_input_port(fsm_switch_time_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_com() const {
    return this->get_input_port(com_port_);
  }

 private:
  void CalcFootPlacement(const drake::systems::Context<double>& context,
                         drake::systems::BasicVector<double>* output) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;
  const drake::multibody::Body<double>& pelvis_;

  double swing_phase_duration_;
  bool is_using_predicted_com_;

  double k_ff_;
  double k_fb_;
  int state_port_;
  int v_des_port_;
  int fsm_switch_time_port_;
  int com_port_;

  // com vel filtering -- necessary?
  double cutoff_freq_ = 10;  // in Hz.
  mutable Eigen::Vector3d filterred_com_vel_ = Eigen::Vector3d::Zero();
  mutable double last_timestamp_ = 0;

};

}}