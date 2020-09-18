#pragma once

#include "systems/framework/output_vector.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace cassie {
namespace osc {

class VdotIntegrator : public drake::systems::LeafSystem<double> {
 public:
  VdotIntegrator(const drake::multibody::MultibodyPlant<double>& plant_w_spr,
                 const drake::multibody::MultibodyPlant<double>& plant_wo_spr);

  void SetInitialTime(drake::systems::Context<double>* context,
                      double time) const;
  void SetInitialState(drake::systems::Context<double>* context,
                       const Eigen::VectorXd& state) const;

  const drake::systems::InputPort<double>& get_robot_output_input_port() const {
    return this->get_input_port(feddback_state_port_);
  }
  const drake::systems::InputPort<double>& get_osc_vdot_input_port() const {
    return this->get_input_port(vdot_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CopyState(const drake::systems::Context<double>& context,
                 systems::TimestampedVector<double>* output) const;

  int feddback_state_port_;
  int vdot_port_;

  int prev_time_idx_;
  int actuated_q_idx_;
  int actuated_v_idx_;

  int nq_spr_;
  int nv_spr_;
  int nx_spr_;

  Eigen::MatrixXd map_from_q_spring_to_q_actuated_joints_;
  Eigen::MatrixXd map_from_v_spring_to_v_actuated_joints_;

  Eigen::MatrixXd map_from_v_no_spring_to_v_actuated_joints_;
  Eigen::MatrixXd map_from_q_actuated_joints_to_q_spring_;
  Eigen::MatrixXd map_from_v_actuated_joints_to_v_spring_;

  // initialization flag
  mutable bool has_been_initialized_ = false;
};

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
