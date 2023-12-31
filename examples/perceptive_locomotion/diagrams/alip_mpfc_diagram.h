#pragma once

#include "drake/solvers/solver_options.h"
#include "drake/systems/framework/diagram.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "examples/perceptive_locomotion/gains/alip_mpfc_gains.h"


namespace dairlib {
namespace perceptive_locomotion {

class AlipMPFCDiagram  : public drake::systems::Diagram<double> {

 public:
  AlipMPFCDiagram(const drake::multibody::MultibodyPlant<double>& plant,
                  const std::string& gains_filename,
                  double debug_publish_period);

  const drake::systems::InputPort<double>& get_input_port_state() const {
      return get_input_port(input_port_state_);
  }
  const drake::systems::InputPort<double>& get_input_port_footholds() const {
      return get_input_port(input_port_footholds_);
  }
  const drake::systems::InputPort<double>& get_input_port_vdes() const {
      return get_input_port(input_port_vdes_);
  }
  const drake::systems::OutputPort<double>& get_output_port_mpc_output() const {
      return get_output_port(output_port_mpc_output_);
  }

 private:


  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> plant_context_;

  // finite state machine
  int left_stance_state = 0;
  int right_stance_state = 1;
  int post_left_double_support_state = 3;
  int post_right_double_support_state = 4;

  systems::controllers::alip_utils::PointOnFramed left_toe;
  systems::controllers::alip_utils::PointOnFramed left_heel;
  systems::controllers::alip_utils::PointOnFramed left_toe_mid;
  systems::controllers::alip_utils::PointOnFramed right_toe_mid;
  std::vector<systems::controllers::alip_utils::PointOnFramed> left_right_toe;

  drake::solvers::SolverOptions planner_solver_options;

  std::vector<int> left_right_fsm_states =
      {left_stance_state, right_stance_state};
  std::vector<int> post_left_right_fsm_states =
      {post_right_double_support_state, post_left_double_support_state};

  std::vector<double> state_durations;

  AlipMpfcGainsImport gains_mpc;

  drake::systems::InputPortIndex input_port_footholds_;
  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_vdes_;
  drake::systems::OutputPortIndex output_port_mpc_output_;

};

} // dairlib
} // perceptive_locomotion

