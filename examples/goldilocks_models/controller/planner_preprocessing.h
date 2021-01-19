#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace goldilocks_models {

///
/// CurrentStanceFoot
///
class CurrentStanceFoot : public drake::systems::LeafSystem<double> {
 public:
  CurrentStanceFoot(const std::vector<int>& left_right_support_fsm_states);

  const drake::systems::InputPort<double>& get_input_port_fsm_and_lo_time()
      const {
    return this->get_input_port(fsm_and_lo_time_port_);
  }

 private:
  void GetStance(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* stance_foot) const;
  int fsm_and_lo_time_port_;
  std::vector<int> left_right_support_fsm_states_;
  mutable bool start_with_right_stance_ = false;
};

///
/// PhaseInFirstMode
///

class PhaseInFirstMode : public drake::systems::LeafSystem<double> {
 public:
  PhaseInFirstMode(
      const drake::multibody::MultibodyPlant<double>& plant_feedback,
      double stride_period);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_and_lo_time()
      const {
    return this->get_input_port(fsm_and_lo_time_port_);
  }

 private:
  void CalcPhase(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* init_phase_output) const;

  // Port indices
  int state_port_;
  int fsm_and_lo_time_port_;

  double stride_period_;
};

///
/// InitialStateForPlanner
///
class InitialStateForPlanner : public drake::systems::LeafSystem<double> {
 public:
  InitialStateForPlanner(
      const drake::multibody::MultibodyPlant<double>& plant_feedback,
      const drake::multibody::MultibodyPlant<double>& plant_controls,
      double final_position_x, int n_step);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_init_phase() const {
    return this->get_input_port(phase_port_);
  }

 private:
  void CalcState(const drake::systems::Context<double>& context,
                 systems::OutputVector<double>* output) const;

  // Port indices
  int state_port_;
  int phase_port_;

  // Map position/velocity from model with spring to without spring
  Eigen::MatrixXd map_position_from_spring_to_no_spring_;
  Eigen::MatrixXd map_velocity_from_spring_to_no_spring_;

  std::map<std::string, int> positions_map_;

  int nq_;

  // Parameters for traj opt
  double final_position_x_;
  int n_step_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
