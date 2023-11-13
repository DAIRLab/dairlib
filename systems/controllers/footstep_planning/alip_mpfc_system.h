#pragma once
#include <dairlib/lcmt_saved_traj.hpp>
#include <dairlib/lcmt_mpc_debug.hpp>
#include <dairlib/lcmt_mpc_solution.hpp>
#include <dairlib/lcmt_alip_mpc_output.hpp>

#include "alip_utils.h"
#include "alip_multiqp.h"
#include "alip_miqp.h"
#include "geometry/convex_polygon_set.h"
#include "systems/filters/s2s_kalman_filter.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems::controllers {
using alip_utils::PointOnFramed;

struct AlipMINLPGains {
  double t_commit;
  double t_min;
  double t_max;
  double u_max;
  double hdes;
  double stance_width;
  double next_footstep_constraint_radius;
  double xlim;
  int nmodes;
  int knots_per_mode;
  alip_utils::ResetDiscretization reset_discretization_method;
  bool filter_alip_state;
  Eigen::Matrix4d Q;
  Eigen::Matrix4d Qf;
  Eigen::Matrix3d W_footstep_reg;
  Eigen::MatrixXd R;
  S2SKalmanFilterData filter_data;
};

class AlipMPFC : public drake::systems::LeafSystem<double> {
 public:

  // TODO (@Brian-Acosta) : Move stance durations to gains struct
  AlipMPFC(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* plant_context,
      std::vector<int> left_right_stance_fsm_states,
      std::vector<int> post_left_right_fsm_states,
      std::vector<double> left_right_stance_durations,
      double double_stance_duration,
      std::vector<PointOnFramed> left_right_foot,
      const AlipMINLPGains& gains,
      const drake::solvers::SolverOptions& trajopt_solver_options);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_vdes() const {
    return this->get_input_port(vdes_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_footholds() const {
    return this->get_input_port(foothold_input_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_mpc_output() const {
    return this->get_output_port(mpc_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_mpc_debug() const {
    return this->get_output_port(mpc_debug_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_fsm() const {
    return this->get_output_port(fsm_output_port_);
  }


 private:

  // System callbacks
  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void CopyMpcOutput(const drake::systems::Context<double>& context,
                     lcmt_alip_mpc_output* traj) const;
  void CopyAnkleTorque(const drake::systems::Context<double>& context,
                       lcmt_saved_traj* traj) const;
  void CopyFsmOutput(const drake::systems::Context<double>& context,
                     drake::systems::BasicVector<double>* fsm) const;
  int GetFsmForOutput(const drake::systems::Context<double>& context) const;
  double GetPrevImpactTimeForOutput(
      const drake::systems::Context<double>& context) const;

  void CopyMpcDebugToLcm(const drake::systems::Context<double>& context,
                         lcmt_mpc_debug* mpc_debug) const;

  // Lcm helper functions
  void CopyMpcSolutionToLcm(const std::vector<Eigen::Vector3d>& pp,
                            const std::vector<Eigen::VectorXd>& xx,
                            const std::vector<Eigen::VectorXd>& uu,
                            const Eigen::VectorXd& tt,
                            lcmt_mpc_solution* solution) const;

  // FSM helper functions
  int curr_fsm(int fsm_idx) const {
    return left_right_stance_fsm_states_.at(fsm_idx);
  }
  int next_fsm(int fsm_idx) const {
    int next= fsm_idx + 1;
    if (next >= left_right_stance_fsm_states_.size()) {
      return curr_fsm(0);
    }
    return curr_fsm(next);
  }

  // drake input ports
  drake::systems::InputPortIndex state_input_port_;
  drake::systems::InputPortIndex vdes_input_port_;
  drake::systems::InputPortIndex foothold_input_port_;

  // drake output ports
  drake::systems::OutputPortIndex mpc_output_port_;
  drake::systems::OutputPortIndex mpc_debug_output_port_;
  drake::systems::OutputPortIndex fsm_output_port_;

  // controller states
  drake::systems::DiscreteStateIndex fsm_state_idx_;
  drake::systems::DiscreteStateIndex next_impact_time_state_idx_;
  drake::systems::DiscreteStateIndex prev_impact_time_state_idx_;
  drake::systems::DiscreteStateIndex initial_conditions_state_idx_;

  // abstract states
  drake::systems::AbstractStateIndex alip_filter_idx_;

  // Multibody Plant Parameters
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  std::map<int, const alip_utils::PointOnFramed> stance_foot_map_;
  int nq_;
  int nv_;
  int nu_;

  // mpc object
  mutable AlipMIQP trajopt_;

  // finite state machine management
  std::vector<int> left_right_stance_fsm_states_;
  std::vector<int> post_left_right_fsm_states_;
  double double_stance_duration_;
  double single_stance_duration_;

  // gains
  AlipMINLPGains gains_;
};
}