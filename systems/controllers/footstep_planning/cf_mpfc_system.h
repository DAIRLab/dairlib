#pragma once
#include <dairlib/lcmt_saved_traj.hpp>
#include <dairlib/lcmt_cf_mpfc_solution.hpp>
#include <dairlib/lcmt_cf_mpfc_output.hpp>

#include "solvers/solver_options_io.h"
#include "cf_mpfc.h"
#include "nonlinear_pendulum_utils.h"
#include "geometry/convex_polygon_set.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_io.h"

namespace dairlib::systems::controllers {

using alip_utils::PointOnFramed;

struct cf_mpfc_params_io {
  int nmodes;
  int nknots;
  double mu;
  double foot_length;
  double time_regularization;
  double soft_constraint_cost;
  double rddot_rate_limit;
  std::vector<double> com_pos_bound;
  std::vector<double> com_vel_bound;
  std::vector<double> input_bounds;
  std::vector<double> Q;
  std::vector<double> R;
  std::vector<double> Qf;
  std::vector<double> Qc;
  std::vector<double> Rc;
  std::string tracking_cost_type;

  // gait params
  double height;
  double stance_width;
  double single_stance_duration;
  double double_stance_duration;
  std::string reset_discretization_method;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(nmodes));
    a->Visit(DRAKE_NVP(nknots));
    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(foot_length));
    a->Visit(DRAKE_NVP(time_regularization));
    a->Visit(DRAKE_NVP(soft_constraint_cost));
    a->Visit(DRAKE_NVP(com_pos_bound));
    a->Visit(DRAKE_NVP(com_vel_bound));
    a->Visit(DRAKE_NVP(input_bounds));
    a->Visit(DRAKE_NVP(rddot_rate_limit));
    a->Visit(DRAKE_NVP(Q));
    a->Visit(DRAKE_NVP(R));
    a->Visit(DRAKE_NVP(Qf));
    a->Visit(DRAKE_NVP(Qc));
    a->Visit(DRAKE_NVP(Rc));
    a->Visit(DRAKE_NVP(tracking_cost_type));
    a->Visit(DRAKE_NVP(height));
    a->Visit(DRAKE_NVP(single_stance_duration));
    a->Visit(DRAKE_NVP(double_stance_duration));
    a->Visit(DRAKE_NVP(stance_width));
    a->Visit(DRAKE_NVP(reset_discretization_method));
  }

  static cf_mpfc_params get_params_from_yaml(
      const std::string& yaml_path,
      const std::string &solver_options_yaml_path,
      const drake::multibody::MultibodyPlant<double> &plant,
      const drake::systems::Context<double> &context) {
    auto io = drake::yaml::LoadYamlFile<cf_mpfc_params_io>(yaml_path);
    cf_mpfc_params params_out;

    params_out.nmodes = io.nmodes;
    params_out.nknots = io.nknots;
    params_out.soft_constraint_cost = io.soft_constraint_cost;
    params_out.time_regularization = io.time_regularization;
    params_out.mu = io.mu;
    params_out.foot_length = io.foot_length;
    params_out.rddot_rate_limit = io.rddot_rate_limit;

    // gait params
    params_out.gait_params.height = io.height;
    params_out.gait_params.mass = plant.CalcTotalMass(context);
    params_out.gait_params.single_stance_duration = io.single_stance_duration;
    params_out.gait_params.double_stance_duration = io.double_stance_duration;
    params_out.gait_params.stance_width = io.stance_width;
    params_out.gait_params.reset_discretization_method =
        alip_utils::reset_discretization(io.reset_discretization_method);

    params_out.com_pos_bound =
        Eigen::Vector2d::Map(io.com_pos_bound.data());
    params_out.com_vel_bound =
        Eigen::Vector2d::Map(io.com_vel_bound.data());
    params_out.input_bounds =
        Eigen::Vector2d::Map(io.input_bounds.data());
    params_out.Q = Eigen::Map <
        Eigen::Matrix < double, 4, 4, Eigen::RowMajor >> (io.Q.data());
    params_out.Qf = Eigen::Map <
        Eigen::Matrix < double, 4, 4, Eigen::RowMajor >> (io.Qf.data());
    params_out.R = Eigen::Map <
        Eigen::Matrix < double, 3, 3, Eigen::RowMajor >> (io.R.data());
    params_out.Qc = Eigen::Map<
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(io.Qc.data());
    params_out.Rc = Eigen::Map<
        Eigen::Matrix<double, 2, 2, Eigen::RowMajor>>(io.Qc.data());
    params_out.tracking_cost_type =
        alip_utils::alip_tracking_cost_type(io.tracking_cost_type);


    solvers::SolverOptionsFromYaml solver_options_io;
    if (!solver_options_yaml_path.empty()) {
      solver_options_io =
          drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
              solver_options_yaml_path
          );
    }
    params_out.solver_options = solver_options_io.GetAsSolverOptions(
        drake::solvers::GurobiSolver::id());
    return params_out;
  }
};

class CFMPFCSystem : public drake::systems::LeafSystem<double> {
 public:

  // TODO (@Brian-Acosta) : Move stance durations to gains struct
  CFMPFCSystem(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* plant_context,
      drake::multibody::ModelInstanceIndex model_instance,
      std::vector<int> left_right_stance_fsm_states,
      std::vector<int> post_left_right_fsm_states,
      std::vector<PointOnFramed> left_right_foot,
      const cf_mpfc_params& mpfc_params);

  void MakeDrivenByStandaloneSimulator(double update_period) {
    DeclareInitializationUnrestrictedUpdateEvent(
        &CFMPFCSystem::UnrestrictedUpdate);
    DeclarePeriodicUnrestrictedUpdateEvent(
        update_period, 0, &CFMPFCSystem::UnrestrictedUpdate);
  }

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_vdes() const {
    return this->get_input_port(vdes_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_footholds() const {
    return this->get_input_port(foothold_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_elevation() const {
    return this->get_input_port(elevation_map_port_);
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
                     lcmt_cf_mpfc_output* out) const;
  void CopyMPFCDebug(const drake::systems::Context<double>& context,
                     lcmt_cf_mpfc_solution* msg) const;
  int GetFsmForOutput(const drake::systems::Context<double>& context) const;
  double GetPrevImpactTimeForOutput(
      const drake::systems::Context<double>& context) const;

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
  drake::systems::InputPortIndex elevation_map_port_;

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
  drake::systems::AbstractStateIndex mpc_solution_idx_;
  drake::systems::AbstractStateIndex footholds_idx_;

  // Multibody Plant Parameters
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::ModelInstanceIndex model_instance_;

  std::map<int, const alip_utils::PointOnFramed> stance_foot_map_;
  int nq_;
  int nv_;
  int nu_;

  // mpc object
  mutable CFMPFC trajopt_;

  // finite state machine management
  std::vector<int> left_right_stance_fsm_states_;
  std::vector<int> post_left_right_fsm_states_;
  double double_stance_duration_;
  double single_stance_duration_;


};
}