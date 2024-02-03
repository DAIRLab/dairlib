#pragma once

#include "solvers/solver_options_io.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_io.h"


namespace dairlib::systems::controllers {

struct alip_s2s_mpfc_params {
  alip_utils::AlipGaitParams gait_params;
  int nmodes;
  double tmin;
  double tmax;
  double soft_constraint_cost;
  double time_regularization;
  Eigen::Vector2d com_pos_bound;
  Eigen::Vector2d com_vel_bound;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd Qf;
  drake::solvers::SolverOptions solver_options;
};

struct alip_s2s_mpfc_params_io {
  int nmodes;
  double tmin;
  double tmax;
  double height;
  double single_stance_duration;
  double double_stance_duration;
  double soft_constraint_cost;
  double time_regularization;
  double stance_width;
  std::string reset_discretization_method;
  std::vector<double> com_pos_bound;
  std::vector<double> com_vel_bound;
  std::vector<double> Q;
  std::vector<double> R;
  std::vector<double> Qf;

  template<typename Archive>
  void Serialize(Archive *a) {
    a->Visit(DRAKE_NVP(nmodes));
    a->Visit(DRAKE_NVP(tmin));
    a->Visit(DRAKE_NVP(tmax));
    a->Visit(DRAKE_NVP(height));
    a->Visit(DRAKE_NVP(single_stance_duration));
    a->Visit(DRAKE_NVP(double_stance_duration));
    a->Visit(DRAKE_NVP(soft_constraint_cost));
    a->Visit(DRAKE_NVP(time_regularization));
    a->Visit(DRAKE_NVP(stance_width));
    a->Visit(DRAKE_NVP(reset_discretization_method));
    a->Visit(DRAKE_NVP(com_pos_bound));
    a->Visit(DRAKE_NVP(com_vel_bound));
    a->Visit(DRAKE_NVP(Q));
    a->Visit(DRAKE_NVP(R));
    a->Visit(DRAKE_NVP(Qf));
  }
};

inline alip_s2s_mpfc_params MakeAlipS2SMPFCParamsFromYaml(
    const std::string &gains_yaml_path,
    const std::string &solver_options_yaml_path,
    const drake::multibody::MultibodyPlant<double> &plant,
    const drake::systems::Context<double> &context) {

  auto params_io = drake::yaml::LoadYamlFile<alip_s2s_mpfc_params_io>(
      gains_yaml_path
  );
  auto solver_options_io =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          solver_options_yaml_path
      );

  alip_s2s_mpfc_params params_out;
  params_out.nmodes = params_io.nmodes;
  params_out.tmin = params_io.tmin;
  params_out.tmax = params_io.tmax;
  params_out.soft_constraint_cost = params_io.soft_constraint_cost;
  params_out.time_regularization = params_io.time_regularization;
  params_out.gait_params.height = params_io.height;
  params_out.gait_params.mass = plant.CalcTotalMass(context);
  params_out.gait_params.single_stance_duration =
      params_io.single_stance_duration;
  params_out.gait_params.double_stance_duration =
      params_io.double_stance_duration;
  params_out.gait_params.stance_width = params_io.stance_width;

  DRAKE_DEMAND(params_io.reset_discretization_method == "ZOH" ||
      params_io.reset_discretization_method == "FOH" ||
      params_io.reset_discretization_method == "SPLIT");

  alip_utils::ResetDiscretization
      reset_disc = alip_utils::ResetDiscretization::kZOH;
  if (params_io.reset_discretization_method == "FOH") {
    reset_disc = alip_utils::ResetDiscretization::kFOH;
  }
  if (params_io.reset_discretization_method == "SPLIT") {
    reset_disc = alip_utils::ResetDiscretization::kSPLIT;
  }
  params_out.gait_params.reset_discretization_method = reset_disc;

  params_out.com_pos_bound =
      Eigen::Vector2d::Map(params_io.com_vel_bound.data());
  params_out.com_vel_bound =
      Eigen::Vector2d::Map(params_io.com_vel_bound.data());
  params_out.Q = Eigen::Map <
      Eigen::Matrix < double, 4, 4, Eigen::RowMajor >> (params_io.Q.data());
  params_out.Qf = Eigen::Map <
      Eigen::Matrix < double, 4, 4, Eigen::RowMajor >> (params_io.Qf.data());
  params_out.R = Eigen::Map <
      Eigen::Matrix < double, 3, 3, Eigen::RowMajor >> (params_io.R.data());
  params_out.solver_options = solver_options_io.GetAsSolverOptions(
      drake::solvers::GurobiSolver::id()
  );
  return params_out;
}

}