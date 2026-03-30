#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct TrifingerControllerParams {

  std::string trifinger_model;
  std::string object_model;
  std::string end_effector_model;
  std::string end_effector_lcs_model;
  std::vector<std::string> end_effector_names;
  std::vector<std::string> ee_base_link_names;
  std::string ground_model;

  std::string osqp_settings_file;
  std::string ic3_options_file;
  std::string c3_controller_options_file;


  bool track_dynamically_feasible;

  double time_to_wait;

  Eigen::Vector3d tool_attachment_frame;
  Eigen::VectorXd x_target;
  Eigen::VectorXd nominal_position;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(trifinger_model));
    a->Visit(DRAKE_NVP(object_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(end_effector_lcs_model));
    a->Visit(DRAKE_NVP(end_effector_names));
    a->Visit(DRAKE_NVP(ee_base_link_names));
    a->Visit(DRAKE_NVP(ground_model));

    a->Visit(DRAKE_NVP(osqp_settings_file));
    a->Visit(DRAKE_NVP(ic3_options_file));
    a->Visit(DRAKE_NVP(c3_controller_options_file));
    a->Visit(DRAKE_NVP(time_to_wait));

    a->Visit(DRAKE_NVP(track_dynamically_feasible));
    a->Visit(DRAKE_NVP(nominal_position));

    a->Visit(DRAKE_NVP(x_target));

  }
};