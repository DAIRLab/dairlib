#pragma once

#include "solvers/c3_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct FrankaPlateC3ControllerParams {

  std::string allegro_model;
  std::string object_model;

  std::string osqp_settings_file;
  std::string ic3_options_file;
  std::string c3_options_file;

  bool run_open_loop;
  bool track_dynamically_feasible;

  double time_to_wait;

  Eigen::VectorXd x_target;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(allegro_model));
    a->Visit(DRAKE_NVP(object_model));

    a->Visit(DRAKE_NVP(osqp_settings_file));
    a->Visit(DRAKE_NVP(ic3_options_file));
    a->Visit(DRAKE_NVP(c3_options_file));

    a->Visit(DRAKE_NVP(run_open_loop));
    a->Visit(DRAKE_NVP(track_dynamically_feasible));

    a->Visit(DRAKE_NVP(time_to_wait));

    a->Visit(DRAKE_NVP(x_target));

  }
};