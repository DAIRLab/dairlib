#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct TrifingerSimParams {
  std::string trifinger_model;
  std::string cube_model;

  double dt;
  double realtime_rate;
  double trifinger_publish_rate;
  bool publish_efforts;
  double actuator_delay;
  double visualizer_publish_rate;

  std::string contact_solver;

  Eigen::VectorXd q_init_trifinger;
  Eigen::VectorXd q_init_cube;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(trifinger_model));
    a->Visit(DRAKE_NVP(cube_model));

    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(trifinger_publish_rate));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(contact_solver));
    a->Visit(DRAKE_NVP(visualizer_publish_rate));

    a->Visit(DRAKE_NVP(q_init_trifinger));
    a->Visit(DRAKE_NVP(q_init_cube));
  }
};
