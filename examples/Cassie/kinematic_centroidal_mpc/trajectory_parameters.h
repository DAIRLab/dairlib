#pragma once

#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct TrajectoryParameters {
  double n_knot_points;
  double com_height;
  double stance_width;
  double vel;
  double tol;
  std::vector<double> duration_scaling;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(n_knot_points));
    a->Visit(DRAKE_NVP(com_height));
    a->Visit(DRAKE_NVP(stance_width));
    a->Visit(DRAKE_NVP(vel));
    a->Visit(DRAKE_NVP(tol));
    a->Visit(DRAKE_NVP(duration_scaling));
  }
};