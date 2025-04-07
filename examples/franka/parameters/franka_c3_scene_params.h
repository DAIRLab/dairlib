#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct FrankaC3SceneParams {
  std::string franka_model;
  std::string end_effector_model;
  std::string end_effector_name;
  Eigen::Vector3d tool_attachment_frame;
  double end_effector_thickness;
  std::string end_effector_lcs_model;
  std::vector<std::string> object_models;
  std::vector<std::string> environment_models;
  std::vector<Eigen::VectorXd> environment_orientations;
  std::vector<Eigen::VectorXd> environment_positions;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(end_effector_thickness));
    a->Visit(DRAKE_NVP(end_effector_lcs_model));
    a->Visit(DRAKE_NVP(object_models));
    a->Visit(DRAKE_NVP(environment_models));
    a->Visit(DRAKE_NVP(environment_orientations));
    a->Visit(DRAKE_NVP(environment_positions));
  }
};