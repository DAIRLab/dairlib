#pragma once
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct KinematicCentroidalGains {
  Eigen::Vector3d com_position;
  std::unordered_map<std::string, double> generalized_positions;
  std::unordered_map<std::string, double> generalized_velocities;
  Eigen::Vector3d contact_pos;
  Eigen::Vector3d contact_vel;
  Eigen::Vector3d contact_force;
  Eigen::Vector3d lin_momentum;
  Eigen::Vector3d ang_momentum;

  double tol;
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(com_position));
    a->Visit(DRAKE_NVP(generalized_positions));
    a->Visit(DRAKE_NVP(generalized_velocities));
    a->Visit(DRAKE_NVP(contact_pos));
    a->Visit(DRAKE_NVP(contact_vel));
    a->Visit(DRAKE_NVP(contact_force));
    a->Visit(DRAKE_NVP(lin_momentum));
    a->Visit(DRAKE_NVP(ang_momentum));
    a->Visit(DRAKE_NVP(tol));
  }
};