#pragma once

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

struct MagnaSimParams {
  double dt;
  double realtime_rate;
  double actuator_delay;
  double franka_publish_rate;
  double object_publish_rate;
  bool publish_efforts;
  Eigen::VectorXd q_init_franka;
  Eigen::VectorXd q_init_franka_hand;
  std::vector<Eigen::VectorXd> init_belt_vertices;
  Eigen::Matrix3Xd q_init_belt;
  std::vector<Eigen::VectorXd> init_timing_belt_points;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(franka_publish_rate));
    a->Visit(DRAKE_NVP(object_publish_rate));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_franka_hand));
    a->Visit(DRAKE_NVP(init_belt_vertices));
    a->Visit(DRAKE_NVP(init_timing_belt_points));
    ComputeBeltVerticesMatrix();
  }

  void ComputeBeltVerticesMatrix() {
    q_init_belt = Eigen::Matrix3Xd::Zero(3, init_belt_vertices.size());
    for (int i = 0; i < init_belt_vertices.size(); ++i) {
      q_init_belt.col(i) = init_belt_vertices[i];
    }
  }
};
