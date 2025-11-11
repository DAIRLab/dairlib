#pragma once

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

struct ElasticSimParams {
  std::string object_mesh;
  double object_mesh_scale;
  double robot_publish_rate;
  double object_publish_rate;
  double actuator_delay;
  bool visualize_drake_sim;
  bool publish_efforts;
  double dt;
  double realtime_rate;
  Eigen::VectorXd q_init_robot;
  Eigen::VectorXd q_init_object;
  bool include_spring_damper_model;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(object_mesh));
    a->Visit(DRAKE_NVP(object_mesh_scale));
    a->Visit(DRAKE_NVP(robot_publish_rate));
    a->Visit(DRAKE_NVP(object_publish_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(visualize_drake_sim));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(q_init_robot));
    a->Visit(DRAKE_NVP(q_init_object));
    a->Visit(DRAKE_NVP(include_spring_damper_model));
  }
};
