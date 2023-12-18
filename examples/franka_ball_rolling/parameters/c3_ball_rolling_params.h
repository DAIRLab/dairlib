#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

struct C3BallRollingParam {
  std::string franka_model;
  std::string end_effector_model;
  std::string ball_model;
  std::string ground_model;
  std::string offset_model;

  Vector3d tool_attachment_frame;
  Vector3d ground_offset_frame;
  Vector3d end_effector_offset;

  Vector3d translational_stiffness;
  Vector3d rotational_stiffness;
  Vector3d translational_damping;
  Vector3d rotational_damping;

  VectorXd stiffness_null;
  VectorXd damping_null;
  VectorXd q_null_desired;

  int enable_integral;
  Vector3d translational_integral_gain;
  Vector3d rotational_integral_gain;
  VectorXd integrator_clamp;
  VectorXd torque_limits;

  int enable_contact;
  double contact_threshold;

  VectorXd q_init_franka;
  VectorXd initial_start;
  VectorXd q_init_ball;

  double traj_radius;
  double phase;
  double x_c;
  double y_c;
  double ball_radius;

  int num_friction_directions;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(ball_model));
    a->Visit(DRAKE_NVP(ground_model));
    a->Visit(DRAKE_NVP(offset_model));

    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(ground_offset_frame));
    a->Visit(DRAKE_NVP(end_effector_offset));

    a->Visit(DRAKE_NVP(translational_stiffness));
    a->Visit(DRAKE_NVP(rotational_stiffness));
    a->Visit(DRAKE_NVP(translational_damping));
    a->Visit(DRAKE_NVP(rotational_damping));

    a->Visit(DRAKE_NVP(stiffness_null));
    a->Visit(DRAKE_NVP(damping_null));
    a->Visit(DRAKE_NVP(q_null_desired));

    a->Visit(DRAKE_NVP(enable_integral));
    a->Visit(DRAKE_NVP(translational_integral_gain));
    a->Visit(DRAKE_NVP(rotational_integral_gain));
    a->Visit(DRAKE_NVP(integrator_clamp));
    a->Visit(DRAKE_NVP(torque_limits));

    a->Visit(DRAKE_NVP(enable_contact));
    a->Visit(DRAKE_NVP(contact_threshold));

    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(initial_start));
    a->Visit(DRAKE_NVP(q_init_ball));

    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(phase));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(ball_radius));

    a->Visit(DRAKE_NVP(num_friction_directions));
  }
};