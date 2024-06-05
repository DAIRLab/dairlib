#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

struct ImpedanceControllerParams {
  std::string franka_model;
  std::string end_effector_model;

  Vector3d tool_attachment_frame;

  std::vector<double> translational_stiffness;
  std::vector<double> rotational_stiffness;
  std::vector<double> translational_damping;
  std::vector<double> rotational_damping;
  std::vector<double> stiffness_list;
  std::vector<double> damping_list;
  VectorXd stiffness_vector;
  VectorXd damping_vector;
  MatrixXd K;
  MatrixXd B;

  VectorXd stiffness_null;
  VectorXd damping_null;
  MatrixXd K_null;
  MatrixXd B_null;
  VectorXd q_null_desired;

  bool gravity_compensation_flag;

  int enable_integral;
  Vector3d translational_integral_gain;
  Vector3d rotational_integral_gain;
  VectorXd integrator_clamp;
  VectorXd torque_limits;

  int enable_contact;
  double contact_threshold;

  VectorXd q_init_franka;
  VectorXd initial_start;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));

    a->Visit(DRAKE_NVP(tool_attachment_frame));

    a->Visit(DRAKE_NVP(translational_stiffness));
    a->Visit(DRAKE_NVP(rotational_stiffness));
    a->Visit(DRAKE_NVP(translational_damping));
    a->Visit(DRAKE_NVP(rotational_damping));

    a->Visit(DRAKE_NVP(stiffness_null));
    a->Visit(DRAKE_NVP(damping_null));
    a->Visit(DRAKE_NVP(q_null_desired));

    a->Visit(DRAKE_NVP(gravity_compensation_flag));

    a->Visit(DRAKE_NVP(enable_integral));
    a->Visit(DRAKE_NVP(translational_integral_gain));
    a->Visit(DRAKE_NVP(rotational_integral_gain));
    a->Visit(DRAKE_NVP(integrator_clamp));
    a->Visit(DRAKE_NVP(torque_limits));

    a->Visit(DRAKE_NVP(enable_contact));
    a->Visit(DRAKE_NVP(contact_threshold));

    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(initial_start));

    stiffness_list = std::vector<double>();
    stiffness_list.insert(stiffness_list.end(), rotational_stiffness.begin(),
                          rotational_stiffness.end());
    stiffness_list.insert(stiffness_list.end(), translational_stiffness.begin(),
                          translational_stiffness.end());
    stiffness_vector = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->stiffness_list.data(), this->stiffness_list.size());
    K = stiffness_vector.asDiagonal();

    damping_list = std::vector<double>();
    damping_list.insert(damping_list.end(), rotational_damping.begin(),
                        rotational_damping.end());
    damping_list.insert(damping_list.end(), translational_damping.begin(),
                        translational_damping.end());
    damping_vector = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->damping_list.data(), this->damping_list.size());
    B = damping_vector.asDiagonal();

    K_null = stiffness_null.asDiagonal();
    B_null = damping_null.asDiagonal();
  }
};