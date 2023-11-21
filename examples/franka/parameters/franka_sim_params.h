#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct FrankaSimParams {
  std::string franka_model;
  std::string end_effector_model;
  std::string table_model;
  std::string table_w_supports_model;
  std::string tray_model;
  std::string box_model;

  double dt;
  double realtime_rate;
  double actuator_delay;
  double franka_publish_rate;
  double tray_publish_rate;

  bool visualize;
  bool publish_efforts;

  std::vector<double> q_init_franka;
  std::vector<double> q_init_plate;
  std::vector<double> q_init_box;
  std::vector<double> tool_attachment_frame;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(table_model));
    a->Visit(DRAKE_NVP(table_w_supports_model));
    a->Visit(DRAKE_NVP(tray_model));
    a->Visit(DRAKE_NVP(box_model));

    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(franka_publish_rate));
    a->Visit(DRAKE_NVP(tray_publish_rate));

    a->Visit(DRAKE_NVP(visualize));
    a->Visit(DRAKE_NVP(publish_efforts));

    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_plate));
    a->Visit(DRAKE_NVP(q_init_box));
    a->Visit(DRAKE_NVP(tool_attachment_frame));

  }
};