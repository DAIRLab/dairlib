#pragma once

#include <optional>
#include <stdexcept>

#include <Eigen/Dense>

#include "examples/sampling_c3/parameter_headers/object_state_error_params.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct RobotSimParams {
  std::vector<std::string> object_models;
  double dt;
  double realtime_rate;
  /// Dead time, in seconds, between a command being received and the robot
  /// beginning to respond to it.
  double actuator_delay;
  /// Optional first-order time constant, in seconds, with which the robot eases
  /// into a command once actuator_delay has elapsed.  Only the 3D printer sim
  /// reads this; omit it to model the command path as a pure delay.
  std::optional<double> command_time_constant;
  double robot_publish_rate;
  double object_publish_rate;
  bool visualize_drake_sim;
  bool publish_efforts;
  Eigen::VectorXd q_init_robot;
  std::vector<Eigen::VectorXd> q_init_objects;

  /// If true, the simulator publishes the clean object state on the lcm
  /// channels' clean_object_state_channels and a version corrupted by
  /// estimation errors on the usual object_state_channels, so the controller
  /// sees hardware-like pose estimates without any change on its side.
  bool inject_object_state_errors;
  /// Required (and only read) when inject_object_state_errors is true.
  std::optional<ObjectStateErrorParams> object_state_error_params;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(object_models));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(command_time_constant));
    a->Visit(DRAKE_NVP(robot_publish_rate));
    a->Visit(DRAKE_NVP(object_publish_rate));
    a->Visit(DRAKE_NVP(visualize_drake_sim));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(q_init_robot));
    a->Visit(DRAKE_NVP(q_init_objects));
    a->Visit(DRAKE_NVP(inject_object_state_errors));
    a->Visit(DRAKE_NVP(object_state_error_params));

    if (inject_object_state_errors && !object_state_error_params.has_value()) {
      throw std::runtime_error(
          "inject_object_state_errors is true but object_state_error_params is "
          "not set.");
    }
  }
};
