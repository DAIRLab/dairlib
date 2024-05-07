#pragma once

#include <assert.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "examples/franka_ball_rolling/parameters/impedance_controller_params.h"
#include "multibody/geom_geom_collider.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RotationMatrix;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::State;

using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {
namespace controllers {

class ImpedanceController : public LeafSystem<double> {
 public:
  ImpedanceController(const drake::multibody::MultibodyPlant<double>& plant,
                      drake::systems::Context<double>& context,
                      const Eigen::MatrixXd& K, const Eigen::MatrixXd& B,
                      const Eigen::MatrixXd& K_null,
                      const Eigen::MatrixXd& B_null,
                      const Eigen::VectorXd& qd_null,
                      bool gravity_compensation_flag);

  const drake::systems::InputPort<double>& get_input_port_franka_state() const {
    return this->get_input_port(franka_state_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_c3_command() const {
    return this->get_input_port(planner_state_input_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_torque() const {
    return this->get_output_port(control_output_port_);
  }

 private:
  // computes the control input
  void CalcControl(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* output) const;
  // updates the integral term
  drake::systems::EventStatus UpdateIntegralTerm(
      const Context<double>& context,
      drake::systems::State<double>* drake_state) const;

  // computes the rotational error
  Eigen::Vector3d CalcRotationalError(
      const drake::math::RotationMatrix<double>& R,
      const Quaterniond& orientation_d) const;
  void ClampJointTorques(VectorXd& tau) const;
  void ClampIntegratorTorque(VectorXd& tau, const VectorXd& clamp) const;
  bool SaturatedClamp(const VectorXd& tau, const VectorXd& clamp) const;

  // parameters
  ImpedanceControllerParams impedance_param_;

  // ports
  int franka_state_input_port_;
  int planner_state_input_port_;
  int contact_feedforward_input_port_;

  int control_output_port_;

  // constructor variables
  // plant and context
  const MultibodyPlant<double>& plant_;
  drake::systems::Context<double>& context_;

  // stiffness and damping
  const Eigen::MatrixXd K_;
  const Eigen::MatrixXd B_;

  // stiffness and damping (null space)
  const MatrixXd K_null_;
  const MatrixXd B_null_;
  const VectorXd qd_null_;

  // whether or not to do gravity compensation (since Franka itself has internal
  // gravity compensation)
  const bool gravity_compensation_flag_;

  // integral control and time recording settings
  int prev_time_;  // this is the index for prev_time (Double type abstract
                   // state)
  int enable_integral_;
  int integrator_;  // this is the index for integrator (6D drake state)
  Eigen::MatrixXd I_;

  // contact force feedforward settings
  int enable_contact_;

  // clamp torque limit settings
  Eigen::VectorXd torque_limits_;

  // frame, joint numbers and kinematics settings
  const drake::multibody::BodyFrame<double>* EE_frame_;
  const drake::multibody::BodyFrame<double>* world_frame_;
  Eigen::Vector3d EE_offset_;
  int n_;  // franka DoF (would be 7D, derived from num_positions)

  // final control output, used for being accessed by context in CalControl
  int tau_;  // final control torque (would be 7D drake state)
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib