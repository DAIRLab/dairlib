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
  /// (Cartesian) Impedance Controller Class, given desired end-effector state
  /// (task space state) and (potential) desired force, calculate the joint
  /// toque for the robot
  /// @param plant The standard <double> MultibodyPlant which includes the robot
  /// @param context The context (double)
  /// @param K The stiffness matrix for the task space target
  /// @param B The damping matrix for the task space target
  /// @param K_null The stiffness matrix for null space target
  /// @param B_null The damping matrix for for null space target
  /// @param qd_null The desired null space target (TODO: maybe should be made
  /// as an external input?)
  /// @param gravity_compensation_flag Whether or not to apply gravity
  /// compensation, exist since franka automatically do gravity compensation

  ImpedanceController(const drake::multibody::MultibodyPlant<double>& plant,
                      drake::systems::Context<double>& context,
                      const Eigen::MatrixXd& K, const Eigen::MatrixXd& B,
                      const Eigen::MatrixXd& K_null,
                      const Eigen::MatrixXd& B_null,
                      const Eigen::VectorXd& qd_null,
                      bool gravity_compensation_flag);

  /// the first input port take in franka state (and torque)
  const drake::systems::InputPort<double>& get_input_port_franka_state() const {
    return this->get_input_port(franka_state_input_port_);
  }

  /// the second input port take in control commanded from high level planner
  /// (desired task space target and potential feedforward terms)
  const drake::systems::InputPort<double>& get_input_port_c3_command() const {
    return this->get_input_port(planner_state_input_port_);
  }

  /// the outport send out the joint torque for the robot
  const drake::systems::OutputPort<double>& get_output_port_torque() const {
    return this->get_output_port(control_output_port_);
  }

 private:
  /// Output the calculated torque for the robot, the actual calculation process
  /// is mainly done together with other calculation in UpdateControl Event
  /// Update function
  /// @param context The context (double)
  void CalcControl(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* output) const;

  /// Update the integral term recorded as drake state and do the main
  /// calculation for impedance control torque
  /// @param context The context (double)
  drake::systems::EventStatus UpdateControl(
      const Context<double>& context,
      drake::systems::State<double>* drake_state) const;

  /// Computes the rotational error, grabbed from libfranka source code
  /// @param R The current orientation parameterized by 3*3 rotation matrix
  /// @param orientation_d The desired orientation parameterized by quaternion
  Eigen::Vector3d CalcRotationalError(
      const drake::math::RotationMatrix<double>& R,
      const Quaterniond& orientation_d) const;

  /// Clamp the final calculated torque into safety margin
  /// @param tau the calculated joint torque
  void ClampJointTorques(VectorXd& tau) const;

  /// Do integral windup for the integral term
  /// @param tau the calculated joint torque
  void ClampIntegratorTorque(VectorXd& tau) const;

  /// Judge whether the integral term is saturated
  /// @param tau the calculated joint torque
  bool SaturatedClamp(const VectorXd& tau) const;

  /// Impedacne parameters
  ImpedanceControllerParams impedance_param_;

  /// Input and output ports index
  int franka_state_input_port_;
  int planner_state_input_port_;
  int contact_feedforward_input_port_;

  int control_output_port_;

  /// Constructor variables
  // plant and context
  const MultibodyPlant<double>& plant_;
  drake::systems::Context<double>& context_;

  /// Stiffness and damping for task space (end-effector)
  const Eigen::MatrixXd K_;
  const Eigen::MatrixXd B_;

  /// Stiffness and damping for null space
  const MatrixXd K_null_;
  const MatrixXd B_null_;
  const VectorXd qd_null_;

  /// Whether or not to do gravity compensation (since Franka itself has
  /// internal gravity compensation)
  const bool gravity_compensation_flag_;

  /// integral control and time recording settings
  drake::systems::AbstractStateIndex prev_time_;  // Index for prev_time (Double type abstract state)
  int enable_integral_;
  drake::systems::DiscreteStateIndex integrator_;  // Index for integrator (6D drake state)
  Eigen::MatrixXd I_;

  /// contact force feedforward settings
  int enable_contact_;

  /// clamp torque limit settings
  Eigen::VectorXd torque_limits_;
  Eigen::VectorXd integral_limits_;

  /// frame, joint numbers and kinematics settings
  const drake::multibody::BodyFrame<double>* EE_frame_;
  const drake::multibody::BodyFrame<double>* world_frame_;
  int n_;  // franka DoF (would be 7D, derived from num_positions)

  /// final control output, used for being accessed by context in CalControl
  drake::systems::DiscreteStateIndex tau_;  // final control torque (would be 7D drake state)
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib