#pragma once

#include <vector>
#include <utility>
#include <chrono>
#include <assert.h>
#include <iostream>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "multibody/geom_geom_collider.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/math/rigid_transform.h>
#include "multibody/multibody_utils.h"
#include "multibody/geom_geom_collider.h"
#include "examples/franka_ball_rolling/c3_parameters.h"
#include "examples/franka_ball_rolling/parameters/impedance_controller_params.h"
#include "yaml-cpp/yaml.h"
#include "drake/common/yaml/yaml_io.h"



using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::State;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RotationMatrix;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Quaterniond;

namespace dairlib {
namespace systems {
namespace controllers {

class ImpedanceController : public LeafSystem<double> {
 public:
  ImpedanceController(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::MultibodyPlant<double>& plant_contact,
      drake::systems::Context<double>& context,
      drake::systems::Context<double>& context_contact,
      const Eigen::MatrixXd& K,
      const Eigen::MatrixXd& B,
      const Eigen::MatrixXd& K_null,
      const Eigen::MatrixXd& B_null,
      const Eigen::VectorXd& qd_null,
      const std::vector<drake::geometry::GeometryId>& contact_geoms,
      int num_friction_directions);

  const drake::systems::InputPort<double>& get_input_port_config() const {
    return this->get_input_port(franka_state_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_c3() const {
    return this->get_input_port(c3_state_input_port_);
  }

  const drake::systems::OutputPort<double>& get_input_port_output() const {
    return this->get_output_port(control_output_port_);
  }

 private:
  // computes the control input
  void CalcControl(const drake::systems::Context<double>& context,
                    TimestampedVector<double>* output) const;
  // updates the integral term
  drake::systems::EventStatus UpdateIntegralTerm(const Context<double>& context,
          drake::systems::State<double>* drake_state) const;

  // computes the rotational error
  Eigen::Vector3d CalcRotationalError(const drake::math::RotationMatrix<double>& R,
    const Quaterniond& orientation_d) const;
  // computes the contact jacobians in J_n and J_t
  void CalcContactJacobians(const std::vector<SortedPair<GeometryId>>& contact_pairs,
                    VectorXd& phi, MatrixXd& J_n, MatrixXd& J_t) const;
  void ClampJointTorques(VectorXd &tau) const;
  void ClampIntegratorTorque(VectorXd& tau, const VectorXd& clamp) const;
  bool SaturatedClamp(const VectorXd& tau, const VectorXd& clamp) const;

  // parameters
  ImpedanceControllerParams impedance_param_;

  // ports
  int franka_state_input_port_;
  int c3_state_input_port_;
  int control_output_port_;
  
  // constructor variables
  // plant and context
  const MultibodyPlant<double>& plant_;
  const MultibodyPlant<double>& plant_contact_;
  drake::systems::Context<double>& context_;
  drake::systems::Context<double>& context_contact_;

  // stiffness and damping
  const Eigen::MatrixXd K_;
  const Eigen::MatrixXd B_;

  // stiffness and damping (null space)
  const MatrixXd K_null_;
  const MatrixXd B_null_;
  const VectorXd qd_null_;

  // integral control and time recording settings
  int prev_time_; // this is the index for prev_time (Double type abstract state)
  int enable_integral_;
  int integrator_; // this is the index for integrator (6D drake state)
  Eigen::MatrixXd I_;

  // contact force feedforward settings
  int enable_contact_;
  const int num_friction_directions_;
  std::vector<drake::geometry::GeometryId> contact_geoms_;
  std::vector<SortedPair<GeometryId>> contact_pairs_;

  // clamp torque limit settings
  Eigen::VectorXd torque_limits_;

  // frame, joint numbers and kinematics settings
  const drake::multibody::BodyFrame<double>* EE_frame_;
  const drake::multibody::BodyFrame<double>* world_frame_;
  Eigen::Vector3d EE_offset_;
  int n_; // franka DoF = 7

  // final control output, used for being accessed by context in CalControl
  int tau_; // final control torque (7D drake state)
};

}  // namespace controller
}  // namespace systems
}  // namespace dairlib