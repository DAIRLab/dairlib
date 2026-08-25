#pragma once

#include <string>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/state_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "systems/franka_kinematics_vector.h"

#include "drake/common/trajectories/trajectory.h"
#include "drake/systems/framework/leaf_system.h"

using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::InputPort;
using drake::systems::InputPortIndex;
using drake::systems::OutputPort;
using drake::systems::OutputPortIndex;
using Eigen::VectorXd;

namespace dairlib {

using systems::OutputVector;
using systems::StateVector;
using systems::TimestampedVector;

namespace systems {

/// Outputs a lcmt_timestamped_saved_traj
class ThreeDPrinterKinematics : public drake::systems::LeafSystem<double> {
 public:
  explicit ThreeDPrinterKinematics(const MultibodyPlant<double>& printer_plant,
                                   Context<double>* printer_context,
                                   const MultibodyPlant<double>& object_plant,
                                   Context<double>* object_context,
                                   const std::string& end_effector_name,
                                   const std::string& object_name);

  explicit ThreeDPrinterKinematics(const MultibodyPlant<double>& printer_plant,
                                   Context<double>* printer_context,
                                   const MultibodyPlant<double>& object_plant,
                                   Context<double>* object_context,
                                   const std::string& end_effector_name,
                                   std::vector<std::string> object_names);

  std::vector<const drake::systems::InputPort<double>*>
  get_input_ports_object_state() const {
    std::vector<const InputPort<double>*> output;
    for (InputPortIndex port : object_state_ports_) {
      output.push_back(&this->get_input_port(port));
    }
    return output;
  }
  const InputPort<double>& get_input_port_printer_state() const {
    return this->get_input_port(printer_state_port_);
  }

  const OutputPort<double>& get_output_port_lcs_state() const {
    return this->get_output_port(lcs_state_port_);
  }

 private:
  void ComputeLCSState(const drake::systems::Context<double>& context,
                       FrankaKinematicsVector<double>* output_traj) const;

  InputPortIndex printer_state_port_;
  std::vector<InputPortIndex> object_state_ports_;
  OutputPortIndex lcs_state_port_;

  const int num_end_effector_positions_ = 3;
  int num_object_positions_;
  const int num_end_effector_velocities_ = 3;
  int num_object_velocities_;
  std::vector<std::string> object_names_;
  int num_objects_;

  const MultibodyPlant<double>& printer_plant_;
  Context<double>* printer_context_;
  const MultibodyPlant<double>& object_plant_;
  Context<double>* object_context_;
  std::string end_effector_name_;
};

/// Converts a desired end effector (tip) trajectory, expressed in world frame,
/// into the corresponding printer joint (gantry carriage) position trajectory
/// -- the inverse of what ThreeDPrinterKinematics computes.
class ThreeDPrinterInverseKinematics
    : public drake::systems::LeafSystem<double> {
 public:
  ThreeDPrinterInverseKinematics(const MultibodyPlant<double>& printer_plant,
                                 Context<double>* printer_context,
                                 const std::string& end_effector_name);

  const InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_port_);
  }
  const OutputPort<double>& get_output_port_trajectory() const {
    return this->get_output_port(joint_trajectory_port_);
  }

 private:
  void CalcJointTrajectory(const drake::systems::Context<double>& context,
                           drake::trajectories::Trajectory<double>* traj) const;

  InputPortIndex trajectory_port_;
  OutputPortIndex joint_trajectory_port_;

  // Position of the end effector tip when all printer joints are at 0 -- the
  // fixed offset between joint (carriage) space and tip world position.
  Eigen::Vector3d end_effector_offset_;
};

}  // namespace systems
}  // namespace dairlib
