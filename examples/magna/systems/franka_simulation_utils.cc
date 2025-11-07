#include "examples/magna/systems/franka_simulation_utils.h"

#include <Eigen/Dense>
#include <drake/lcmt_schunk_wsg_command.hpp>

#include "common/find_resource.h"
#include "multibody/multibody_utils.h"
#include "solvers/solver_options_io.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/system_utils.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solver_options.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

using dairlib::multibody::SetPositionsAndVelocitiesIfNew;
using dairlib::systems::OutputVector;
using dairlib::systems::TimestampedVector;
using dairlib::systems::controllers::JointSpaceTrackingData;
using dairlib::systems::controllers::OperationalSpaceControl;

using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::OutputPort;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {

ShunkCommandToTrajectory::ShunkCommandToTrajectory() {
  drake::lcmt_schunk_wsg_command default_command;
  default_command.utime = 0;
  default_command.target_position_mm = 0.0;
  default_command.force = 0.0;
  input_schunk_command_port_ =
      this->DeclareAbstractInputPort(
              "schunk_command",
              drake::Value<drake::lcmt_schunk_wsg_command>{default_command})
          .get_index();
  PiecewisePolynomial<double> pp(VectorXd(0));
  Trajectory<double>& default_instantiation = pp;
  position_trajectory_port_ =
      this->DeclareAbstractOutputPort("position_trajectory",
                                      default_instantiation,
                                      &ShunkCommandToTrajectory::ToTrajectory)
          .get_index();
}

void ShunkCommandToTrajectory::ToTrajectory(const Context<double>& context,
                                            Trajectory<double>* output) const {
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          output);
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& schunk_command =
      input->get_value<drake::lcmt_schunk_wsg_command>();
  MatrixXd finger_offsets(2, 1);
  // Convert from mm to m and from gripper width to finger offsets
  finger_offsets << schunk_command.target_position_mm / 2000.0,
      schunk_command.target_position_mm / 2000.0;
  *casted_traj = PiecewisePolynomial<double>(finger_offsets);
}

StatusToRobotOutput::StatusToRobotOutput(
    const MultibodyPlant<double>& plant,
    drake::multibody::ModelInstanceIndex model_instance) {
  input_position_port_ =
      this->DeclareVectorInputPort("positions", 2).get_index();
  input_velocity_port_ =
      this->DeclareVectorInputPort("velocities", 2).get_index();
  input_effort_port_ = this->DeclareVectorInputPort("efforts", 2).get_index();

  output_robot_output_port_ =
      this->DeclareVectorOutputPort(
              "x, u, t",
              OutputVector<double>(plant.num_positions(model_instance),
                                   plant.num_velocities(model_instance),
                                   plant.num_actuators()),
              &StatusToRobotOutput::ToRobotOutput)
          .get_index();
}

void StatusToRobotOutput::ToRobotOutput(const Context<double>& context,
                                        OutputVector<double>* output) const {
  const auto& position =
      this->EvalVectorInput(context, input_position_port_)->get_value();
  const auto& velocity =
      this->EvalVectorInput(context, input_velocity_port_)->get_value();
  const auto& effort =
      this->EvalVectorInput(context, input_effort_port_)->get_value();

  output->SetPositions(position);
  output->SetVelocities(velocity);
  output->SetEfforts(effort);
  output->SetIMUAccelerations(Vector3d::Zero());
  output->set_timestamp(context.get_time());
}

GravityCompensator::GravityCompensator(const MultibodyPlant<double>& plant,
                                       Context<double>& context)
    : plant_(plant), context_(context) {
  num_actuators_ = plant_.num_actuators();
  state_port_ = this->DeclareVectorInputPort(
                        "x", plant_.num_positions() + plant_.num_velocities())
                    .get_index();
  actuation_port_ =
      this->DeclareVectorInputPort("u", num_actuators_).get_index();
  compensated_actuation_port_ =
      this->DeclareVectorOutputPort("u_compensated", num_actuators_,
                                    &GravityCompensator::AddGravityCompensation)
          .get_index();
}

void GravityCompensator::AddGravityCompensation(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& state = this->EvalVectorInput(context, state_port_)->get_value();
  // Set the plant context to the current state
  SetPositionsAndVelocitiesIfNew<double>(plant_, state, &context_);
  const auto& tau =
      this->EvalVectorInput(context, actuation_port_)->get_value();
  VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);
  for (int i = 0; i < num_actuators_; i++) {
    (*output)[i] = tau[i] - tau_g[i];
  }
}

const OutputPort<double>& SimulatePandaHand(
    DiagramBuilder<double>* builder,
    const drake::multibody::MultibodyPlant<double>& hand_mbplant,
    drake::systems::Context<double>* hand_mbplant_context,
    drake::lcm::DrakeLcmInterface* lcm,
    std::string gripper_command_channel,
    const drake::systems::OutputPort<double>& gripper_state_input_port,
    std::string osc_qp_settings_file) {
  auto command_subscriber = builder->AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<
          drake::lcmt_schunk_wsg_command>(gripper_command_channel, lcm));

  auto schunk_cmd_to_trajectory =
      builder->AddSystem<ShunkCommandToTrajectory>();
  builder->Connect(command_subscriber->get_output_port(),
                   schunk_cmd_to_trajectory->get_input_port_schunk_command());

  auto hand_osc_controller = builder->AddSystem<OperationalSpaceControl>(
      hand_mbplant, hand_mbplant, hand_mbplant_context, hand_mbplant_context,
      false);

  // Track joint positions to maintain them as much as possible
  auto joint_position_tracking_data = std::make_unique<JointSpaceTrackingData>(
      "joint_position_target", Eigen::MatrixXd::Identity(2, 2) * 100,
      Eigen::MatrixXd::Identity(2, 2) * 40, Eigen::MatrixXd::Identity(2, 2) * 1,
      hand_mbplant, hand_mbplant);
  // Maintain joint positions as much as possible
  std::vector<std::string> joint_position_names = {"panda_finger_joint1",
                                                   "panda_finger_joint2"};
  std::vector<std::string> joint_velocity_names = {"panda_finger_joint1dot",
                                                   "panda_finger_joint2dot"};
  joint_position_tracking_data->AddJointsToTrack(joint_position_names,
                                                 joint_velocity_names);
  hand_osc_controller->AddTrackingData(std::move(joint_position_tracking_data));
  // Set other OSC parameters
  hand_osc_controller->SetAccelerationCostWeights(
      Eigen::MatrixXd::Identity(2, 2) * 1e-7);
  hand_osc_controller->SetInputCostWeights(
      Eigen::MatrixXd::Zero(2, 2));  // No input cost
  hand_osc_controller->SetInputSmoothingCostWeights(
      Eigen::MatrixXd::Zero(2, 2));  // No input smoothing cost
  hand_osc_controller->SetAccelerationConstraints(false);
  hand_osc_controller->SetContactFriction(0.4615);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<dairlib::solvers::SolverOptionsFromYaml>(
          dairlib::FindResourceOrThrow(osc_qp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());
  hand_osc_controller->SetOsqpSolverOptions(solver_options);
  hand_osc_controller->Build();

  // Connect Input and Outputs
  builder->Connect(gripper_state_input_port,
                   hand_osc_controller->get_input_port_robot_output());

  builder->Connect(
      schunk_cmd_to_trajectory->get_output_port_position_trajectory(),
      hand_osc_controller->get_input_port_tracking_data(
          "joint_position_target"));

  return (hand_osc_controller->get_output_port_osc_command());
}

}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib