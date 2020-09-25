#include "examples/Cassie/osc/vdot_integrator.h"

#include <algorithm>  // std::max
#include "multibody/multibody_utils.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using dairlib::systems::TimestampedVector;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::LeafSystem;

namespace dairlib {
namespace cassie {
namespace osc {

VdotIntegrator::VdotIntegrator(
    const drake::multibody::MultibodyPlant<double>& plant_w_spr,
    const drake::multibody::MultibodyPlant<double>& plant_wo_spr) {
  nq_spr_ = plant_w_spr.num_positions();
  nv_spr_ = plant_w_spr.num_velocities();
  nx_spr_ = plant_w_spr.num_positions() + plant_w_spr.num_velocities();

  // Input/Output Setup
  vdot_port_ = this->DeclareVectorInputPort(systems::TimestampedVector<double>(
                                                plant_wo_spr.num_velocities()))
                   .get_index();
  feddback_state_port_ = this
                             ->DeclareVectorInputPort(OutputVector<double>(
                                 nq_spr_, nv_spr_, plant_w_spr.num_actuators()))
                             .get_index();
  this->DeclareVectorOutputPort(
      systems::TimestampedVector<double>(plant_w_spr.num_positions() +
                                         plant_w_spr.num_velocities()),
      &VdotIntegrator::CopyState);

  // Per-step update to integrate dynamics
  DeclarePerStepDiscreteUpdateEvent(&VdotIntegrator::DiscreteVariableUpdate);
  // The start time of the current fsm state
  prev_time_idx_ = this->DeclareDiscreteState(1);
  actuated_q_idx_ = this->DeclareDiscreteState(plant_w_spr.num_actuators());
  actuated_v_idx_ = this->DeclareDiscreteState(plant_w_spr.num_actuators());

  const std::map<string, int>& pos_map_w_spr =
      multibody::makeNameToPositionsMap(plant_w_spr);
  const std::map<string, int>& pos_map_wo_spr =
      multibody::makeNameToPositionsMap(plant_wo_spr);
  const std::map<string, int>& vel_map_w_spr =
      multibody::makeNameToVelocitiesMap(plant_w_spr);
  const std::map<string, int>& vel_map_wo_spr =
      multibody::makeNameToVelocitiesMap(plant_wo_spr);

  std::vector<std::string> actuated_joint_names{"hip_roll", "hip_yaw",
                                                "hip_pitch", "knee", "toe"};
  std::vector<std::string> left_right_names{"_left", "_right"};

  // Get mappings
  map_from_q_spring_to_q_actuated_joints_ =
      MatrixXd::Zero(plant_w_spr.num_actuators(), plant_w_spr.num_positions());
  map_from_v_spring_to_v_actuated_joints_ =
      MatrixXd::Zero(plant_w_spr.num_actuators(), plant_w_spr.num_velocities());
  map_from_v_no_spring_to_v_actuated_joints_ = MatrixXd::Zero(
      plant_w_spr.num_actuators(), plant_wo_spr.num_velocities());
  map_from_q_actuated_joints_to_q_spring_ =
      MatrixXd::Zero(plant_w_spr.num_positions(), plant_w_spr.num_actuators());
  map_from_v_actuated_joints_to_v_spring_ =
      MatrixXd::Zero(plant_w_spr.num_velocities(), plant_w_spr.num_actuators());
  int joint_idx = 0;
  for (auto& left_right : left_right_names) {
    for (auto& joint : actuated_joint_names) {
      std::string joint_name = joint + left_right;

      // Get mapping from q_w_spr to q_actuated_joints
      bool successfully_added = false;
      for (const auto& map_element : pos_map_w_spr) {
        if (map_element.first == joint_name) {
          map_from_q_spring_to_q_actuated_joints_(joint_idx,
                                                  map_element.second) = 1;
          successfully_added = true;
        }
      }
      DRAKE_DEMAND(successfully_added);

      // Get mapping from v_w_spr to v_actuated_joints
      successfully_added = false;
      for (const auto& map_element : vel_map_w_spr) {
        if (map_element.first == joint_name + "dot") {
          map_from_v_spring_to_v_actuated_joints_(joint_idx,
                                                  map_element.second) = 1;
          successfully_added = true;
        }
      }
      DRAKE_DEMAND(successfully_added);

      // Get mapping from vdot_wo_spr to vdot_actuated_joints
      successfully_added = false;
      for (const auto& map_element : vel_map_wo_spr) {
        if (map_element.first == joint_name + "dot") {
          map_from_v_no_spring_to_v_actuated_joints_(joint_idx,
                                                     map_element.second) = 1;
          successfully_added = true;
        }
      }
      DRAKE_DEMAND(successfully_added);

      // Get mapping from q_actuated_joints to q_w_spr
      successfully_added = false;
      for (const auto& map_element : pos_map_w_spr) {
        if (map_element.first == joint_name) {
          map_from_q_actuated_joints_to_q_spring_(map_element.second,
                                                  joint_idx) = 1;
          successfully_added = true;
        }
      }
      DRAKE_DEMAND(successfully_added);

      // Get mapping from v_actuated_joints to v_w_spr
      successfully_added = false;
      for (const auto& map_element : vel_map_w_spr) {
        if (map_element.first == joint_name + "dot") {
          map_from_v_actuated_joints_to_v_spring_(map_element.second,
                                                  joint_idx) = 1;
          successfully_added = true;
        }
      }
      DRAKE_DEMAND(successfully_added);

      joint_idx++;
    }
  }
}

void VdotIntegrator::SetInitialTime(Context<double>* context,
                                    double time) const {
  context->get_mutable_discrete_state(prev_time_idx_).get_mutable_value()
      << time;
}
void VdotIntegrator::SetInitialState(Context<double>* context,
                                     const VectorXd& state) const {
  context->get_mutable_discrete_state(actuated_q_idx_).get_mutable_value()
      << map_from_q_spring_to_q_actuated_joints_ * state.head(nq_spr_);
  context->get_mutable_discrete_state(actuated_v_idx_).get_mutable_value()
      << map_from_v_spring_to_v_actuated_joints_ * state.tail(nv_spr_);
}

EventStatus VdotIntegrator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Initialize the desired state
  auto robot_output = (OutputVector<double>*)this->EvalVectorInput(
      context, feddback_state_port_);
  if (!has_been_initialized_ && (robot_output->GetVelocities().norm() != 0)) {
    discrete_state->get_mutable_vector(prev_time_idx_).get_mutable_value()
        << robot_output->get_timestamp();
    discrete_state->get_mutable_vector(actuated_q_idx_).get_mutable_value()
        << map_from_q_spring_to_q_actuated_joints_ *
               robot_output->GetPositions();
    discrete_state->get_mutable_vector(actuated_v_idx_).get_mutable_value()
        << map_from_v_spring_to_v_actuated_joints_ *
               robot_output->GetVelocities();
    //    discrete_state->get_mutable_vector(actuated_v_idx_).get_mutable_value().setZero();
    has_been_initialized_ = true;
  }

  // Read in current vdot
  auto vdot =
      (TimestampedVector<double>*)this->EvalVectorInput(context, vdot_port_);
  double time = vdot->get_timestamp();

  // Read in previous finite state machine state
  auto prev_time =
      discrete_state->get_mutable_vector(prev_time_idx_).get_mutable_value();

  // when entering a new state which is in fsm_states_of_interest
  if (time != prev_time(0)) {
    double dt = time - prev_time(0);
    prev_time(0) = time;

    // Integrate dynamics
    auto q =
        discrete_state->get_mutable_vector(actuated_q_idx_).get_mutable_value();
    auto v =
        discrete_state->get_mutable_vector(actuated_v_idx_).get_mutable_value();
    q += dt * v;
    v += dt * map_from_v_no_spring_to_v_actuated_joints_ * vdot->get_data();

    //    cout << "t = " << time << endl;
    //    cout << "vdot = " << (map_from_v_no_spring_to_v_actuated_joints_ *
    //    vdot->get_data()).transpose() << endl;

    // Add satuation to desired velocity
    double max_vel_des = 10;
    for (int i = 0; i < 10; i++) {
      v(i) = std::max(-max_vel_des, std::min(max_vel_des, v(i)));
    }
  }

  return EventStatus::Succeeded();
}

void VdotIntegrator::CopyState(
    const drake::systems::Context<double>& context,
    systems::TimestampedVector<double>* output) const {
  // Read in current vdot
  auto vdot =
      (TimestampedVector<double>*)this->EvalVectorInput(context, vdot_port_);

  // Read in integrated joint's state
  //  VectorXd desired_state(nx_spr_);
  //  desired_state << map_from_q_actuated_joints_to_q_spring_ *
  //                       context.get_discrete_state(actuated_q_idx_).get_value(),
  //      map_from_v_actuated_joints_to_v_spring_ *
  //          context.get_discrete_state(actuated_v_idx_).get_value();
  VectorXd desired_state = VectorXd::Zero(nx_spr_);
  desired_state.head(nq_spr_) =
      map_from_q_actuated_joints_to_q_spring_ *
      context.get_discrete_state(actuated_q_idx_).get_value();

  output->SetDataVector(desired_state);
  output->set_timestamp(vdot->get_timestamp());
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
