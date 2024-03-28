#include "heuristic_generator.h"
#include <iostream>


using dairlib::systems::OutputVector;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::EventStatus;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

HeuristicGenerator::HeuristicGenerator(
    const MultibodyPlant<double>& robot_plant,
    const SimulateFrankaParams& sim_param,
    const HeuristicGaitParams& heuristic_param) {
    // INPUT PORTS, get current plant state (franka + object)
    plant_state_port_ =
    this->DeclareVectorInputPort(
                    "plant_state", OutputVector<double>(robot_plant.num_positions(),
                                                        robot_plant.num_velocities(),
                                                        robot_plant.num_actuators()))
            .get_index();
    // OUTPUT PORTS 1: send out simple model desired state (y_des)
    target_port_ =
      this->DeclareVectorOutputPort(
              "track_target", BasicVector<double>(3),
              &HeuristicGenerator::CalcHeuristicTarget)
          .get_index();

}

void HeuristicGenerator::SetHeuristicParameters(
        const HeuristicGaitParams& heuristic_param) {
    roll_phase_ = heuristic_param.roll_phase;
    return_phase_ = heuristic_param.return_phase;
    gait_parameters_ = heuristic_param.gait_parameters;
    axis_option_ = heuristic_param.axis_option;
    tilt_degrees_ = heuristic_param.tilt_degrees;
    q_new_vector_ = heuristic_param.q_new_vector;
}

void HeuristicGenerator::CalcHeuristicTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* target) const {

  // Evaluate input port for object state
  auto plant_state = (OutputVector<double>*)this->EvalVectorInput(context, plant_state_port_);

  // Get ball position and timestamp
  VectorXd obj_curr_position = plant_state->GetPositions().tail(3);
  double timestamp = plant_state->get_timestamp();
  SimulateFrankaParams sim_param = drake::yaml::LoadYamlFile<SimulateFrankaParams>(
            "examples/franka_ball_rolling/parameters/simulate_franka_params.yaml");
  double object_height = sim_param.ball_radius - sim_param.ground_offset_frame(2);

  // Initialize target pose
  VectorXd target_obj_state = VectorXd::Zero(7);
  VectorXd target_obj_position = VectorXd::Zero(3);

  target_obj_state << 1, 0, 0, 0, target_obj_position;
  target->SetFromVector(target_obj_state);
}
}  // namespace systems
}  // namespace dairlib