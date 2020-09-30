#include "spring_to_no_spring_converter.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
namespace cassie {

using drake::multibody::JointActuatorIndex;
using drake::multibody::JointIndex;
using drake::systems::Context;
using drake::systems::LeafSystem;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::string;
using systems::OutputVector;

SpringToNoSpringConverter::SpringToNoSpringConverter(
    const drake::multibody::MultibodyPlant<double>& plant_w_spr,
    const drake::multibody::MultibodyPlant<double>& plant_wo_spr) {
  int nq_w_spr = plant_w_spr.num_positions();
  int nv_w_spr = plant_w_spr.num_velocities();
  int nu_w_spr = plant_w_spr.num_actuators();
  nq_wo_spr_ = plant_wo_spr.num_positions();
  nv_wo_spr_ = plant_wo_spr.num_velocities();
  nu_wo_spr_ = plant_wo_spr.num_actuators();

  const std::map<string, int>& pos_map_w_spr =
      multibody::makeNameToPositionsMap(plant_w_spr);
  const std::map<string, int>& vel_map_w_spr =
      multibody::makeNameToVelocitiesMap(plant_w_spr);
  const std::map<string, int>& pos_map_wo_spr =
      multibody::makeNameToPositionsMap(plant_wo_spr);
  const std::map<string, int>& vel_map_wo_spr =
      multibody::makeNameToVelocitiesMap(plant_wo_spr);

  // Initialize the mapping from spring to no spring
  map_position_from_spring_to_no_spring_ = MatrixXd::Zero(nq_wo_spr_, nq_w_spr);
  map_velocity_from_spring_to_no_spring_ = MatrixXd::Zero(nv_wo_spr_, nv_w_spr);

  for (auto pos_pair_wo_spr : pos_map_wo_spr) {
    bool successfully_added = false;
    for (auto pos_pair_w_spr : pos_map_w_spr) {
      if (pos_pair_wo_spr.first == pos_pair_w_spr.first) {
        map_position_from_spring_to_no_spring_(pos_pair_wo_spr.second,
                                               pos_pair_w_spr.second) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }
  for (auto vel_pair_wo_spr : vel_map_wo_spr) {
    bool successfully_added = false;
    for (auto vel_pair_w_spr : vel_map_w_spr) {
      if (vel_pair_wo_spr.first == vel_pair_w_spr.first) {
        map_velocity_from_spring_to_no_spring_(vel_pair_wo_spr.second,
                                               vel_pair_w_spr.second) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }

  this->DeclareVectorInputPort(
          OutputVector<double>(nq_w_spr, nv_w_spr, nu_w_spr))
      .get_index();
  this->DeclareVectorOutputPort(
      OutputVector<double>(nq_wo_spr_, nv_wo_spr_, nu_wo_spr_),
      &SpringToNoSpringConverter::CopyOutput);
}

void SpringToNoSpringConverter::CopyOutput(const Context<double>& context,
                                           OutputVector<double>* output) const {
  // Read in robot output with spring
  const auto robot_output_w_spr =
      this->template EvalVectorInput<OutputVector>(context, 0);

  // Assign robot output without spring
  output->SetPositions(map_position_from_spring_to_no_spring_ *
                       robot_output_w_spr->GetPositions());
  output->SetVelocities(map_velocity_from_spring_to_no_spring_ *
                        robot_output_w_spr->GetVelocities());
  output->SetEfforts(robot_output_w_spr->GetEfforts());
  output->set_timestamp(robot_output_w_spr->get_timestamp());
}

}  // namespace cassie
}  // namespace dairlib
