#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::systems::controllers::id_mpc {

struct GaitParams {
  Eigen::VectorXd standing_pose_q;
  Eigen::VectorXd standing_pose_lambda;
  Eigen::VectorXd standing_pose_u;
  std::string left_foot_body_name;
  std::string right_foot_body_name;
  std::vector<std::string> left_foot_contacts;
  std::vector<std::string> right_foot_contacts;
  Eigen::Vector3d foot_midpoint;
};

class WalkingReferenceSystem : public drake::systems::LeafSystem<double> {
 public:

  WalkingReferenceSystem(
      const drake::multibody::MultibodyPlant<double> &plant,
      drake::systems::Context<double> *plant_context,
      const GaitParams& params);

 private:

  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::systems::Context<double> *plant_context_;
  GaitParams params_;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_vdes_;

  drake::systems::AbstractStateIndex reference_state_idx_;
};

}