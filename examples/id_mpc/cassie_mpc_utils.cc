#include <memory>
#include "cassie_mpc_utils.h"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"


namespace dairlib {

using systems::controllers::id_mpc::ConstrainedDynamicsInfo;
using systems::controllers::id_mpc::GaitParams;
using systems::controllers::id_mpc::IDMPCParams;

using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::multibody::MultibodyPlant;

std::unique_ptr<ConstrainedDynamicsInfo> MakeCassieDynamics() {
  std::string urdf = "examples/Cassie/urdf/cassie_fixed_spring_conservative.urdf";
  auto dynamics_info = std::make_unique<ConstrainedDynamicsInfo>(urdf);

  VectorXd rotor_inertias(10);
  rotor_inertias << 61, 61, 61, 61, 365, 365, 365, 365, 4.9, 4.9;
  rotor_inertias *= 1e-6;
  VectorXd gear_ratios(10);
  gear_ratios << 25, 25, 25, 25, 16, 16, 16, 16, 50, 50;
  std::vector<std::string> motor_joint_names = {
      "hip_roll_left_motor", "hip_roll_right_motor", "hip_yaw_left_motor",
      "hip_yaw_right_motor", "hip_pitch_left_motor", "hip_pitch_right_motor",
      "knee_left_motor",     "knee_right_motor",     "toe_left_motor",
      "toe_right_motor"};

  for (int i = 0; i < rotor_inertias.size(); ++i) {
    auto& joint_actuator = dynamics_info->get_mutable_plant().get_mutable_joint_actuator(
        drake::multibody::JointActuatorIndex(i));
    joint_actuator.set_default_rotor_inertia(rotor_inertias(i));
    joint_actuator.set_default_gear_ratio(gear_ratios(i));
    DRAKE_DEMAND(motor_joint_names[i] == joint_actuator.name());
  }

  dynamics_info->Finalize();

  dynamics_info->AddContactPoint(
      "toe_left_front",
      "toe_left",
      Vector3d(-0.0457, 0.112, 0),
      {0, 1, 2},
      0.8
  );
  dynamics_info->AddContactPoint(
      "toe_left_rear",
      "toe_left",
      Vector3d(0.088, 0, 0),
      {1, 2},
      0.8
  );
  dynamics_info->AddContactPoint(
      "toe_right_front",
      "toe_right",
      Vector3d(-0.0457, 0.112, 0),
      {0, 1, 2},
      0.8
  );
  dynamics_info->AddContactPoint(
      "toe_right_rear",
      "toe_right",
      Vector3d(0.088, 0, 0),
      {1, 2},
      0.8
  );

  dynamics_info->AddDistanceConstraint(
      "thigh_left",
      Vector3d(0.0, 0.0, 0.045),
      "heel_spring_left",
      Vector3d(.11877, -.01, 0.0),
      0.5012
  );

  dynamics_info->AddDistanceConstraint(
      "thigh_right",
      Vector3d(0.0, 0.0, -0.045),
      "heel_spring_right",
      Vector3d(.11877, -.01, 0.0),
      0.5012
  );

  return std::move(dynamics_info);
}

GaitParams MakeCassieGaitParams(const IDMPCParams& mpc_params) {
  std::string urdf = "examples/Cassie/urdf/cassie_fixed_spring_conservative.urdf";
  MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, nullptr, true, urdf, false, false);
  plant.Finalize();

  GaitParams params;
  params.t_ss = 0.35;
  params.t_ds = 0.05;
  params.mpc_N = mpc_params.N;
  params.mpc_dt = mpc_params.dt;
  params.stance_width = 0.3;

  params.standing_pose_q = VectorXd::Zero(plant.num_positions());
  params.standing_pose_u = VectorXd::Zero(plant.num_actuators());
  params.standing_pose_lambda = VectorXd::Zero(3 * 4 + 2);

  CassieFixedPointSolver(plant, 0.9, mpc_params.mu, 20, true,
                         0.5 * params.stance_width,
                         &params.standing_pose_q,
                         &params.standing_pose_u,
                         &params.standing_pose_lambda);

  params.left_foot_body_name = "toe_left";
  params.right_foot_body_name = "toe_right";
  params.floating_base_name = "pelvis";
  params.left_foot_contacts = {"toe_left_front", "toe_left_rear"};
  params.right_foot_contacts = {"toe_right_front", "toe_right_rear"};
  params.right_leg_holonomic_constraint_idxs = {1};
  params.left_leg_holonomic_constraint_idxs = {0};
  params.foot_midpoint =
      0.5 * (RightToeRear(plant).first + RightToeFront(plant).first);

  auto act_map = multibody::MakeNameToActuatorsMap(plant);

  params.left_leg_actuator_idxs = {
      act_map["hip_roll_left_motor"],
      act_map["hip_yaw_left_motor"],
      act_map["hip_pitch_left_motor"],
      act_map["knee_left_motor"],
      act_map["toe_left_motor"]
  };

  params.right_leg_actuator_idxs = {
      act_map["hip_roll_right_motor"],
      act_map["hip_yaw_right_motor"],
      act_map["hip_pitch_right_motor"],
      act_map["knee_right_motor"],
      act_map["toe_right_motor"]
  };

  return params;
}

}