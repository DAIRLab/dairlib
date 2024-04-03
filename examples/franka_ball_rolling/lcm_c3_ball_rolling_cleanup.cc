#include <vector>
#include <math.h>
#include <gflags/gflags.h>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/math/rigid_transform.h>
#include "drake/math/autodiff.h"


#include "systems/robot_lcm_systems.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_c3.hpp"
#include "multibody/multibody_utils.h"
#include "systems/system_utils.h"


#include "examples/franka_ball_rolling/parameters/heuristic_gait_params.h"
#include "examples/franka_ball_rolling/parameters/trajectory_params.h"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "solvers/c3_options.h"

#include "systems/controllers/c3_controller.h"
#include "systems/framework/lcm_driven_loop.h"

DEFINE_int32(TTL, 0,
              "TTL level for publisher. "
              "Default value is 0.");

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::math::RigidTransform;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::Context;
using drake::multibody::Parser;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

int DoMain(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  /* -------------------------------- Load Parameters --------------------------------------------*/
  SimulateFrankaParams sim_param = drake::yaml::LoadYamlFile<SimulateFrankaParams>(
          "examples/franka_ball_rolling/parameters/simulate_franka_params.yaml");
  BallRollingTrajectoryParams traj_param = drake::yaml::LoadYamlFile<BallRollingTrajectoryParams>(
          "examples/franka_ball_rolling/parameters/trajectory_params.yaml");
  HeuristicGaitParams heuristic_param = drake::yaml::LoadYamlFile<HeuristicGaitParams>(
            "examples/franka_ball_rolling/parameters/heuristic_gait_params.yaml");


  /* -------------------------------- Setup LCM --------------------------------------------*/
  drake::lcm::DrakeLcm lcm;
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");


  /* ----------------------- Create plants for Forward Kinematics  --------------------------*/
  // Forward kinematics need single franka plant and signle object plant
  // Franka plant
  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.AddModels(sim_param.franka_model);
  drake::multibody::ModelInstanceIndex end_effector_index =
            parser_franka.AddModels(sim_param.end_effector_model)[0];
  parser_franka.AddModels(sim_param.ground_model);
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_EE = RigidTransform<double>(sim_param.tool_attachment_frame);
  RigidTransform<double> X_F_G = RigidTransform<double>(sim_param.ground_offset_frame);

  plant_franka.WeldFrames(plant_franka.world_frame(),
                            plant_franka.GetFrameByName("panda_link0"), X_WI);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link7"),
                          plant_franka.GetFrameByName("end_effector_base"), X_F_EE);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link0"),
                          plant_franka.GetFrameByName("ground"), X_F_G);
  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  // Ball plant
  MultibodyPlant<double> plant_ball(0.0);
  Parser parser_ball(&plant_ball, nullptr);
  parser_ball.AddModels(sim_param.ball_model);
  plant_ball.Finalize();
  auto ball_context = plant_ball.CreateDefaultContext();

  DiagramBuilder<double> builder;
  /* ----------------------- Subscriber and Sender for forward kinematics --------------------------*/
  auto ball_state_sub =
            builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
                    "BALL_STATE_ESTIMATE_NEW", &lcm));
  auto franka_state_receiver =
            builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
  auto ball_state_receiver =
            builder.AddSystem<systems::ObjectStateReceiver>(plant_ball);


  /* -------------------------------------------------------------------------------------------*/
//  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}
