#include <vector>
#include <math.h>
#include <gflags/gflags.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/tree/multibody_element.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/math/rigid_transform.h>
#include "drake/math/autodiff.h"


#include "systems/robot_lcm_systems.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_c3.hpp"
#include "multibody/multibody_utils.h"
#include "systems/system_utils.h"

#include "examples/cube_franka/c3_parameters.h"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/c3_controller_franka_cube.h"
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
using multibody::makeNameToPositionsMap;
using multibody::makeNameToVelocitiesMap;
using drake::trajectories::PiecewisePolynomial;

using Eigen::VectorXd;
using Eigen::MatrixXd;

int DoMain(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  C3Parameters param = drake::yaml::LoadYamlFile<C3Parameters>(
    "examples/cube_franka/parameters.yaml");
  drake::lcm::DrakeLcm drake_lcm;
  drake::lcm::DrakeLcm drake_lcm_network("udpm://239.255.76.67:7667?ttl=1");


  //This plant that is created is mainly to allow access to the drake methods of accessing parts by name etc. and is a very general version of the plant
  /// parse plant from urdfs
  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  parser.package_map().Add("robot_properties_fingers",
                        "examples/cube_franka/robot_properties_fingers");
  parser.AddModelFromFile("examples/cube_franka/robot_properties_fingers/urdf/trifinger_minimal_collision_2.urdf");
  parser.AddModelFromFile("examples/cube_franka/robot_properties_fingers/urdf/jack.urdf");

  /// Fix base of finger to world
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"), X_WI);
  plant.Finalize();

  DiagramBuilder<double> builder;

  /* -------------------------------------------------------------------------------------------*/

  DiagramBuilder<double> builder_f;
  
  //This plant_f is the simplified version of our robot that takes into account only the end effector and ball
  auto [plant_f, scene_graph] = AddMultibodyPlantSceneGraph(&builder_f, 0.0);
  Parser parser_f(&plant_f);
  parser_f.package_map().Add("robot_properties_fingers",
                        "examples/cube_franka/robot_properties_fingers");
  parser_f.AddModelFromFile("examples/cube_franka/robot_properties_fingers/urdf/trifinger_minimal_collision_2.urdf");
  parser_f.AddModelFromFile("examples/cube_franka/robot_properties_fingers/urdf/jack.urdf");
  RigidTransform<double> X_WI_f = RigidTransform<double>::Identity();
  plant_f.WeldFrames(plant_f.world_frame(), plant_f.GetFrameByName("base_link"), X_WI_f);
  plant_f.Finalize();

  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_ad_f = 
    drake::systems::System<double>::ToAutoDiffXd(plant_f);
  auto context_ad_f = plant_ad_f->CreateDefaultContext();
  auto diagram_f = builder_f.Build();
  std::unique_ptr<Context<double>> diagram_context = diagram_f->CreateDefaultContext();
  auto& context_f = diagram_f->GetMutableSubsystemContext(plant_f, diagram_context.get());

  /* -------------------------------------------------------------------------------------------*/
  //This plant_franka is a copy of the original full robot model including the franka arm, 
  //ee and ball but is not linked in any way to the ground truth model plant in simulate_lcm_franka. 
  //This plant here is used to take in the input of the state estimator(the joint positions and ball positions) 
  //and perform FK on it to get the end effector position and ball position to later set ee.position and ball.position accordingly.
  DiagramBuilder<double> builder_franka;
  double sim_dt = param.sim_dt;  // TODO: This was hardcoded at 1e-4 but should most likely match this parameter.

  auto [plant_franka, scene_graph_franka] = AddMultibodyPlantSceneGraph(&builder_franka, sim_dt);
  Parser parser_franka(&plant_franka);
  parser_franka.AddModelFromFile("examples/cube_franka/robot_properties_fingers/urdf/franka_box.urdf");
  parser_franka.AddModelFromFile("examples/cube_franka/robot_properties_fingers/urdf/jack.urdf");
  RigidTransform<double> X_WI_franka = RigidTransform<double>::Identity();
  plant_franka.WeldFrames(plant_franka.world_frame(), plant_franka.GetFrameByName("panda_link0"), X_WI_franka);
  plant_franka.Finalize();
  auto context_franka = plant_franka.CreateDefaultContext();

  /* -------------------------------------------------------------------------------------------*/

  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nu = plant.num_actuators();
  int nc = 4;  //number of contacts 

  VectorXd q = VectorXd::Zero(nq);
  std::map<std::string, int> q_map = makeNameToPositionsMap(plant);
  std::map<std::string, int> v_map = makeNameToVelocitiesMap(plant);
  
  q(0) = param.q_init_finger(0);
  q(1) = param.q_init_finger(1);
  //q(2) = param.q_init_finger(2) + param.table_offset;
  q(2) = param.finger_radius + 2*param.jack_half_width + param.table_offset;

  q[q_map["base_qw"]] = param.q_init_ball_c3(0);
  q[q_map["base_qx"]] = param.q_init_ball_c3(1);
  q[q_map["base_qy"]] = param.q_init_ball_c3(2);
  q[q_map["base_qz"]] = param.q_init_ball_c3(3);
  q[q_map["base_x"]] = param.q_init_ball_c3(4);
  q[q_map["base_y"]] = param.q_init_ball_c3(5);
  q[q_map["base_z"]] = param.q_init_ball_c3(6);
  double mu = param.mu;

  MatrixXd Qinit = param.Q_default * MatrixXd::Identity(nq+nv, nq+nv);
  Qinit.block(0,0,3,3) << param.Q_finger * MatrixXd::Identity(3,3);
  Qinit(7,7) = param.Q_ball_x;
  Qinit(8,8) = param.Q_ball_y;
  Qinit(9,9) = param.Q_ball_z;
  // Qinit.block(10,10,nv,nv) << param.Q_ball_vel * MatrixXd::Identity(nv,nv);   // This is sketchy?  TODO
  Qinit.block(16,16,3,3) << param.Q_ball_vel * MatrixXd::Identity(3,3);
  Qinit.block(10,10,3,3) << param.Q_finger_vel * MatrixXd::Identity(3,3);
  MatrixXd Rinit = param.R * MatrixXd::Identity(nu, nu);

  MatrixXd Ginit = param.G * MatrixXd::Identity(nq+nv+nu+6*nc, nq+nv+nu+6*nc);
  Ginit.block(nq+nv+nc, nq+nv+nc, 1, 1) = param.G_EE * MatrixXd::Identity(1, 1);                        // End effector normal forces.
  Ginit.block(nq+nv+nc+1, nq+nv+nc+1, nc-1, nc-1) = param.G_ground * MatrixXd::Identity(nc-1, nc-1);    // Ground normal forces.
  // Ginit.block(nq+nv+4+4+4, nq+nv+4+4+4, 3*4, 3*4) = param.G_ground * MatrixXd::Identity(3*4, 3*4);      // Ground tangential forces.

  MatrixXd Uinit = param.U_default * MatrixXd::Identity(nq+nv+nu+6*nc, nq+nv+nu+6*nc);
  //Penalizing change in position between projection steps
  Uinit.block(0,0,nq,nq) << 
    param.U_pos * MatrixXd::Identity(nq,nq);
  //Penalizing change in velocity between projection steps
  Uinit.block(nq,nq,nv,nv) << 
    param.U_vel * MatrixXd::Identity(nv,nv);
  //Penalizing change in input between projection steps
  Uinit.block(nq+nv+6*nc, nq+nv+6*nc, nu, nu) << 
    param.U_u * MatrixXd::Identity(nu, nu);
  
  VectorXd xdesiredinit = VectorXd::Zero(nq+nv);
  xdesiredinit.head(nq) << q;

  /* -------------------------------------------------------------------------------------------*/

  double r = param.traj_radius;
  double xc = param.x_c;
  double yc = param.y_c;
  double degree_increment = param.degree_increment;
  double lower, upper;
  assert(degree_increment > 0); // avoid potential infinite loops

  if (param.hold_order == 0){
    lower = 0;
    upper = 40000;
  }
  else if (param.hold_order == 1){
    lower = degree_increment;
    upper = 400 + degree_increment;      
  }
  else{
    std::cout << "Received an invalid hold_order parameter." << std::endl;
    return 0;
  }
  
  std::vector<double> values;
  for (int val = lower; val < upper; val+=degree_increment){
    values.push_back(val);
  }
  VectorXd theta(values.size());
  for (int i = 0; i < (int) values.size(); i++){
    theta(i) = values[i];
  }

  ///change this part for different shapes
  std::vector<VectorXd> xtraj;
  for (int i = 0; i < (int) values.size(); i++){

      ///line
      int line_step_size = 10;
      double pattern_line_y[line_step_size*2];
      for(int j=0; j<line_step_size; j++)
        pattern_line_y[j] = -0.1 + j * (0.2) / line_step_size;
      for(int j=line_step_size; j<2*line_step_size; j++)
        pattern_line_y[j] = 0.1 - (j-line_step_size) * (0.2) / line_step_size;
      double x = 0.55;
      double y = pattern_line_y[i%(2*line_step_size)];
      q(q_map["base_x"]) = x;
      q(q_map["base_y"]) = y;
      q(q_map["base_z"]) = param.jack_half_width + param.table_offset;
      VectorXd xtraj_hold = VectorXd::Zero(nq + nv);
      xtraj_hold.head(nq) << q;
      xtraj.push_back(xtraj_hold);

//      ///square
    //  int line_step_size = 5;
    //  double pattern_line_x[line_step_size*4];
    //  double pattern_line_y[line_step_size*4];
    //  for(int j=0; j<line_step_size; j++)
    //      pattern_line_x[j] = 0.55;
    //  for(int j=line_step_size; j<2*line_step_size; j++)
    //      pattern_line_x[j] = 0.55 + (j-line_step_size) * (0.1) / line_step_size;
    //  for(int j=2*line_step_size; j<3*line_step_size; j++)
    //      pattern_line_x[j] = 0.65;
    //  for(int j=3*line_step_size; j<4*line_step_size; j++)
    //      pattern_line_x[j] = 0.65 - (j-3*line_step_size) * (0.1) / line_step_size;

    //  for(int j=0; j<line_step_size; j++)
    //      pattern_line_y[j] = -0.1 + j * (0.2) / line_step_size;
    //  for(int j=line_step_size; j<2*line_step_size; j++)
    //      pattern_line_y[j] = 0.1;
    //  for(int j=2*line_step_size; j<3*line_step_size; j++)
    //      pattern_line_y[j] = 0.1 - (j-2*line_step_size) * (0.2) / line_step_size;
    //  for(int j=3*line_step_size; j<4*line_step_size; j++)
    //      pattern_line_y[j] = -0.1;

    //  double x = pattern_line_x[i%(4*line_step_size)];;
    //  double y = pattern_line_y[i%(4*line_step_size)];
    //  q(q_map["base_x"]) = x;
    //  q(q_map["base_y"]) = y;
    //  q(q_map["base_z"]) = param.ball_radius + param.table_offset;
    //  VectorXd xtraj_hold = VectorXd::Zero(nq + nv);
    //  xtraj_hold.head(nq) << q;
    //  xtraj.push_back(xtraj_hold);

//    ///circle (reverse)
//    double angle = theta(i);
//    double x = -r * sin(M_PI * (angle + param.phase) / 180.0);
//    double y = r * cos(M_PI * (angle + param.phase) / 180.0);
//    q(q_map["base_x"]) = x + xc;
//    q(q_map["base_y"]) = y + yc;
//    q(q_map["base_z"]) = param.ball_radius + param.table_offset;
//    VectorXd xtraj_hold = VectorXd::Zero(nq+nv);
//    xtraj_hold.head(nq) << q;
//    xtraj.push_back(xtraj_hold);
  }

  // convert xtraj into MatrixXd
  MatrixXd xtraj_mat(nq+nv, values.size());
  for (int i = 0; i < (int) values.size(); i++){
    xtraj_mat.block(0, i, nv+nq, 1) << xtraj[i];
  }

  double time_increment = param.time_increment;
  double settling_time = param.stabilize_time1 + param.move_time + param.stabilize_time2;
  VectorXd timings(values.size());
  for (int i = 0; i < (int) values.size(); i++){
    timings(i) = settling_time + i * time_increment;
  }

  PiecewisePolynomial<double> pp;
  if (param.hold_order == 0){
    pp = PiecewisePolynomial<double>::ZeroOrderHold(timings, xtraj_mat);
  }
  else if (param.hold_order == 1){
    pp = PiecewisePolynomial<double>::FirstOrderHold(timings, xtraj_mat);
  }

  /* -------------------------------------------------------------------------------------------*/

  int num_friction_directions = 2;
  int N = param.horizon_length;   //5
  std::vector<MatrixXd> Q, R, G, U;
  std::vector<VectorXd> xdesired;

  for (int i = 0; i < N; i++){
    Q.push_back(Qinit);
    R.push_back(Rinit);
    G.push_back(Ginit);
    U.push_back(Uinit);
    xdesired.push_back(xdesiredinit);
  }
  Q.push_back(Qinit);
  xdesired.push_back(xdesiredinit);

  drake::geometry::GeometryId finger_geoms = 
    plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("tip_link_1_real"))[0];
    // drake::geometry::GeometryId sphere_geoms = 
    // plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("sphere_v2"))[0];
  drake::geometry::GeometryId capsule1_geoms = 
    plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("capsule_1"))[0];
  drake::geometry::GeometryId capsule2_geoms = 
    plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("capsule_2"))[0];
  drake::geometry::GeometryId capsule3_geoms = 
    plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("capsule_3"))[0];
  drake::geometry::GeometryId ground_geoms = 
    plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("box"))[0];
  std::vector<drake::geometry::GeometryId> contact_geoms = 
    {finger_geoms, capsule1_geoms, capsule2_geoms, capsule3_geoms, ground_geoms};

  /* -------------------------------------------------------------------------------------------*/

  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_ad = 
    drake::systems::System<double>::ToAutoDiffXd(plant);
  auto context = plant.CreateDefaultContext();
  auto context_ad = plant_ad->CreateDefaultContext();

  //state receiver gets the joint configurations from the full model franka arm and jack
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);

  
  
  auto controller = builder.AddSystem<systems::controllers::C3Controller_franka>(
                                  plant, plant_f, plant_franka, *context, 
                                  context_f, *context_franka, *plant_ad, 
                                  *plant_ad_f, *context_ad, *context_ad_f, 
                                  scene_graph, *diagram_f, contact_geoms, 
                                  num_friction_directions, mu, Q, R, G, U, 
                                  xdesired, pp);

  // Function Arguments: int num_positions, int num_velocities, int lambda_size, int  misc_size
  auto state_force_sender = builder.AddSystem<systems::RobotC3Sender>(NUM_POSITIONS, NUM_VELOCITIES, NUM_LAMBDAS, NUM_VISUALIZATION);

  builder.Connect(state_receiver->get_output_port(0), controller->get_input_port(0));    
  builder.Connect(controller->get_output_port(), state_force_sender->get_input_port(0));

  // determine if ttl 0 or 1 should be used for publishing
  drake::lcm::DrakeLcm* pub_lcm;
  if (FLAGS_TTL == 0) {
    std::cout << "Using TTL=0" << std::endl;
    pub_lcm = &drake_lcm;
  }
  else if (FLAGS_TTL == 1) {
    std::cout << "Using TTL=1" << std::endl;
    pub_lcm = &drake_lcm_network;
  }
  auto control_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_c3>(
        "CONTROLLER_INPUT", pub_lcm, 
        {drake::systems::TriggerType::kForced}, 0.0));
  builder.Connect(state_force_sender->get_output_port(),
      control_publisher->get_input_port());

  auto diagram = builder.Build();
  DrawAndSaveDiagramGraph(*diagram, "examples/cube_franka/diagram_lcm_control_demo");
  DrawAndSaveDiagramGraph(*diagram_f, "examples/cube_franka/diagram_lcm_control_demo_f");
  auto context_d = diagram->CreateDefaultContext();
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &drake_lcm, std::move(diagram), state_receiver, "FRANKA_OUTPUT", true);
  
  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}
