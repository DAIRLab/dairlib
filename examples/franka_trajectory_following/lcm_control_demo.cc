#include <vector>
#include <math.h>

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

#include "c3_parameters.h"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/c3_controller_franka.h"
#include "systems/framework/lcm_driven_loop.h"

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
    C3Parameters param = drake::yaml::LoadYamlFile<C3Parameters>(
      "examples/franka_trajectory_following/parameters.yaml");
    drake::lcm::DrakeLcm drake_lcm;

    /// parse plant from urdfs
    MultibodyPlant<double> plant(0.0);
    Parser parser(&plant);
    parser.package_map().Add("robot_properties_fingers",
                         "examples/franka_trajectory_following/robot_properties_fingers");
    parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/trifinger_minimal_collision_2.urdf");
    parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere.urdf");

    /// Fix base of finger to world
    RigidTransform<double> X_WI = RigidTransform<double>::Identity();
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"), X_WI);
    plant.Finalize();

    DiagramBuilder<double> builder;

    /* -------------------------------------------------------------------------------------------*/

    DiagramBuilder<double> builder_f;

    auto [plant_f, scene_graph] = AddMultibodyPlantSceneGraph(&builder_f, 0.0);
    Parser parser_f(&plant_f);
    parser_f.package_map().Add("robot_properties_fingers",
                         "examples/franka_trajectory_following/robot_properties_fingers");
    parser_f.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/trifinger_minimal_collision_2.urdf");
    parser_f.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere.urdf");
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

    DiagramBuilder<double> builder_franka;
    double sim_dt = 1e-4;

    auto [plant_franka, scene_graph_franka] = AddMultibodyPlantSceneGraph(&builder_franka, sim_dt);
    Parser parser_franka(&plant_franka);
    parser_franka.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/franka_box.urdf");
    parser_franka.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere.urdf");
    RigidTransform<double> X_WI_franka = RigidTransform<double>::Identity();
    plant_franka.WeldFrames(plant_franka.world_frame(), plant_franka.GetFrameByName("panda_link0"), X_WI_franka);
    plant_franka.Finalize();
    auto context_franka = plant_franka.CreateDefaultContext();

    /* -------------------------------------------------------------------------------------------*/

    int nq = plant.num_positions();
    int nv = plant.num_velocities();
    int nu = plant.num_actuators();
    int nc = 2;

    VectorXd q = VectorXd::Zero(nq);
    std::map<std::string, int> q_map = makeNameToPositionsMap(plant);
    std::map<std::string, int> v_map = makeNameToVelocitiesMap(plant);
    
    q(0) = param.q_init_finger(0);
    q(1) = param.q_init_finger(1);
    q(2) = param.q_init_finger(2);

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
    Qinit.block(10,10,nv,nv) << param.Q_ball_vel * MatrixXd::Identity(nv,nv);
    Qinit.block(10,10,3,3) << param.Q_finger_vel * MatrixXd::Identity(3,3);
    MatrixXd Rinit = param.R * MatrixXd::Identity(nu, nu);

    MatrixXd Ginit = param.G * MatrixXd::Identity(nq+nv+nu+6*nc, nq+nv+nu+6*nc);
    MatrixXd Uinit = param.U_default * MatrixXd::Identity(nq+nv+nu+6*nc, nq+nv+nu+6*nc);
    Uinit.block(0,0,nq+nv,nq+nv) << 
      param.U_pos_vel * MatrixXd::Identity(nq+nv,nq+nv);
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
      upper = 400;
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

    std::vector<VectorXd> xtraj;
    for (int i = 0; i < (int) values.size(); i++){
      double angle = theta(i);
      double x = r * sin(M_PI * (angle + param.phase) / 180.0);
      double y = r * cos(M_PI * (angle + param.phase) / 180.0);
      q(q_map["base_x"]) = x + xc;
      q(q_map["base_y"]) = y + yc;
      q(q_map["base_z"]) = param.ball_radius + param.table_offset;
      VectorXd xtraj_hold = VectorXd::Zero(nq+nv);
      xtraj_hold.head(nq) << q;
      xtraj.push_back(xtraj_hold); 
    }

    // convert xtraj into MatrixXd
    MatrixXd xtraj_mat(values.size(), nq+nv);
    for (int i = 0; i < (int) values.size(); i++){
      xtraj_mat.block(i, 0, 1, nv+nq) << xtraj[i].transpose();
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
    int N = 5;
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
    drake::geometry::GeometryId sphere_geoms = 
      plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("sphere"))[0];
    drake::geometry::GeometryId ground_geoms = 
      plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("box"))[0];
    std::vector<drake::geometry::GeometryId> contact_geoms = 
      {finger_geoms, sphere_geoms, ground_geoms};

    /* -------------------------------------------------------------------------------------------*/

    std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_ad = 
      drake::systems::System<double>::ToAutoDiffXd(plant);
    auto context = plant.CreateDefaultContext();
    auto context_ad = plant_ad->CreateDefaultContext();

    auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
    
    // TODO: build this line
    // auto controller = builder.AddSystem<systems::controllers::C3Controller_franka>(
    //                                 plant, plant_f, plant_franka, context, 
    //                                 context_f, context_franka, plant_ad, 
    //                                 plant_ad_f, context_ad, context_ad_f, 
    //                                 scene_graph, diagram_f, contact_geoms, 
    //                                 num_friction_directions, mu, Q, R, G, U, 
    //                                 xdesired, pp);
    auto state_force_sender = builder.AddSystem<systems::RobotC3Sender>(10, 9, 6, 9);

    // TODO: build these lines
    // builder.Connect(state_receiver->get_output_port(0), controller->get_input_port(0));    
    // builder.Connect(controller->get_output_port(), state_force_sender->get_input_port(0));

    auto control_publisher = builder.AddSystem(
        LcmPublisherSystem::Make<dairlib::lcmt_c3>(
          "CONTROLLER_INPUT", &drake_lcm, 
          {drake::systems::TriggerType::kForced}, 0.0));

    auto diagram = builder.Build();
    // Run lcm-driven simulation
    systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
        &drake_lcm, std::move(diagram), state_receiver, "FRANKA_OUTPUT", true);
    
    // TODO: code to connect the publisher


    loop.Simulate(std::numeric_limits<double>::infinity());

    return 0;
  }
} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}
