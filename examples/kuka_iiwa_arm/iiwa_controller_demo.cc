#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"

#include "systems/controllers/endeffector_velocity_controller.h"
#include "systems/controllers/endeffector_position_controller.h"
#include <lcm/lcm-cpp.hpp>

#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using json = nlohmann::json;
namespace dairlib {

// CsvVector class: Takes CSV file as a parameter,
// loads data into a 2D vector of doubles, provides
// method to get the 2D vector.
class CsvVector {
  private:
    std::vector<std::vector<double>> data;
  public:
    CsvVector(std::string fileName) {
      //Opens stream to fileName.
      std::ifstream stream;
      stream.open(fileName);

      //Checks to make sure it has been opened.
      if (stream.is_open()) {
        std::cout << "CSV file " + fileName + " opened successfully" << std::endl;
      }

      //Finds number of columns in CSV file to resize data vector.
      std::string firstLine;
      std::getline(stream, firstLine);
      std::stringstream lineStream(firstLine);
      std::string dat;
      int length = 0;
      while (std::getline(lineStream, dat, ',')) {
        data.resize(data.size() + 1);
        data[length].push_back(std::stod(dat));
        length++;
      }

      //Loads CSV file contents into data vector.
      while(std::getline(stream, firstLine)) {
        int col = 0;
        std::stringstream moreLines(firstLine);
        while (std::getline(moreLines, dat, ',')) {
          data[col].push_back(std::stod(dat));
          col++;
        }
      }
      stream.close();
    }
    //Returns the data array.
    std::vector<std::vector<double>> getArray() {
      return data;
    }

    int knotSize() {
        return data[0].size();
    }
};

class InitialPosHandler {
    public:
        InitialPosHandler(){
            messageReceived = false;
        }

        ~InitialPosHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const drake::lcmt_iiwa_status* msg) {
            initialPositions << (double)msg->joint_position_measured[0],
                                (double)msg->joint_position_measured[1],
                                (double)msg->joint_position_measured[2],
                                (double)msg->joint_position_measured[3],
                                (double)msg->joint_position_measured[4],
                                (double)msg->joint_position_measured[5],
                                (double)msg->joint_position_measured[6];
            messageReceived = true;
        }

        bool messageReceived;
        Eigen::VectorXd initialPositions = Eigen::VectorXd(7);
};

// This function creates a controller for a Kuka LBR Iiwa arm by connecting an
// EndEffectorPositionController to an EndEffectorVelocityController to control
// the individual joint torques as to move the endeffector
// to a desired position.

int do_main(int argc, char* argv[]) {
  //Loads in joint gains json file
  std::ifstream joint_gains_file("examples/kuka_iiwa_arm/simulation_settings.json");
  if (joint_gains_file.is_open()) {
    std::cout << "Json file opened successfully." << std::endl;
  }
  //Initializes joint_gains json object
  json joint_gains = json::parse(joint_gains_file);

  //Kp and 'Rotational' Kp
  const double K_P = joint_gains["kuka_gains"]["K_P"];
  const double K_R = joint_gains["kuka_gains"]["K_R"];

  // Kd and 'Rotational' Kd
  const double K_D = joint_gains["kuka_gains"]["K_D"];
  const double K_OMEGA = joint_gains["kuka_gains"]["K_OMEGA"];

  //Safety Limits
  const double MAX_LINEAR_VEL = joint_gains["limits"]["max_linear_velocity"];
  const double MAX_ANGULAR_VEL = joint_gains["limits"]["max_angular_veloctiy"];
  const double JOINT_TORQUE_LIMIT = joint_gains["limits"]["joint_torque_limit"];

  std::cout << "Using following parameters:" << std::endl;
  std::cout << "K_P: " << K_P << " | K_R: " << K_R << " | K_D: " << K_D;
  std::cout << " | K_OMEGA: " << K_OMEGA << std::endl;
  std::cout << "Linear Velocity Limit: " << MAX_LINEAR_VEL << std::endl;
  std::cout << "Angular Velocity Limit: " << MAX_ANGULAR_VEL << std::endl;
  std::cout << "Joint Torque Limit: " << JOINT_TORQUE_LIMIT << std::endl;

  // Initialize Kuka model URDF-- from Drake kuka simulation files
  std::string kModelPath = "../drake/manipulation/models/iiwa_description"
                           "/iiwa7/iiwa7_no_collision.sdf";
  const std::string urdf_string = FindResourceOrThrow(kModelPath);

  // MultibodyPlants are created here, then passed by reference
  // to the controller blocks for internal modelling.
  const auto X_WI = drake::math::RigidTransform<double>::Identity();
  std::unique_ptr<MultibodyPlant<double>> owned_plant =
      std::make_unique<MultibodyPlant<double>>();

  drake::multibody::Parser plant_parser(owned_plant.get());
  plant_parser.AddModelFromFile(urdf_string, "iiwa");
  owned_plant->WeldFrames(owned_plant->world_frame(),
                          owned_plant->GetFrameByName("iiwa_link_0"), X_WI);
  owned_plant->Finalize();

  drake::systems::DiagramBuilder<double> builder;

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Adding status subscriber and receiver blocks
  auto status_subscriber = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm));
  auto status_receiver = builder.AddSystem<
      drake::examples::kuka_iiwa_arm::IiwaStatusReceiver>();

  // The coordinates for the end effector with respect to the last joint,
  // used to determine location of end effector
  Eigen::Vector3d eeContactFrame;
  eeContactFrame << 0.0, 0, 0.09;

  const std::string link_7 = "iiwa_link_7";

  // Adding position controller block
  auto position_controller = builder.AddSystem<
      systems::EndEffectorPositionController>(
          *owned_plant, link_7, eeContactFrame, K_P, K_R, MAX_LINEAR_VEL,
          MAX_ANGULAR_VEL);

  // Adding Velocity Controller block
  auto velocity_controller = builder.AddSystem<
      systems::EndEffectorVelocityController>(
          *owned_plant, link_7, eeContactFrame, K_D, K_OMEGA, JOINT_TORQUE_LIMIT);

  //Processes Trajectories CSV file.
  CsvVector waypoints("examples/kuka_iiwa_arm/Trajectories.csv");

  // Listens for LCM message then performs invKin to get traj initial position.
  lcm::LCM lcm_;
  if (!lcm_.good()) return 1;

  Eigen::Vector3d x_initial;
  InitialPosHandler handler;
  lcm_.subscribe("IIWA_STATUS", &InitialPosHandler::handleMessage, &handler);
  while (lcm_.handle() == 0 && !handler.messageReceived);

  const std::unique_ptr<Context<double>> plant_context =
      owned_plant->CreateDefaultContext();
  owned_plant->SetPositions(plant_context.get(), handler.initialPositions);
  owned_plant->CalcPointsPositions(*plant_context, owned_plant->GetFrameByName(link_7),
                             eeContactFrame, owned_plant->world_frame(), &x_initial);

  Eigen::Vector3d checkpointTwo;
  checkpointTwo << waypoints.getArray()[1][0], waypoints.getArray()[2][0],
                   waypoints.getArray()[3][0];

  // 1/second m/s movement speed
  const int seconds = 20;
  int firstMovementTime = (x_initial - checkpointTwo).norm() * seconds;

  //Initializes trajectories to trajectoryVectors array.
  std::vector<Eigen::MatrixXd> trajectoryVectors;
  if (firstMovementTime > 0.0){
      trajectoryVectors.push_back(x_initial);
  }
  for (unsigned int x = 0; x < waypoints.getArray()[0].size(); x++) {
    Eigen::Vector3d temp;
    temp << waypoints.getArray()[1][x], waypoints.getArray()[2][x],
            waypoints.getArray()[3][x];
    trajectoryVectors.push_back(temp);
  }

  std::vector<double> timeKnots = waypoints.getArray()[0];
  // Add the movement time to every timestep
  if (firstMovementTime > 0.0) {
      for (double& d : timeKnots) d += abs(firstMovementTime);
      std::cout << firstMovementTime << std::endl;
      timeKnots.insert(timeKnots.begin(), 0.0);
  }
  auto ee_trajectory =
      drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
          timeKnots, trajectoryVectors);
  auto ee_velocity = ee_trajectory.derivative(1);

  // Processes EndEffectorOrientations CSV file.
  CsvVector orientations("examples/kuka_iiwa_arm/EndEffectorOrientations.csv");

  //Initializes orientations to orient_points array.
  std::vector<Eigen::MatrixXd> orient_points;
  for (int y = 0; y < orientations.knotSize(); y++) {
    Eigen::Vector4d aPoint;
    aPoint << orientations.getArray()[1][y], orientations.getArray()[2][y],
              orientations.getArray()[3][y], orientations.getArray()[4][y];
    orient_points.push_back(aPoint);
  }

  auto orientation_trajectory =
      drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
          orientations.getArray()[0], orient_points);

  // Adding linear position Trajectory Source
  auto input_trajectory = builder.AddSystem<drake::systems::TrajectorySource>(
      ee_trajectory);
  auto input_velocity = builder.AddSystem<drake::systems::TrajectorySource>(
      ee_velocity);
  // Adding orientation Trajectory Source
  auto input_orientation = builder.AddSystem<drake::systems::TrajectorySource>(
      orientation_trajectory);


  // Adding command publisher and broadcaster blocks
  auto command_sender = builder.AddSystem<
      drake::examples::kuka_iiwa_arm::IiwaCommandSender>();
  auto command_publisher = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm, 1.0/200.0));

  // Torque Controller-- includes virtual springs and damping.
  VectorXd ConstPositionCommand;

  // The virtual spring stiffness in Nm/rad.
  ConstPositionCommand.resize(7);
  ConstPositionCommand << 0, 0, 0, 0, 0, 0, 0;

  auto positionCommand = builder.AddSystem<
      drake::systems::ConstantVectorSource>(ConstPositionCommand);

  builder.Connect(status_subscriber->get_output_port(),
                  status_receiver->get_input_port());
  builder.Connect(status_receiver->get_position_measured_output_port(),
                  velocity_controller->get_joint_pos_input_port());
  builder.Connect(status_receiver->get_velocity_estimated_output_port(),
                  velocity_controller->get_joint_vel_input_port());
  //Connecting q input from status receiver to position controller
  builder.Connect(status_receiver->get_position_measured_output_port(),
                  position_controller->get_joint_pos_input_port());
  //Connecting x_desired input from trajectory to position controller
  builder.Connect(input_trajectory->get_output_port(),
                  position_controller->get_endpoint_pos_input_port());
  builder.Connect(input_velocity->get_output_port(),
                  position_controller->get_endpoint_vel_input_port());
  //Connecting desired orientation to position controller
  builder.Connect(input_orientation->get_output_port(),
                  position_controller->get_endpoint_orient_input_port());
  //Connecting position (twist) controller to trajectory/velocity controller;
  builder.Connect(position_controller->get_endpoint_cmd_output_port(),
                  velocity_controller->get_endpoint_twist_input_port());

  builder.Connect(velocity_controller->get_endpoint_torque_output_port(),
                  command_sender->get_torque_input_port());

  builder.Connect(status_receiver->get_position_measured_output_port(),
                  command_sender->get_position_input_port());
  builder.Connect(command_sender->get_output_port(),
                  command_publisher->get_input_port());

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  simulator.AdvanceTo(ee_trajectory.end_time());
  return 0;
}

} // Namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
