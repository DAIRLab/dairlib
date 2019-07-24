#include "examples/Cassie/cassie_utils.h"
#include "common/find_resource.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"

#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

#include "attic/multibody/multibody_solvers.h"

#include "examples/Cassie/cassie_utils.h"

namespace dairlib {

using Eigen::VectorXd;
using Eigen::Vector3d;
using drake::solvers::Constraint;
using drake::AutoDiffVecXd;
using drake::solvers::MathematicalProgram;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::multibody::RevoluteSpring;
using drake::multibody::joints::FloatingBaseType;

using std::map;
using std::string;
using std::vector;
using std::unique_ptr;
using std::make_unique;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using dairlib::multibody::FixedPointSolver;
using dairlib::multibody::PositionSolver;
using dairlib::multibody::ContactInfo;
using dairlib::multibody::GetBodyIndexFromName;


/// Add a fixed base cassie to the given multibody plant and scene graph
/// These methods are to be used rather that direct construction of the plant
/// from the URDF to centralize any modeling changes or additions
void addCassieMultibody(MultibodyPlant<double>* plant,
    SceneGraph<double>* scene_graph, bool floating_base, std::string filename) {
  std::string full_name = FindResourceOrThrow(filename);
  Parser parser(plant, scene_graph);
  parser.AddModelFromFile(full_name);

  plant->mutable_gravity_field().set_gravity_vector(
      -9.81 * Eigen::Vector3d::UnitZ());

  if (!floating_base) {
    plant->WeldFrames(
      plant->world_frame(), plant->GetFrameByName("pelvis"),
      drake::math::RigidTransform<double>(Vector3d::Zero()));
  }

  // Add springss
  // stiffness is 2300 in URDF, 1500 from gazebo
  plant->AddForceElement<RevoluteSpring>(
      dynamic_cast<const drake::multibody::RevoluteJoint<double>&>(
          plant->GetJointByName("knee_joint_left")),
      0, 1500);
  plant->AddForceElement<RevoluteSpring>(
      dynamic_cast<const drake::multibody::RevoluteJoint<double>&>(
          plant->GetJointByName("knee_joint_right")),
      0, 1500);
  plant->AddForceElement<RevoluteSpring>(
      dynamic_cast<const drake::multibody::RevoluteJoint<double>&>(
          plant->GetJointByName("ankle_spring_joint_left")),
      0, 1250);
  plant->AddForceElement<RevoluteSpring>(
      dynamic_cast<const drake::multibody::RevoluteJoint<double>&>(
          plant->GetJointByName("ankle_spring_joint_right")),
      0, 1250);

  // TOOO(mposa): add loop closures when implemented in Drake
}

std::unique_ptr<RigidBodyTree<double>> makeCassieTreePointer(
    std::string filename, FloatingBaseType base_type) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  buildCassieTree(*tree.get(), filename, base_type);
  return tree;
}

void buildCassieTree(RigidBodyTree<double>& tree, std::string filename,
                     FloatingBaseType base_type) {
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(filename), base_type, &tree);

  // Add distance constraints for the two legs
  double achilles_length = .5012;
  int heel_spring_left = tree.FindBodyIndex("heel_spring_left");
  int thigh_left = tree.FindBodyIndex("thigh_left");

  int heel_spring_right = tree.FindBodyIndex("heel_spring_right");
  int thigh_right = tree.FindBodyIndex("thigh_right");

  Vector3d rod_on_heel_spring;  // symmetric left and right
  rod_on_heel_spring << .11877, -.01, 0.0;

  Vector3d rod_on_thigh_left;
  rod_on_thigh_left << 0.0, 0.0, 0.045;

  Vector3d rod_on_thigh_right;
  rod_on_thigh_right << 0.0, 0.0, -0.045;

  tree.addDistanceConstraint(heel_spring_left, rod_on_heel_spring, thigh_left,
                             rod_on_thigh_left, achilles_length);

  tree.addDistanceConstraint(heel_spring_right, rod_on_heel_spring, thigh_right,
                             rod_on_thigh_right, achilles_length);

  // Add spring forces
  int body_index = tree.FindIndexOfChildBodyOfJoint("knee_joint_left");
  auto body = tree.get_mutable_body(body_index);
  RevoluteJoint& knee_joint_left =
      dynamic_cast<RevoluteJoint&>(body->get_mutable_joint());
  // stiffness is 2300 in URDF,these #s from gazebo
  knee_joint_left.SetSpringDynamics(1500.0, 0.0);

  body_index = tree.FindIndexOfChildBodyOfJoint("knee_joint_right");
  body = tree.get_mutable_body(body_index);
  RevoluteJoint& knee_joint_right =
      dynamic_cast<RevoluteJoint&>(body->get_mutable_joint());
  knee_joint_right.SetSpringDynamics(1500.0, 0.0);  // 2300 in URDF

  body_index = tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_left");
  body = tree.get_mutable_body(body_index);
  RevoluteJoint& ankle_spring_joint_left =
      dynamic_cast<RevoluteJoint&>(body->get_mutable_joint());
  ankle_spring_joint_left.SetSpringDynamics(1250.0, 0.0);  // 2000 in URDF

  body_index = tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_right");
  body = tree.get_mutable_body(body_index);
  RevoluteJoint& ankle_spring_joint_right =
      dynamic_cast<RevoluteJoint&>(body->get_mutable_joint());
  ankle_spring_joint_right.SetSpringDynamics(1250.0, 0.0);  // 2000 in URDF
}

void addImuFrameToCassiePelvis(std::unique_ptr<RigidBodyTree<double>> & tree){
  // IMU position
  // source: https://github.com/osudrl/cassie-mujoco-sim/blob/master/model/cassie.xml#L86
  Eigen::Isometry3d Imu_pos_wrt_pelvis_origin;
  Imu_pos_wrt_pelvis_origin.linear() = Eigen::Matrix3d::Identity();;
  Imu_pos_wrt_pelvis_origin.translation() =
      Eigen::Vector3d(0.03155, 0, -0.07996);

  std::shared_ptr<RigidBodyFrame<double>> imu_frame =
           std::allocate_shared<RigidBodyFrame<double>>(
               Eigen::aligned_allocator<RigidBodyFrame<double>>(),
               "imu frame",
               tree->FindBody("pelvis"),
               Imu_pos_wrt_pelvis_origin);
  tree->addFrame(imu_frame);
}
drake::systems::sensors::Accelerometer * addSimAccelerometer(
    drake::systems::DiagramBuilder<double> & builder,
    drake::systems::RigidBodyPlant<double> * plant) {

  std::shared_ptr< RigidBodyFrame<double> > imu_frame_ptr =
      plant->get_rigid_body_tree().findFrame("imu frame");

  auto accel_sim = builder.AddSystem<drake::systems::sensors::Accelerometer>(
                    "Simulated Accelerometer", *imu_frame_ptr,
                    plant->get_rigid_body_tree(), true);
  builder.Connect(plant->state_output_port(),
                  accel_sim->get_plant_state_input_port());
  builder.Connect(plant->state_derivative_output_port(),
                  accel_sim->get_plant_state_derivative_input_port());

  return accel_sim;
}
drake::systems::sensors::Gyroscope * addSimGyroscope(
    drake::systems::DiagramBuilder<double> & builder,
    drake::systems::RigidBodyPlant<double> * plant) {

  std::shared_ptr< RigidBodyFrame<double> > imu_frame_ptr =
      plant->get_rigid_body_tree().findFrame("imu frame");

  auto gyro_sim = builder.AddSystem<drake::systems::sensors::Gyroscope>(
                    "Simulated Gyroscope",
                    *imu_frame_ptr, plant->get_rigid_body_tree());
  builder.Connect(plant->state_output_port(), gyro_sim->get_input_port());

  return gyro_sim;
}
systems::SimCassieSensorAggregator * addSimCassieSensorAggregator(
    drake::systems::DiagramBuilder<double> & builder,
    drake::systems::RigidBodyPlant<double> * plant,
    SubvectorPassThrough<double> * passthrough,
    drake::systems::sensors::Accelerometer * accel_sim,
    drake::systems::sensors::Gyroscope * gyro_sim) {
  auto cassie_sensor_aggregator =
    builder.AddSystem<systems::SimCassieSensorAggregator>(
      plant->get_rigid_body_tree());
  builder.Connect(passthrough->get_output_port(),
                  cassie_sensor_aggregator->get_input_port_input());
  builder.Connect(plant->state_output_port(),
                  cassie_sensor_aggregator->get_input_port_state());
  builder.Connect(accel_sim->get_output_port(),
                  cassie_sensor_aggregator->get_input_port_acce());
  builder.Connect(gyro_sim->get_output_port(),
                  cassie_sensor_aggregator->get_input_port_gyro());
  return cassie_sensor_aggregator;
}
systems::SimCassieSensorAggregator * addImuAndAggregatorToSimulation(
    drake::systems::DiagramBuilder<double> & builder,
    drake::systems::RigidBodyPlant<double> * plant,
    SubvectorPassThrough<double> * passthrough) {

  auto accel_sim = addSimAccelerometer(builder, plant);
  auto gyro_sim = addSimGyroscope(builder, plant);
  auto cassie_sensor_aggregator = addSimCassieSensorAggregator(
                                    builder, plant, passthrough,
                                    accel_sim, gyro_sim);

  return cassie_sensor_aggregator;
}

multibody::ContactInfo ComputeCassieContactInfo(const RigidBodyTree<double>& tree,
                                     const VectorXd& q0) {
  VectorXd phi_total;
  Matrix3Xd normal_total, xA_total, xB_total;
  vector<int> idxA_total, idxB_total;
  KinematicsCache<double> k_cache = tree.doKinematics(q0);

  // The full collisionDetect solution.
  const_cast<RigidBodyTree<double>&>(tree).collisionDetect(
      k_cache, phi_total, normal_total, xA_total, xB_total, idxA_total,
      idxB_total);

  const int world_ind = GetBodyIndexFromName(tree, "world");
  const int toe_left_ind = GetBodyIndexFromName(tree, "toe_left");
  const int toe_right_ind = GetBodyIndexFromName(tree, "toe_right");

  // Extracting information into the four contacts.
  VectorXd phi(4);
  Matrix3Xd normal(3, 4), xA(3, 4), xB(3, 4);
  vector<int> idxA(4), idxB(4);

  int k = 0;
  for (unsigned i = 0; i < idxA_total.size(); ++i) {
    int ind_a = idxA_total.at(i);
    int ind_b = idxB_total.at(i);
    if ((ind_a == world_ind && ind_b == toe_left_ind) ||
        (ind_a == world_ind && ind_b == toe_right_ind) ||
        (ind_a == toe_left_ind && ind_b == world_ind) ||
        (ind_a == toe_right_ind && ind_b == world_ind)) {
      xA.col(k) = xA_total.col(i);
      xB.col(k) = xB_total.col(i);
      idxA.at(k) = idxA_total.at(i);
      idxB.at(k) = idxB_total.at(i);
      ++k;
    }
  }

  ContactInfo contact_info = {xB, idxB};
  //std::cout<<xB<<std::endl;
  //for (auto i : idxB)
  //   std::cout<<i<<" ";
  //   std::cout << std::endl;
  return contact_info;
}

std::string getVelocityName(const RigidBodyTree<double>& tree, int index) {
  return tree.get_velocity_name(index);
}

std::string getPositionName(const RigidBodyTree<double>& tree, int index) {
  return tree.get_position_name(index);
}

} // namespace dairlib
