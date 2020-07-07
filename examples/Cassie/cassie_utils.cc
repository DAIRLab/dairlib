#include "examples/Cassie/cassie_utils.h"
#include "common/find_resource.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/linear_spring_damper.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/solvers/mathematical_program.h"

#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/tree/revolute_spring.h"

namespace dairlib {

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::geometry::SceneGraph;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RevoluteSpring;
using drake::multibody::joints::FloatingBaseType;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using Eigen::Vector3d;
using Eigen::VectorXd;

template <typename T>
std::pair<const Vector3d, const Frame<T>&> LeftToeFront(
    const MultibodyPlant<T>& plant) {
  return std::pair<const Vector3d, const Frame<T>&>(
    Vector3d(-0.0457, 0.112, 0), plant.GetFrameByName("toe_left"));
}

template <typename T>
std::pair<const Vector3d, const Frame<T>&> RightToeFront(
    const MultibodyPlant<T>& plant) {
  return std::pair<const Vector3d, const Frame<T>&>(
    Vector3d(-0.0457, 0.112, 0), plant.GetFrameByName("toe_right"));
}

template <typename T>
std::pair<const Vector3d, const Frame<T>&> LeftToeRear(
    const MultibodyPlant<T>& plant) {
  return std::pair<const Vector3d, const Frame<T>&>(
    Vector3d(0.088, 0, 0), plant.GetFrameByName("toe_left"));
}

template <typename T>
std::pair<const Vector3d, const Frame<T>&> RightToeRear(
    const MultibodyPlant<T>& plant) {
  return std::pair<const Vector3d, const Frame<T>&>(
    Vector3d(0.088, 0, 0), plant.GetFrameByName("toe_right"));
}

template <typename T>
std::pair<const Vector3d, const Frame<T>&> LeftRodOnThigh(
    const drake::multibody::MultibodyPlant<T>& plant) {
  return std::pair<const Vector3d, const Frame<T>&>(
    Vector3d(0.0, 0.0, 0.045), plant.GetFrameByName("thigh_left"));
}

template <typename T>
std::pair<const Vector3d, const Frame<T>&> RightRodOnThigh(
    const drake::multibody::MultibodyPlant<T>& plant) {
  return std::pair<const Vector3d, const Frame<T>&>(
    Vector3d(0.0, 0.0, -0.045), plant.GetFrameByName("thigh_right"));
}

template <typename T>
std::pair<const Vector3d, const Frame<T>&> LeftRodOnHeel(
    const drake::multibody::MultibodyPlant<T>& plant) {
  return std::pair<const Vector3d, const Frame<T>&>(
    Vector3d(.11877, -.01, 0.0), plant.GetFrameByName("heel_spring_left"));
}

template <typename T>
std::pair<const Vector3d, const Frame<T>&> RightRodOnHeel(
    const drake::multibody::MultibodyPlant<T>& plant) {
  return std::pair<const Vector3d, const Frame<T>&>(
    Vector3d(.11877, -.01, 0.0), plant.GetFrameByName("heel_spring_right"));
}

template <typename T>
multibody::DistanceEvaluator<T> LeftLoopClosureEvaluator(
    const MultibodyPlant<T>& plant) {
  auto rod_on_thigh = LeftRodOnThigh(plant);
  auto rod_on_heel = LeftRodOnHeel(plant);
  return multibody::DistanceEvaluator<T>(
      plant, rod_on_heel.first, rod_on_heel.second, rod_on_thigh.first,
      rod_on_thigh.second, kCassieAchillesLength);
}

template <typename T>
multibody::DistanceEvaluator<T> RightLoopClosureEvaluator(
    const MultibodyPlant<T>& plant) {
  auto rod_on_thigh = RightRodOnThigh(plant);
  auto rod_on_heel = RightRodOnHeel(plant);
  return multibody::DistanceEvaluator<T>(
      plant, rod_on_heel.first, rod_on_heel.second, rod_on_thigh.first,
      rod_on_thigh.second, kCassieAchillesLength);
}

/// Add a fixed base cassie to the given multibody plant and scene graph
/// These methods are to be used rather that direct construction of the plant
/// from the URDF to centralize any modeling changes or additions
void addCassieMultibody(MultibodyPlant<double>* plant,
                        SceneGraph<double>* scene_graph, bool floating_base,
                        std::string filename, bool add_leaf_springs,
                        bool add_loop_closure) {
  std::string full_name = FindResourceOrThrow(filename);
  Parser parser(plant, scene_graph);
  parser.AddModelFromFile(full_name);

  plant->mutable_gravity_field().set_gravity_vector(-9.81 * Vector3d::UnitZ());

  if (!floating_base) {
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("pelvis"),
                      drake::math::RigidTransform<double>(Vector3d::Zero()));
  }

  if (add_leaf_springs) {
    // Add springs
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
  }

  if (add_loop_closure) {
    // TOOO(mposa): add loop closures when implemented in Drake
    // Add a spring to represent loop closure
    double achilles_stiffness = 2e5;
    double achilles_damping = 2e3;
    const auto& heel_spring_left = LeftRodOnHeel(*plant).second.body();
    const auto& thigh_left = LeftRodOnThigh(*plant).second.body();
    const auto& heel_spring_right = RightRodOnHeel(*plant).second.body();
    const auto& thigh_right = RightRodOnThigh(*plant).second.body();

    // symmetric left and right for heel
    Vector3d rod_on_heel_spring = LeftRodOnHeel(*plant).first;
    Vector3d rod_on_thigh_left = LeftRodOnThigh(*plant).first;
    Vector3d rod_on_thigh_right = RightRodOnThigh(*plant).first;

    plant->AddForceElement<drake::multibody::LinearSpringDamper>(
        heel_spring_left, rod_on_heel_spring, thigh_left, rod_on_thigh_left,
        kCassieAchillesLength, achilles_stiffness, achilles_damping);

    plant->AddForceElement<drake::multibody::LinearSpringDamper>(
        heel_spring_right, rod_on_heel_spring, thigh_right, rod_on_thigh_right,
        kCassieAchillesLength, achilles_stiffness, achilles_damping);
  }
}

std::unique_ptr<RigidBodyTree<double>> makeCassieTreePointer(
    std::string filename, FloatingBaseType base_type, bool is_with_springs) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  buildCassieTree(*tree.get(), filename, base_type, is_with_springs);
  return tree;
}

void buildCassieTree(RigidBodyTree<double>& tree, std::string filename,
                     FloatingBaseType base_type, bool is_with_springs) {
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(filename), base_type, &tree);

  // Add distance constraints for the two legs
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
                             rod_on_thigh_left, kCassieAchillesLength);

  tree.addDistanceConstraint(heel_spring_right, rod_on_heel_spring, thigh_right,
                             rod_on_thigh_right, kCassieAchillesLength);

  // Add spring forces
  if(is_with_springs){
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
    ankle_spring_joint_right.SetSpringDynamics(1250.0, 0.0);  // 2300 in URDF
  }
}

void addImuFrameToCassiePelvis(std::unique_ptr<RigidBodyTree<double>> & tree){
  // IMU position
  // source: https://github.com/osudrl/cassie-mujoco-sim/blob/master/model/cassie.xml#L86
  Eigen::Isometry3d Imu_pos_wrt_pelvis_origin;
  Imu_pos_wrt_pelvis_origin.linear() = Eigen::Matrix3d::Identity();;
  Imu_pos_wrt_pelvis_origin.translation() =
      Vector3d(0.03155, 0, -0.07996);

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

template std::pair<const Vector3d, const Frame<double>&> LeftToeFront(const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> RightToeFront(const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> LeftToeRear(const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> RightToeRear(const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> LeftToeFront(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> RightToeFront(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> LeftToeRear(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> RightToeRear(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> LeftRodOnThigh(const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> RightRodOnThigh(const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> LeftRodOnHeel(const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> RightRodOnHeel(const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> LeftRodOnThigh(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> RightRodOnThigh(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> LeftRodOnHeel(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> RightRodOnHeel(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template multibody::DistanceEvaluator<double> LeftLoopClosureEvaluator(const MultibodyPlant<double>& plant);  // NOLINT
template multibody::DistanceEvaluator<double> RightLoopClosureEvaluator(const MultibodyPlant<double>& plant);  // NOLINT
template multibody::DistanceEvaluator<AutoDiffXd> LeftLoopClosureEvaluator(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template multibody::DistanceEvaluator<AutoDiffXd> RightLoopClosureEvaluator(const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
} // namespace dairlib
