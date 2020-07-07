#include "examples/Cassie/cassie_utils.h"

#include "common/find_resource.h"

#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/linear_spring_damper.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/systems/sensors/accelerometer_sensor.h"
#include "drake/systems/sensors/gyroscope_sensor.h"

namespace dairlib {

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::geometry::SceneGraph;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RevoluteSpring;
using drake::systems::sensors::Accelerometer;
using drake::systems::sensors::Gyroscope;
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
    double achilles_stiffness = 1e6;
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

const systems::SimCassieSensorAggregator& AddImuAndAggregator(
    drake::systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    const drake::systems::OutputPort<double>& actuation_port) {
  Vector3d imu_in_pelvis(0.03155, 0, -0.07996);
  const drake::math::RigidTransform<double> X_BS(imu_in_pelvis);
  const auto& body = plant.GetBodyByName("pelvis");

  const auto& gyroscope =
      Gyroscope<double>::AddToDiagram(body, X_BS, plant, builder);

  const auto& accelerometer = Accelerometer<double>::AddToDiagram(
      body, X_BS, plant.gravity_field().gravity_vector(), plant, builder);

  auto sensor_aggregator =
      builder->AddSystem<systems::SimCassieSensorAggregator>(plant);
  builder->Connect(actuation_port,
                   sensor_aggregator->get_input_port_input());
  builder->Connect(plant.get_state_output_port(),
                   sensor_aggregator->get_input_port_state());
  builder->Connect(accelerometer.get_measurement_output_port(),
                   sensor_aggregator->get_input_port_acce());
  builder->Connect(gyroscope.get_measurement_output_port(),
                   sensor_aggregator->get_input_port_gyro());
  return *sensor_aggregator;
}

template std::pair<const Vector3d, const Frame<double>&> LeftToeFront(
    const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> RightToeFront(
    const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> LeftToeRear(
    const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> RightToeRear(
    const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> LeftToeFront(
    const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> RightToeFront(
    const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> LeftToeRear(
    const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> RightToeRear(
    const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> LeftRodOnThigh(
    const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> RightRodOnThigh(
    const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> LeftRodOnHeel(
    const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<double>&> RightRodOnHeel(
    const MultibodyPlant<double>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> LeftRodOnThigh(
    const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> RightRodOnThigh(
    const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> LeftRodOnHeel(
    const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template std::pair<const Vector3d, const Frame<AutoDiffXd>&> RightRodOnHeel(
    const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template multibody::DistanceEvaluator<double> LeftLoopClosureEvaluator(
    const MultibodyPlant<double>& plant);  // NOLINT
template multibody::DistanceEvaluator<double> RightLoopClosureEvaluator(
    const MultibodyPlant<double>& plant);  // NOLINT
template multibody::DistanceEvaluator<AutoDiffXd> LeftLoopClosureEvaluator(
    const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
template multibody::DistanceEvaluator<AutoDiffXd> RightLoopClosureEvaluator(
    const MultibodyPlant<AutoDiffXd>& plant);  // NOLINT
}  // namespace dairlib
