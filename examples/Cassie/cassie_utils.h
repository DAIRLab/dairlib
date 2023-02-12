#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/find_resource.h"
#include "examples/Cassie/systems/sim_cassie_sensor_aggregator.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "systems/framework/geared_motor.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace dairlib {

static constexpr double kCassieAchillesLength = 0.5012;

template <typename T>
std::pair<const Eigen::Vector3d, const drake::multibody::Frame<T>&>
LeftToeFront(const drake::multibody::MultibodyPlant<T>& plant);

template <typename T>
std::pair<const Eigen::Vector3d, const drake::multibody::Frame<T>&>
RightToeFront(const drake::multibody::MultibodyPlant<T>& plant);

template <typename T>
std::pair<const Eigen::Vector3d, const drake::multibody::Frame<T>&> LeftToeRear(
    const drake::multibody::MultibodyPlant<T>& plant);

template <typename T>
std::pair<const Eigen::Vector3d, const drake::multibody::Frame<T>&>
RightToeRear(const drake::multibody::MultibodyPlant<T>& plant);

template <typename T>
std::pair<const Eigen::Vector3d, const drake::multibody::Frame<T>&>
LeftRodOnThigh(const drake::multibody::MultibodyPlant<T>& plant);

template <typename T>
std::pair<const Eigen::Vector3d, const drake::multibody::Frame<T>&>
RightRodOnThigh(const drake::multibody::MultibodyPlant<T>& plant);

template <typename T>
std::pair<const Eigen::Vector3d, const drake::multibody::Frame<T>&>
LeftRodOnHeel(const drake::multibody::MultibodyPlant<T>& plant);

template <typename T>
std::pair<const Eigen::Vector3d, const drake::multibody::Frame<T>&>
RightRodOnHeel(const drake::multibody::MultibodyPlant<T>& plant);

template <typename T>
multibody::DistanceEvaluator<T> LeftLoopClosureEvaluator(
    const drake::multibody::MultibodyPlant<T>& plant);

template <typename T>
multibody::DistanceEvaluator<T> RightLoopClosureEvaluator(
    const drake::multibody::MultibodyPlant<T>& plant);

/// Given the filename of a yaml of joint position offsets labeled by name, i.e.
/// {'toe_left': 0.02, 'knee_right': .0115}, constructs the vector q_offset,
/// such that the corrected position vector is given by q + q_offset.
/// Any subset of joints can be given, so long as each joint appears at
/// most once.
/// @param filename the filename of the yaml to load. returns zero if empty
/// @param plant the plant for which the offsets are to be applied
Eigen::VectorXd LoadJointPositionOffsetsFromYaml(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& filename);

/// Add a fixed base cassie to the given multibody plant and scene graph
/// These methods are to be used rather that direct construction of the plant
/// from the URDF to centralize any modeling changes or additions
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param filename the URDF or SDF file to use for Cassie
///        omit to use default value
/// @param add_leaf_springs Whether or not to add the 4 leaf springs in the legs
///        Default = true
/// @param add_loop_closure Whether or not to add the loop closure
///        distance constraint via stiff springs. Default = true.
/// @param add_reflected_inertia Whether or not to add reflected inertia.
///        Rotor inertia and gear ratio values are constant values from the
///        model given by Agility Robotics
void AddCassieMultibody(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    bool floating_base = true,
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf",
    bool add_leaf_springs = true, bool add_loop_closure = true,
    bool add_reflected_inertia = true);

/// Add simulated gyroscope and accelerometer along with sensor aggregator,
/// which creates and publishes a simulated lcmt_cassie_out LCM message.
/// @param builder The diagram builder
/// @param plant The Cassie plant
/// @param actuation_input_port The vector-valued output port containing the
/// actuation input "u", which will be aggregated.
const systems::SimCassieSensorAggregator& AddImuAndAggregator(
    drake::systems::DiagramBuilder<double>* builder,
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::OutputPort<double>& actuation_port);

const systems::GearedMotor& AddMotorModel(
    drake::systems::DiagramBuilder<double>* builder,
    const drake::multibody::MultibodyPlant<double>& plant);

}  // namespace dairlib
