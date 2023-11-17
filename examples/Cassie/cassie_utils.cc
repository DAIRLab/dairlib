#include <iostream>
#include "examples/Cassie/cassie_utils.h"
#include "common/find_resource.h"

#include "examples/Cassie/systems/cassie_encoder.h"
#include "multibody/multibody_solvers.h"
#include "drake/geometry/scene_graph.h"

#include "drake/solvers/solve.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/linear_spring_damper.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/systems/sensors/accelerometer.h"
#include "drake/systems/sensors/gyroscope.h"

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

drake::math::RigidTransformd CassieTransformFootToToeFrame() {
  Vector3d toe(-0.0457, 0.112, 0);
  Vector3d heel(0.088, 0, 0);
  Vector3d x = (toe - heel).normalized();
  Vector3d y = -Vector3d::UnitZ();
  Vector3d z = x.cross(y).normalized();
  Vector3d center = 0.5 * (toe + heel);
  return {
    drake::math::RotationMatrixd::MakeFromOrthonormalColumns(x, y, z), center
  };
}

/// Add a fixed base cassie to the given multibody plant and scene graph
/// These methods are to be used rather that direct construction of the plant
/// from the URDF to centralize any modeling changes or additions
const drake::multibody::ModelInstanceIndex
AddCassieMultibody(MultibodyPlant<double>* plant,
                   SceneGraph<double>* scene_graph, bool floating_base,
                  std::string filename, bool add_leaf_springs,
                  bool add_loop_closure, bool add_reflected_inertia) {
  std::string full_name = FindResourceOrThrow(filename);
  Parser parser(plant, scene_graph);
  auto model_instance_idx = parser.AddModelFromFile(full_name);

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

    if (plant->get_discrete_contact_solver() ==
        drake::multibody::DiscreteContactSolver::kSap) {
      plant->AddDistanceConstraint(
          heel_spring_left, rod_on_heel_spring, thigh_left, rod_on_thigh_left,
          kCassieAchillesLength, achilles_stiffness, achilles_damping);
      plant->AddDistanceConstraint(heel_spring_right, rod_on_heel_spring,
                                   thigh_right, rod_on_thigh_right,
                                   kCassieAchillesLength, achilles_stiffness,
                                   achilles_damping);
    } else {
      plant->AddForceElement<drake::multibody::LinearSpringDamper>(
          heel_spring_left, rod_on_heel_spring, thigh_left, rod_on_thigh_left,
          kCassieAchillesLength, achilles_stiffness, achilles_damping);

      plant->AddForceElement<drake::multibody::LinearSpringDamper>(
          heel_spring_right, rod_on_heel_spring, thigh_right,
          rod_on_thigh_right, kCassieAchillesLength, achilles_stiffness,
          achilles_damping);
    }
  }

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
  if (add_reflected_inertia) {
    for (int i = 0; i < rotor_inertias.size(); ++i) {
      auto& joint_actuator = plant->get_mutable_joint_actuator(
          drake::multibody::JointActuatorIndex(i));
      joint_actuator.set_default_rotor_inertia(rotor_inertias(i));
      joint_actuator.set_default_gear_ratio(gear_ratios(i));
      DRAKE_DEMAND(motor_joint_names[i] == joint_actuator.name());
    }
  }
  return model_instance_idx;
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

  std::vector<int> joint_pos_indices;
  std::vector<int> joint_vel_indices;
  std::vector<int> ticks_per_revolution;

  const auto& encoders = builder->AddSystem<CassieEncoder>(plant);

  auto sensor_aggregator =
      builder->AddSystem<systems::SimCassieSensorAggregator>(plant);
  builder->Connect(actuation_port, sensor_aggregator->get_input_port_input());
  builder->Connect(plant.get_state_output_port(), encoders->get_input_port());
  builder->Connect(encoders->get_output_port(),
                   sensor_aggregator->get_input_port_state());
  builder->Connect(accelerometer.get_measurement_output_port(),
                   sensor_aggregator->get_input_port_acce());
  builder->Connect(gyroscope.get_measurement_output_port(),
                   sensor_aggregator->get_input_port_gyro());
  return *sensor_aggregator;
}

const systems::GearedMotor& AddMotorModel(
    drake::systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant) {
  std::unordered_map<std::string, double> omega_max = {
      {"hip_roll_left_motor", 303.687},  {"hip_roll_right_motor", 303.687},
      {"hip_yaw_left_motor", 303.687},   {"hip_yaw_right_motor", 303.687},
      {"hip_pitch_left_motor", 136.136}, {"hip_pitch_right_motor", 136.136},
      {"knee_left_motor", 136.136},      {"knee_right_motor", 136.136},
      {"toe_left_motor", 575.958},       {"toe_right_motor", 575.958}};
  auto cassie_motor =
      builder->AddSystem<systems::GearedMotor>(plant, omega_max);
  return *cassie_motor;
}

VectorXd SolveFourBarIK(const MultibodyPlant<double>& plant,
                        const VectorXd& q_nominal) {
  auto prog = multibody::MultibodyProgram<double>(plant);
  auto q = prog.AddPositionVariables();
  auto fourbar_right = RightLoopClosureEvaluator(plant);
  auto fourbar_left = LeftLoopClosureEvaluator(plant);
  auto left_right_loops = multibody::KinematicEvaluatorSet<double>(plant);
  left_right_loops.add_evaluator(&fourbar_left);
  left_right_loops.add_evaluator(&fourbar_right);
  prog.AddKinematicPositionConstraint(left_right_loops, q);
  prog.AddQuadraticErrorCost(
      Eigen::MatrixXd::Identity(plant.num_positions(), plant.num_positions()),
      q_nominal,
      q);
  auto sol = drake::solvers::Solve(prog);
  if (sol.is_success()) {
    return sol.GetSolution(q);
  } else {
    std::cerr << "Failed with code " << sol.get_solution_result() << std::endl;
    DRAKE_DEMAND(false);
  }

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
