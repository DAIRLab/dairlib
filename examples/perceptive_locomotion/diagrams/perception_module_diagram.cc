#include "perception_module_diagram.h"

// dairlib
#include "systems/perception/pointcloud/drake_to_pcl_pointcloud.h"
#include "systems/perception/pointcloud/voxel_grid_filter.h"
#include "systems/perception/elevation_mapping_system.h"
#include "systems/perception/perceptive_locomotion_preprocessor.h"
#include "systems/primitives/pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/networking/cassie_output_receiver.h"
#include "examples/Cassie/cassie_state_estimator.h"
#include "examples/Cassie/systems/sim_cassie_sensor_aggregator.h"
#include "multibody/multibody_utils.h"

// drake
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/perception/depth_image_to_point_cloud.h"

namespace dairlib {
namespace perceptive_locomotion {

using Eigen::Matrix3d;
using Eigen::Vector3d;

using pcl::PointXYZRGBConfidenceRatio;
using elevation_mapping::SensorProcessorBase;

using multibody::DistanceEvaluator;
using multibody::WorldPointEvaluator;
using multibody::KinematicEvaluatorSet;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

using perception::elevation_mapping_params_io;
using perception::perceptive_locomotion_preprocessor_params;
using perception::PerceptiveLocomotionPreprocessor;
using perception::DrakeToPclPointCloud;

using drake::systems::BasicVector;
using drake::systems::AbstractStateIndex;
using drake::systems::ConstantVectorSource;

PerceptionModuleDiagram::PerceptionModuleDiagram(
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
    std::string elevation_mapping_params_yaml_path,
    std::map<std::string, drake::systems::sensors::CameraInfo> depth_sensor_info,
    std::string joint_offsets_yaml) :
    plant_(std::move(plant)),
    plant_context_(plant_->CreateDefaultContext()),
    fourbar_(*plant_),
    left_contact_(*plant_),
    right_contact_(*plant_),
    left_loop_(LeftLoopClosureEvaluator(*plant_)),
    right_loop_(RightLoopClosureEvaluator(*plant_)),
    left_toe_evaluator_(*plant_, LeftToeFront(*plant_).first,
                        LeftToeFront(*plant_).second, Matrix3d::Identity(),
                        Vector3d::Zero(), {1, 2}),
    left_heel_evaluator_(*plant_, LeftToeRear(*plant_).first,
                         LeftToeRear(*plant_).second, Matrix3d::Identity(),
                         Vector3d::Zero(), {0, 1, 2}),
    right_toe_evaluator_(*plant_, RightToeFront(*plant_).first,
                        RightToeFront(*plant_).second, Matrix3d::Identity(),
                        Vector3d::Zero(), {1, 2}),
    right_heel_evaluator_(*plant_, RightToeRear(*plant_).first,
                         RightToeRear(*plant_).second, Matrix3d::Identity(),
                         Vector3d::Zero(), {0, 1, 2}) {

  fourbar_.add_evaluator(&left_loop_);
  fourbar_.add_evaluator(&right_loop_);
  right_contact_.add_evaluator(&right_toe_evaluator_);
  right_contact_.add_evaluator(&right_heel_evaluator_);
  left_contact_.add_evaluator(&left_toe_evaluator_);
  left_contact_.add_evaluator(&right_toe_evaluator_);

  drake::systems::DiagramBuilder<double> builder;

  // output receiver
  auto output_receiver = builder.AddSystem<systems::CassieOutputReceiver>();

  // 2000 Hz update rate with a 3 ms delay
  auto delay = builder.AddSystem<drake::systems::DiscreteTimeDelay<double>>(
        ekf_update_period_, communication_delay_periods_, drake::Value<lcmt_cassie_out>()
  );
  delay->set_name("communication_delay");

  // state estimator
  // TODO: Add option to set joint offsets in state estimator
  state_estimator_ = builder.AddSystem<systems::CassieStateEstimator>(
      *plant_, &fourbar_, &left_contact_, &right_contact_, joint_offsets_,
      false, false, 2
  );
  state_estimator_->MakeDrivenBySimulator(ekf_update_period_);

  // robot output sender
  auto robot_output_sender =
      builder.AddSystem<systems::RobotOutputSender>(*plant_, true, true);

  // passthroughs
  auto state_passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
      state_estimator_->get_robot_output_port().size(), 0,
      robot_output_sender->get_input_port_state().size());

  // Passthrough to pass efforts
  auto effort_passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
      state_estimator_->get_robot_output_port().size(),
      robot_output_sender->get_input_port_state().size(),
      robot_output_sender->get_input_port_effort().size());

  auto imu_passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
      state_estimator_->get_robot_output_port().size(),
      robot_output_sender->get_input_port_state().size() +
          robot_output_sender->get_input_port_effort().size(),
      robot_output_sender->get_input_port_imu().size());

  // elevation mapping
  auto mapping_params =
      elevation_mapping_params_io::ReadElevationMappingParamsFromYaml(
          elevation_mapping_params_yaml_path
      );
  auto elevation_mapping_system =
      builder.AddSystem<perception::ElevationMappingSystem>(
      *plant_, plant_context_.get(), mapping_params
  );

  // configure depth sensors
  DRAKE_DEMAND(depth_sensor_info.size() == mapping_params.sensor_poses.size());

  perceptive_locomotion_preprocessor_params preprocessor_params {
      "examples/perceptive_locomotion/camera_calib/d455_noise_model.yaml",
      {
          {"toe_left", Vector3d(0.3, 0.1, 0.2),
            CassieTransformFootToToeFrame()},
          {"tarsus_left", Vector3d(0.5, 0.2, 0.2),
           drake::math::RigidTransformd(Vector3d(0.204, -0.02, 0))},
          {"toe_right", Vector3d(0.3, 0.1, 0.2),
            CassieTransformFootToToeFrame()},
          {"tarsus_right", Vector3d(0.5, 0.2, 0.2),
           drake::math::RigidTransformd(Vector3d(0.204, -0.02, 0))},
      } // crop boxes
  };
  auto preprocessor = std::make_shared<PerceptiveLocomotionPreprocessor>(
      *plant_, plant_context_.get(), preprocessor_params,
      elevation_mapping::SensorProcessorBase::GeneralParameters{"pelvis", "world"}
  );

  Eigen::MatrixXd base_cov_dummy = 0.1 * Eigen::MatrixXd::Identity(6, 6);
  base_cov_dummy.resize(36,1);

  // TODO (@Brian-Acosta) Why is the "real" covariance problematic?
  auto cov_source = builder.AddSystem<ConstantVectorSource<double>>(
      base_cov_dummy
  );

  for (const auto& sensor_pose : mapping_params.sensor_poses) {
    const auto &sensor_name = sensor_pose.sensor_name_;
    DRAKE_DEMAND(depth_sensor_info.count(sensor_name) == 1);

    elevation_mapping_system->AddSensorPreProcessor(sensor_name, preprocessor);
    auto frame_rate = builder.AddNamedSystem<drake::systems::ZeroOrderHold>(
        "frame_rate_" + sensor_name,
        1.0 / 30.0, drake::Value<drake::systems::sensors::ImageDepth32F>()
    );
    auto to_point_cloud =
        builder.AddNamedSystem<drake::perception::DepthImageToPointCloud>(
            "to_point_cloud_" + sensor_name,
            depth_sensor_info.at(sensor_name)
        );
    auto voxel_filter = builder.AddNamedSystem<perception::VoxelGridFilter>(
        "voxel_filter_" + sensor_name, 0.02
    );
    auto point_cloud_converter =
       builder.AddNamedSystem<DrakeToPclPointCloud<PointXYZRGBConfidenceRatio>>(
          "drake_to_pcl_pc_" + sensor_name
    );
    builder.Connect(
        frame_rate->get_output_port(), to_point_cloud->depth_image_input_port()
    );
    builder.Connect(*to_point_cloud, *voxel_filter);
    builder.Connect(*voxel_filter, *point_cloud_converter);
    builder.Connect(
        point_cloud_converter->get_output_port(),
        elevation_mapping_system->get_input_port_pointcloud(sensor_name)
    );
    input_port_depth_image_.insert({
        sensor_name,
        builder.ExportInput(frame_rate->get_input_port(), sensor_name)
    });
  }

  builder.Connect(*delay, *output_receiver);
  builder.Connect(*output_receiver, *state_estimator_);
  builder.Connect(state_estimator_->get_robot_output_port(),
                  state_passthrough->get_input_port());
  builder.Connect(state_estimator_->get_robot_output_port(),
                  effort_passthrough->get_input_port());
  builder.Connect(state_estimator_->get_robot_output_port(),
                  imu_passthrough->get_input_port());
  builder.Connect(state_passthrough->get_output_port(),
                  robot_output_sender->get_input_port_state());
  builder.Connect(effort_passthrough->get_output_port(),
                  robot_output_sender->get_input_port_effort());
  builder.Connect(imu_passthrough->get_output_port(),
                  robot_output_sender->get_input_port_imu());
  builder.Connect(state_estimator_->get_robot_output_port(),
                  elevation_mapping_system->get_input_port_state());
  builder.Connect(cov_source->get_output_port(),
                  elevation_mapping_system->get_input_port_covariance());

  input_port_cassie_out_ = builder.ExportInput(
      delay->get_input_port(), "lcmt_cassie_out"
  );
  output_port_elevation_map_ = builder.ExportOutput(
      elevation_mapping_system->get_output_port_grid_map(), "elevation_grid_map"
  );
  output_port_robot_output_ = builder.ExportOutput(
      robot_output_sender->get_output_port(), "lcmt_robot_output"
  );
  output_port_state_ = builder.ExportOutput(
      state_estimator_->get_robot_output_port(), "x, u, t"
  );

  set_name("perception_stack");
  builder.BuildInto(this);
}

std::unique_ptr<PerceptionModuleDiagram> PerceptionModuleDiagram::Make(
    std::string elevation_mapping_params_yaml_path,
    std::map<std::string, drake::systems::sensors::CameraInfo> depth_sensor_info,
    std::string joint_offsets_yaml) {
  const std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  auto plant = std::make_unique<drake::multibody::MultibodyPlant<double>>(0);
  AddCassieMultibody(plant.get(), nullptr, true, urdf, true, false);
  plant->Finalize();
  return std::make_unique<PerceptionModuleDiagram>(
    std::move(plant), elevation_mapping_params_yaml_path,
    depth_sensor_info, joint_offsets_yaml
  );
}

void PerceptionModuleDiagram::InitializeEkf(
    drake::systems::Context<double> *root_context,
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro) const {

  auto x = BasicVector<double>(
      plant_->num_positions() + plant_->num_velocities()
  );
  x.get_mutable_value().head(plant_->num_positions()) = q;
  x.get_mutable_value().tail(plant_->num_velocities()) = v;

  // fill out the cassie_out message with the relevant parameters
  lcmt_cassie_out cassie_out_message;
  systems::SimCassieSensorAggregator::CopyJointStates(
      MakeNameToPositionsMap(*plant_),
      MakeNameToVelocitiesMap(*plant_),
      &cassie_out_message,
      &x
  );

  for (int i = 0; i < 3; ++i) {
    cassie_out_message.pelvis.vectorNav.angularVelocity[i] = gyro(i);
    cassie_out_message.pelvis.vectorNav.linearAcceleration[i] = accel(i);
  }

  auto& delay_system =
      dynamic_cast<const drake::systems::DiscreteTimeDelay<double>&>(
          GetSubsystemByName("communication_delay")
      );
  auto& delay_context = delay_system.GetMyMutableContextFromRoot(root_context);

  for (AbstractStateIndex i{0}; i < communication_delay_periods_; ++i) {
    delay_context.SetAbstractState<lcmt_cassie_out>(i, cassie_out_message);
  }

  auto& state_estimator_context = state_estimator_->GetMyMutableContextFromRoot(
      root_context
  );
  state_estimator_->setPreviousTime(
      &state_estimator_context, root_context->get_time()
  );
  state_estimator_->setInitialPelvisPose(
      &state_estimator_context, q.head<4>(), q.segment<3>(4), v.segment<3>(3)
  );
  Eigen::VectorXd init_imu = Eigen::VectorXd::Zero(6);
  init_imu.head<3>() = gyro;
  init_imu.tail<3>() = accel;
  state_estimator_->setPreviousImuMeasurement(
      &state_estimator_context, init_imu
  );
}

void PerceptionModuleDiagram::InitializeEkf(
    drake::systems::Context<double> *root_context,
    const Eigen::VectorXd &q, const Eigen::VectorXd &v) const {
  plant_->SetPositions(plant_context_.get(), q);
  auto pose = plant_->GetBodyByName("pelvis").EvalPoseInWorld(*plant_context_);
  Vector3d accel(0, 0, 9.81);
  Vector3d omega = v.head<3>();
  accel = pose.rotation().inverse() * accel;
  omega = pose.rotation().inverse() * omega;
  InitializeEkf(root_context, q, v, accel, omega);
}

}
}