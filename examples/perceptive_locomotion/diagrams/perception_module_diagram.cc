#include "perception_module_diagram.h"

// dairlib
#include "systems/system_utils.h"
#include "systems/perception/pointcloud/drake_to_pcl_pointcloud.h"
#include "systems/perception/pointcloud/voxel_grid_filter.h"
#include "systems/perception/elevation_mapping_system.h"
#include "systems/perception/perceptive_locomotion_preprocessor.h"
#include "systems/primitives/pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "examples/Cassie/networking/cassie_output_receiver.h"
#include "examples/Cassie/cassie_state_estimator.h"

// drake
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/systems/primitives/zero_order_hold.h"
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

using perception::elevation_mapping_params_io;
using perception::perceptive_locomotion_preprocessor_params;
using perception::PerceptiveLocomotionPreprocessor;
using perception::DrakeToPclPointCloud;

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
        0.0005, communication_delay_periods_, drake::Value<lcmt_cassie_out>()
  );
  delay->set_name("communication_delay");

  // state estimator
  // TODO: Add option to set joint offsets in state estimator
  state_estimator_ = builder.AddSystem<systems::CassieStateEstimator>(
      *plant_, &fourbar_, &left_contact_, &right_contact_, joint_offsets_,
      false, false, 2
  );

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
      {} // crop boxes
  };
  auto preprocessor = std::make_shared<PerceptiveLocomotionPreprocessor>(
      *plant_, plant_context_.get(), preprocessor_params,
      elevation_mapping::SensorProcessorBase::GeneralParameters{"pelvis", "world"}
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
  builder.Connect(state_estimator_->get_covariance_output_port(),
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
    const Eigen::VectorXd& q, const Eigen::VectorXd& v) const {
  auto& delay_system =
      dynamic_cast<const drake::systems::DiscreteTimeDelay<double>&>(
          GetSubsystemByName("communication_delay")
      );
  auto& delay_context = delay_system.GetMyMutableContextFromRoot(root_context);
  for (int i = 0; i < communication_delay_periods_; i++) {
    delay_system.SaveInputToBuffer(&delay_context);
  }
}

}
}