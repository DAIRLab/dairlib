//dairlib
#include "multibody/multibody_utils.h"
#include "systems/perception/elevation_mapping_system.h"
#include "systems/perception/perceptive_locomotion_preprocessor.h"
#include "systems/framework/output_vector.h"
#include "common/time_series_buffer.h"

namespace dairlib {
namespace perception {

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

using systems::OutputVector;

using drake::systems::State;
using drake::systems::Context;
using drake::systems::TriggerType;
using drake::systems::EventStatus;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::MultibodyPlant;

using grid_map::GridMap;
using elevation_mapping::ElevationMap;
using elevation_mapping::PointCloudType;
using elevation_mapping::RobotMotionMapUpdater;
using elevation_mapping::SensorProcessorBase;

namespace {
  static constexpr size_t kBufSize = 20;
}

ElevationMappingSystem::ElevationMappingSystem(
    const MultibodyPlant<double>& plant,
    Context<double>* context,
    elevation_mapping_params params) :
    plant_(plant),
    robot_base_(plant.GetBodyByName(params.base_frame_name)),
    context_(context),
    track_point_(params.track_point),
    params_(params) {

  double pitch_offset = params.sensor_poses.size() > 1 ?
    0 : params.pitch_bias * M_PI / 180.0;
  auto X_offset = RigidTransformd(
    RotationMatrixd::MakeXRotation(pitch_offset), Vector3d::Zero());

  // configure sensors
  drake::Vector1d prev_time_model_vector{-1};
  for (const auto& pose_param : params.sensor_poses) {
    DRAKE_DEMAND(plant_.HasBodyNamed(pose_param.sensor_parent_body_));
    DRAKE_DEMAND(sensor_poses_.count(pose_param.sensor_name_) == 0);
    auto pose_param_temp = pose_param;
    pose_param_temp.sensor_pose_in_parent_body_ = pose_param.sensor_pose_in_parent_body_ * X_offset;
    sensor_poses_.insert({pose_param.sensor_name_, pose_param_temp});
    input_ports_pcl_.insert({pose_param.sensor_name_, DeclareAbstractInputPort(
        "Point_cloud_" + pose_param.sensor_name_,
        drake::Value<PointCloudType::Ptr>()).get_index()
    });
    sensor_prev_update_time_indices_.insert(
        {pose_param.sensor_name_, DeclareDiscreteState(prev_time_model_vector)}
    );
  }

  // configure contacts
  for (const auto& [k, v]: params.contacts) {
    DRAKE_DEMAND(plant_.HasBodyNamed(v.first));
    contacts_.insert({k, v});
  }

  // state input
  input_port_state_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(
          plant.num_positions(), plant.num_velocities(), plant.num_actuators()
      )).get_index();

  // covariance of the floating base pose in column major order
  input_port_pose_covariance_ = DeclareVectorInputPort("cov", 36).get_index();

  if (not contacts_.empty()) {
    input_port_contact_ = DeclareAbstractInputPort(
        "lcmt_contact", drake::Value<lcmt_contact>()
    ).get_index();
  }

  // create the elevation map
  ElevationMap map;
  map.setGeometry(params.map_length, params.resolution, track_point_.head<2>());

  auto model_value_map = drake::Value<ElevationMap>(map);
  auto model_value_updater = drake::Value<RobotMotionMapUpdater>();
  elevation_map_state_index_ = DeclareAbstractState(model_value_map);
  motion_updater_state_index_ = DeclareAbstractState(model_value_updater);

  state_buffer_index_ = DeclareAbstractState(
      drake::Value<std::shared_ptr<TimeSeriesBuffer<VectorXd, kBufSize>>>(nullptr));

  output_port_elevation_map_ = DeclareStateOutputPort(
      "elevation_map", elevation_map_state_index_
  ).get_index();

  output_port_grid_map_ = DeclareAbstractOutputPort(
      "grid_map", &ElevationMappingSystem::CopyGridMap
  ).get_index();

  // create update event
  auto& update_params = params.update_params;
  switch (update_params.map_update_type_) {
    case TriggerType::kForced:
      DeclareForcedUnrestrictedUpdateEvent(
          &ElevationMappingSystem::ElevationMapUpdateEvent
    );
      break;
    case TriggerType::kPeriodic:
      DRAKE_DEMAND(update_params.map_update_rate_hz_ > 0);
      DeclarePeriodicUnrestrictedUpdateEvent(
          1.0 / update_params.map_update_rate_hz_, 0,
          &ElevationMappingSystem::ElevationMapUpdateEvent
      );
      break;
    default:
      throw std::runtime_error(
          "Elevation map update type must be kPeriodic or kForced");
  }
  DeclarePerStepUnrestrictedUpdateEvent(
      &ElevationMappingSystem::UpdateStateBuffer);
}

void ElevationMappingSystem::SetDefaultState(const Context<double> &context,
                                             State<double> *state) const {
  auto& buf_ptr = state->get_mutable_abstract_state<
      std::shared_ptr<TimeSeriesBuffer<VectorXd, kBufSize>>>(state_buffer_index_);
  buf_ptr = std::make_shared<TimeSeriesBuffer<VectorXd, kBufSize>>();
  buf_ptr->reset();
}

EventStatus ElevationMappingSystem::Initialize(const Context<double> &context,
                                               State<double> *state) const {
  auto& buf_ptr = state->get_mutable_abstract_state<
      std::shared_ptr<TimeSeriesBuffer<VectorXd, kBufSize>>>(state_buffer_index_);
  buf_ptr->reset();

  return EventStatus::Succeeded();
}

EventStatus ElevationMappingSystem::UpdateStateBuffer(
    const Context<double> &context, State<double> *state) const {

  // 1. Get the robot base pose and covariance
  const auto& robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_)
  );

  uint64_t now = 1e6 * robot_output->get_timestamp();
  const VectorXd& robot_state = robot_output->GetState();

  auto& buf_ptr = state->get_mutable_abstract_state<
      std::shared_ptr<TimeSeriesBuffer<VectorXd, kBufSize>>>(state_buffer_index_);

  buf_ptr->put(now, robot_state);
  return EventStatus::Succeeded();
}

void ElevationMappingSystem::AddSensorPreProcessor(
    const std::string& sensor_name,
    std::shared_ptr<SensorProcessorBase> processor) {

  DRAKE_DEMAND(sensor_poses_.count(sensor_name) == 1);
  auto processor_derived = dynamic_cast<PerceptiveLocomotionPreprocessor*>(
      processor.get()
  );
  if (processor_derived != nullptr) {
    // processor must share the plant context in order for state info to be
    // propagated correctly
    DRAKE_DEMAND(processor_derived->context() == context_);
  }
  sensor_preprocessors_.insert({sensor_name, processor});
}

double ElevationMappingSystem::CalcMapOffsetFromContactState(
    lcmt_contact contact_msg, const grid_map::GridMap& map) const {

  double map_offset = 0;
  int n_valid_contacts = 0;

  for (int i = 0; i < contact_msg.num_contacts; i++) {
    if (contact_msg.contact[i] and contacts_.count(contact_msg.contact_names[i]) > 0) {

      const auto& contact = contacts_.at(contact_msg.contact_names[i]);
      const Vector3d stance_pos = plant_.EvalBodyPoseInWorld(
          *context_,
          plant_.GetBodyByName(contact.first)
      ) * contact.second;
      try {

        double sub_map_length = 3.0 * map.getResolution();
        grid_map::Position center_sub_map = stance_pos.head<2>();
        grid_map::Length length_sub_map = {sub_map_length, sub_map_length};
        bool success;

        // Getting the submap of where the foot location is
        auto sub_map = map.getSubmap(center_sub_map, length_sub_map, success);

        if (success) {
          // Retrieving the data and making the median of the values
          const auto& mat = sub_map.get("elevation");
          std::vector<double> zvals(mat.data(), mat.data() + mat.rows() * mat.cols());
          std::sort(zvals.begin(), zvals.end());
          int n = zvals.size() / 2;
          double map_z = (zvals.size() % 2 == 0) ?
                         0.5 * (zvals.at(n-1) + zvals.at(n)) : zvals.at(n);

          if (not std::isnan(map_z)) {
            map_offset += stance_pos(2) - map_z;
            ++n_valid_contacts;
          }
        }
      } catch (std::out_of_range& ex) {
        drake::log()->warn("{}", ex.what());
      }
    }
  }
  map_offset = (n_valid_contacts > 0) ? map_offset / n_valid_contacts : 0;
  return map_offset;
}

std::map<std::string, PointCloudType::Ptr>
ElevationMappingSystem::CollectNewPointClouds(
    const Context<double>& context, State<double>* state) const {

  std::map<std::string, PointCloudType::Ptr> new_pointclouds{};

  for (const auto& [name, input_port_idx_pcl] : input_ports_pcl_) {

    // Get the timestamp of the previous pointcloud
    double prev_pointcloud_stamp = state->get_discrete_state(
        sensor_prev_update_time_indices_.at(name)
    ).value()(0);

    auto pointcloud = EvalAbstractInput(
        context, input_port_idx_pcl)->get_value<PointCloudType::Ptr>();

    // Only do anything if the timestamp has been updated
    if (pointcloud->header.stamp * 1e-6 > prev_pointcloud_stamp and pointcloud->size() > 0) {
      new_pointclouds.insert({name, pointcloud});

      // update the timestamp of the previous point cloud seen from this sensor
      state->get_mutable_discrete_state(
          sensor_prev_update_time_indices_.at(name)
      ).set_value(drake::Vector1d::Constant(pointcloud->header.stamp * 1e-6));
    }
  }

  return new_pointclouds;
}

drake::systems::EventStatus ElevationMappingSystem::ElevationMapUpdateEvent(
    const Context<double>& context, State<double>* state) const {

  auto new_pointclouds = CollectNewPointClouds(context, state);

  if (new_pointclouds.empty()) {
    return drake::systems::EventStatus::DidNothing();
  }

  // Get the elevation map
  auto& map = state->get_mutable_abstract_state<ElevationMap>(
      elevation_map_state_index_);
  auto& motion_updater = state->get_mutable_abstract_state<RobotMotionMapUpdater>(
      motion_updater_state_index_);

  // 1. Get the robot base pose and covariance
  auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_));

  VectorXd q_v = robot_output->GetState();
  double timestamp = robot_output->get_timestamp();

  MatrixXd pose_covariance = EvalVectorInput(
      context, input_port_pose_covariance_)->get_value();
  pose_covariance.resize(6,6);
  multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, q_v, context_);
  const auto base_pose = plant_.EvalBodyPoseInWorld(*context_, robot_base_);

  // 2. Update the map location
  Vector3d track_point_in_world = base_pose * track_point_;
  map.move(track_point_in_world.head<2>());

  // 3. Apply prediction step
  motion_updater.update(map, base_pose, pose_covariance, timestamp);

  // 4. If contact information is provided, update the map using contact points
  //    as a reference
  if (has_contacts()) {
    const auto& contact_msg = EvalAbstractInput(
        context, input_port_contact_)->get_value<lcmt_contact>();
    double map_offset =
        CalcMapOffsetFromContactState(contact_msg, map.getRawGridMap());
    map.shift_map_z(map_offset);
  }

  const auto& state_buffer = state->get_abstract_state<
      std::shared_ptr<TimeSeriesBuffer<VectorXd, kBufSize>>>(state_buffer_index_);

  // 5. add the point clouds to the map
  for (const auto& [name, cloud] : new_pointclouds) {
    // allocate data for processing
    Eigen::VectorXf measurement_variances;
    PointCloudType::Ptr pc_processed(new PointCloudType);

    const auto X_bias = RigidTransformd(params_.point_cloud_bias);

    const auto& state_for_cloud = state_buffer->empty() ?
        q_v : state_buffer->get(cloud->header.stamp);

    multibody::SetPositionsAndVelocitiesIfNew<double>(
        plant_, state_for_cloud, context_
    );

    // TODO (@Brian-Acosta) does it make sense to propogate the base variance
    //  if we add non-base parent frames?
    const auto X_WP = X_bias * plant_.EvalBodyPoseInWorld(
        *context_,
        plant_.GetBodyByName(sensor_poses_.at(name).sensor_parent_body_)
    );
    const auto& X_PS =  sensor_poses_.at(name).sensor_pose_in_parent_body_;

    // apply preprocessor
    sensor_preprocessors_.at(name)->process(
        cloud,
        pose_covariance,
        pc_processed,
        measurement_variances,
        X_PS, X_WP
    );

    map.add(pc_processed, measurement_variances, timestamp, X_WP * X_PS);
  }

  // TODO (@Brian-Acosta) decide how to go about fusing if needed
  //  map.fuseAll();
  return drake::systems::EventStatus::Succeeded();
}

void ElevationMappingSystem::ReInitialize(
    Context<double>* root_context, const GridMap &init_map, std::string layer) const {

  Context<double>& context = this->GetMyMutableContextFromRoot(root_context);

  auto& map = context.get_mutable_abstract_state<ElevationMap>(
      elevation_map_state_index_);

  GridMap tmp_map_in(init_map);
  tmp_map_in["elevation"] = tmp_map_in[layer];

  GridMap tmp_map_out = map.getRawGridMap();
  tmp_map_out.addDataFrom(tmp_map_in, false, true, false, {"elevation"});
  map.setRawGridMap(tmp_map_out);
}

void ElevationMappingSystem::InitializeFlatTerrain(
    const VectorXd& robot_state,
    std::vector<std::pair<const Eigen::Vector3d,
                const drake::multibody::Frame<double>&>> contacts,
    Context<double>& context) const {

  multibody::SetPositionsAndVelocitiesIfNew<double>(
      plant_, robot_state, context_
  );
  auto& map = context.get_mutable_abstract_state<ElevationMap>(
      elevation_map_state_index_
  );
  auto& motion_updater = context.get_mutable_abstract_state<RobotMotionMapUpdater>(
      motion_updater_state_index_
  );
  PointCloudType::Ptr init_pc = std::make_shared<PointCloudType>();

  double resolution = map.getRawGridMap().getResolution();
  int npoints =  2 * std::ceil(params_.initialization_radius / resolution);
  float half_len = static_cast<float>(params_.initialization_radius);

  for (const auto& contact : contacts) {

    Vector3d point_pos;
    plant_.CalcPointsPositions(
        *context_, contact.second, contact.first, plant_.world_frame(),
        &point_pos
    );

    for (int xi = 0; xi < npoints; ++xi) {
      for (int yi = 0; yi < npoints; ++yi) {
        Eigen::Vector4f pt_xyzw(
            xi * resolution - half_len, yi * resolution - half_len, 0, 1.0
        );
        pt_xyzw.head<3>() += point_pos.cast<float>();
        pt_xyzw.z() += static_cast<float>(params_.initialization_offset);

        pcl::PointXYZRGBConfidenceRatio pt;
        pt.getArray4fMap() = Eigen::Map<Eigen::Array4f>(pt_xyzw.data());
        init_pc->push_back(pt);
      }
    }
  }
  Eigen::VectorXf variances = Eigen::VectorXf::Constant(init_pc->size(), 0.01);

  MatrixXd pose_covariance = MatrixXd::Identity(6,6);
  multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, robot_state, context_);
  const auto base_pose = plant_.EvalBodyPoseInWorld(*context_, robot_base_);

  // Update the map location
  Vector3d track_point_in_world = base_pose * track_point_;
  map.move(track_point_in_world.head<2>());

  // Apply prediction step
  motion_updater.update(map, base_pose, pose_covariance, 0);

  // apply measurement step
  map.add(init_pc, variances, 0, RigidTransformd());
}

void ElevationMappingSystem::CopyGridMap(
    const Context<double>& context, GridMap* grid_map) const {
  *grid_map = context.get_abstract_state<ElevationMap>(
      elevation_map_state_index_
  ).getRawGridMap();
}

}
}