#include "simulated_hmap_point_server.h"

#include "pcl/common/transforms.h"

#include "multibody/multibody_utils.h"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "systems/framework/output_vector.h"


namespace dairlib {
namespace perception {

using drake::systems::Context;
using systems::OutputVector;

using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::math::RigidTransformd;



template<typename PointT>
SimulatedHmapPointServer<PointT>::SimulatedHmapPointServer(
    const drake::multibody::MultibodyPlant<double>& plant,
    std::unique_ptr<drake::systems::Context<double>> context,
    elevation_mapping_params params,
    drake::systems::sensors::CameraInfo camera_info,
    geometry::ConvexPolygonSet polygons) :
    plant_(plant),
    context_(std::move(context)),
    robot_base_(plant.GetBodyByName(params.base_frame_name)),
    camera_info_(camera_info),
    track_point_(params.track_point),
    params_(params),
    polygons_(polygons){

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(
          plant.num_positions(), plant.num_velocities(), plant.num_actuators()
  )).get_index();

  typename pcl::PointCloud<PointT>::Ptr model =
      std::make_shared<pcl::PointCloud<PointT>>();

  output_port_pcl_ = DeclareAbstractOutputPort(
      "PointCloudPtr", model, &SimulatedHmapPointServer<PointT>::Calc
  ).get_index();

  Eigen::Matrix4f camera_coords;
  camera_coords << 0, 0, 1, 0,
                   0, -1, 0, 0,
                   1, 0, 0, 0,
                   0, 0, 0, 1;
  fov_filter_.setCameraPose(camera_coords);
  fov_filter_.setHorizontalFOV(180 * camera_info_.fov_y() / 3.14159265);
  fov_filter_.setVerticalFOV(180 * camera_info_.fov_x() / 3.14159265);


  for (const auto& pose_param : params.sensor_poses) {
    DRAKE_DEMAND(plant_.HasBodyNamed(pose_param.sensor_parent_body_));
    sensor_pose_ = pose_param;
  }

}

template <typename PointT>
void SimulatedHmapPointServer<PointT>::Calc(
    const Context<double>&context,
    typename pcl::PointCloud<PointT>::Ptr* ptr) const {

  auto& pcl_cloud = *ptr;
  pcl_cloud->header.frame_id = "";
  pcl_cloud->header.stamp = 1e6 * context.get_time();
  AssignFields(context, pcl_cloud);
}

template <typename PointT>
void SimulatedHmapPointServer<PointT>::AssignFields(
    const Context<double>& context,
    typename pcl::PointCloud<PointT>::Ptr& ptr) const {

  auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_)
  );

  ptr->header.stamp = 1e6 * robot_output->get_timestamp();

  VectorXd q_v = robot_output->GetState();

  multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, q_v, context_.get());
  const auto X_WB = plant_.EvalBodyPoseInWorld(*context_, robot_base_);
  Vector3d track_point_in_world = X_WB * track_point_;

  int n_points = static_cast<int>(params_.map_length.x() / params_.resolution);
  double half_length = params_.map_length.x() / 2;

  ptr->points.resize(n_points * n_points);


  const auto X_WP = plant_.GetBodyByName(
      sensor_pose_.sensor_parent_body_).EvalPoseInWorld(*context_);

  RigidTransformd X_WS = X_WP * sensor_pose_.sensor_pose_in_parent_body_;

  // fill in the pointcloud
  int nfinite = 0;
  for (int i = 0; i < n_points; ++i) {
    for (int j = 0; j < n_points; ++j) {
      Vector3d world_point = track_point_in_world;
      world_point.x() += i * params_.resolution - half_length;
      world_point.y() += j * params_.resolution - half_length;
      world_point.z() = polygons_.CalcHeightOfPoint(world_point);
      if (world_point.array().isFinite().all()) {
        ptr->points[n_points * i + j].getVector3fMap() = (X_WS.inverse() * world_point).cast<float>();
        ++nfinite;
      }
    }
  }
  ptr->points.resize(nfinite);
  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*ptr, *ptr, indices);

  fov_filter_.setInputCloud(ptr);
  fov_filter_.filter(*ptr);
}

template class SimulatedHmapPointServer<pcl::PointXYZ>;
template class SimulatedHmapPointServer<pcl::PointXYZRGB>;
template class SimulatedHmapPointServer<pcl::PointXYZRGBConfidenceRatio>;

}
}