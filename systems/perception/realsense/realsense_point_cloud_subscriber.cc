#include "realsense_point_cloud_subscriber.h"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

#include "pcl/filters/filter.h"
#include "pcl/filters/passthrough.h"

#include <iostream>

namespace dairlib::perception {

using drake::Value;
using drake::systems::State;
using drake::systems::Context;
using drake::systems::EventStatus;

using pcl::PointCloud;

using rs2_systems::SingleRSInterface;

template <typename PointT>
RealsensePointCloudSubscriber<PointT>::RealsensePointCloudSubscriber(
    rs2_systems::SingleRSInterface *realsense) {
  DRAKE_DEMAND(realsense != nullptr);

  realsense->Subscribe(
      [this](const SingleRSInterface::rs_frames& frame) {
        this->HandleFrame(frame);
      }
  );

  frame_count_index_ = this->DeclareAbstractState(Value<int>(0));

  point_cloud_index_ = this->DeclareAbstractState(
      Value<std::shared_ptr<PointCloud<PointT>>>(nullptr));

  this->DeclareStateOutputPort(
      drake::systems::kUseDefaultName, point_cloud_index_);

  this->DeclareForcedUnrestrictedUpdateEvent(
      &RealsensePointCloudSubscriber<PointT>::ProcessFrameAndStoreToAbstractState);

  this->DeclareInitializationUnrestrictedUpdateEvent(
      &RealsensePointCloudSubscriber<PointT>::Initialize);

  this->set_name("realsense_pointcloud_subscriber");
}

template <typename PointT>
void RealsensePointCloudSubscriber<PointT>::SetDefaultState(
    const Context<double>& context, State<double>* state) const {

  auto& pc_ptr = state->get_mutable_abstract_state<
      std::shared_ptr<PointCloud<PointT>>>(point_cloud_index_);
  pc_ptr = std::make_shared<PointCloud<PointT>>();

}

template <typename PointT>
void RealsensePointCloudSubscriber<PointT>::DoCalcNextUpdateTime(
    const Context<double> &context,
    drake::systems::CompositeEventCollection<double>* events, double *time) const{

  LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);

  DRAKE_THROW_UNLESS(events->HasEvents() == false);
  DRAKE_THROW_UNLESS(std::isinf(*time));

  const int context_frame_count =
      context.get_abstract_state<int>(frame_count_index_);
  const int received_frame_count = [this]() {
    std::scoped_lock<std::mutex> lock(received_frame_mutex_);
    return received_frames_count_;
  }();

  if (context_frame_count == received_frame_count) {
    return;
  }

  auto callback = [](
      const drake::systems::System<double>& system,
      const Context<double>& callback_context,
      const drake::systems::UnrestrictedUpdateEvent<double>&,
      State<double>* callback_state) {
    const auto& self = dynamic_cast<const RealsensePointCloudSubscriber<PointT>&>(system);
    return self.ProcessFrameAndStoreToAbstractState(callback_context, callback_state);
  };

  *time = context.get_time();
  drake::systems::EventCollection<drake::systems::UnrestrictedUpdateEvent<double>>& uu_events =
      events->get_mutable_unrestricted_update_events();
  uu_events.AddEvent(
      drake::systems::UnrestrictedUpdateEvent<double>(
          drake::systems::TriggerType::kTimed, callback));
}

template <typename PointT>
void RealsensePointCloudSubscriber<PointT>::HandleFrame(
    const rs2_systems::SingleRSInterface::rs_frames &frame) {
  frame_queue_.enqueue(frame.decimated_depth);

  std::scoped_lock<std::mutex> lock(received_frame_mutex_);
  received_frames_count_++;
  received_frame_variable_.notify_one();
}

template <typename PointT>
EventStatus
RealsensePointCloudSubscriber<PointT>::ProcessFrameAndStoreToAbstractState(
    const Context<double> &context, State<double>* state) const {

  {
    std::scoped_lock<std::mutex> lock(received_frame_mutex_);
    const int
        context_frame_count = context.get_abstract_state<int>(frame_count_index_);
    if (context_frame_count == received_frames_count_) {
      state->SetFrom(context.get_state());
      return EventStatus::DidNothing();
    }
    state->get_mutable_abstract_state<int>(frame_count_index_) = received_frames_count_;
  }


  rs2::frame received_depth_frame;
  frame_queue_.poll_for_frame(&received_depth_frame);
  const rs2::depth_frame &depth = received_depth_frame.as<rs2::depth_frame>();
  const int width = depth.get_width();
  const int height = depth.get_height();
  const int npoints = width * height;
  const rs2_intrinsics depth_interinsics = received_depth_frame.get_profile().as<
      rs2::video_stream_profile>().get_intrinsics();


  PointCloud<PointT> &cloud = *state->get_mutable_abstract_state<
      std::shared_ptr<PointCloud<PointT>>>(point_cloud_index_);

  cloud.points.resize(npoints);
  int nfinite = 0;

  for (int y = 0; y < height; ++y) {
    for(int x = 0; x < width; ++x) {
      float d = depth.get_distance(x, y);
      if (d == 0.0f)  continue;

      float pixel[] =  {static_cast<float>(x), static_cast<float>(y)};;
      float point[3];
      rs2_deproject_pixel_to_point(point, &depth_interinsics, pixel, d);
      cloud.points[nfinite].getVector3fMap() = Eigen::Vector3f::Map(point);
      ++nfinite;
    }
  }

  cloud.points.resize(nfinite);
  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(cloud, cloud, indices);

  // ms to us
  cloud.header.stamp = 1e3 * depth.get_timestamp();

//  std::cout << "\ntime: " << context.get_time() << "\nframe number " << received_frames_count_ << "\n"
//            << nfinite << "points, timestamp = " << cloud.header.stamp << std::endl;

  return EventStatus::Succeeded();
}

template <typename PointT>
EventStatus  RealsensePointCloudSubscriber<PointT>::Initialize(
    const drake::systems::Context<double>& context,
    drake::systems::State<double>* state) const {

  return EventStatus::Succeeded();
}

template class RealsensePointCloudSubscriber<pcl::PointXYZ>;
template class RealsensePointCloudSubscriber<pcl::PointXYZRGB>;
template class RealsensePointCloudSubscriber<pcl::PointXYZRGBConfidenceRatio>;

}

