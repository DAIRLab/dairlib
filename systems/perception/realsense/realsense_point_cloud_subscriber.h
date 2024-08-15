#pragma once

#include "single_rs_interface.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::perception {

/*!
 * Subscribes to decimated depth images from a SingleRSInterface, and transforms
 * them into pcl pointclouds
 */
template <typename PointT>
class RealsensePointCloudSubscriber : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RealsensePointCloudSubscriber);
  RealsensePointCloudSubscriber(rs2_systems::SingleRSInterface *realsense);

 private:
  void SetDefaultState(const drake::systems::Context<double>& context,
                       drake::systems::State<double>* state) const final;
  void HandleFrame(const rs2_systems::SingleRSInterface::rs_frames& frame);

  void DoCalcNextUpdateTime(const drake::systems::Context<double>& context,
                            drake::systems::CompositeEventCollection<double>* events,
                            double* time) const final;

  drake::systems::EventStatus ProcessFrameAndStoreToAbstractState(
      const drake::systems::Context<double>&,
          drake::systems::State<double>* state) const;

  drake::systems::EventStatus Initialize(
      const drake::systems::Context<double>&,
          drake::systems::State<double>* state) const;

  mutable std::mutex received_frame_mutex_;

  int received_frames_count_;
  rs2::frame received_depth_frame_;

  rs2_systems::SingleRSInterface* rs_interface_;

  drake::systems::AbstractStateIndex frame_count_index_;
  drake::systems::AbstractStateIndex point_cloud_index_;

};
}