#pragma once

// STL
#include <queue>
#include <list>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// open_vins
#include "cam/CamRadtan.h"
#include "track/TrackKLT.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"

// dairlib
#include "lcm/lcm-cpp.hpp"
#include "dairlib/lcmt_landmark_array.hpp"
#include "systems/perception/feature_tracking/klt_tracking_params.h"

namespace dairlib {
namespace perception {

struct feature_tracking_node_params {
  std::string tracker_params_yaml;
  std::string sensor_extrinsics_yaml;
  std::string sensor_intrinsics_yaml;
  std::string rgb_topic;
  std::string depth_topic;
  std::string landmark_channel;
  double max_depth;
  double min_depth;
  klt_tracking_params tracker_params;
  Eigen::Matrix3d sensor_orientation;
  Eigen::Vector3d sensor_translation;

  // TODO (@Brian-Acosta) separate intrinsics needed for depth and color?
  std::map<std::string, double> intrinsics;

  template<typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(tracker_params_yaml));
    a->Visit(DRAKE_NVP(sensor_extrinsics_yaml));
    a->Visit(DRAKE_NVP(sensor_intrinsics_yaml));
    a->Visit(DRAKE_NVP(rgb_topic));
    a->Visit(DRAKE_NVP(depth_topic));
    a->Visit(DRAKE_NVP(landmark_channel));
    a->Visit(DRAKE_NVP(max_depth));
    a->Visit(DRAKE_NVP(min_depth));

    tracker_params = drake::yaml::LoadYamlFile<klt_tracking_params>(
        tracker_params_yaml);

    intrinsics = drake::yaml::LoadYamlFile<std::map<std::string, double>>(
        sensor_intrinsics_yaml);

    auto R_p = drake::yaml::LoadYamlFile<
        std::map<std::string, std::vector<double>>>(sensor_extrinsics_yaml);

    DRAKE_DEMAND(R_p.count("translation") == 1);
    DRAKE_DEMAND(R_p.count("rotation") == 1);
    DRAKE_DEMAND(R_p.at("translation").size() == 3);
    DRAKE_DEMAND(R_p.at("rotation").size() == 9);

    sensor_orientation =Eigen::Map<Eigen::Matrix<
        double, 3, 3, Eigen::RowMajor>>(R_p.at("rotation").data());
    sensor_translation = Eigen::Map<Eigen::Vector3d>(
        R_p.at("translation").data());
  }
};

class FeatureTrackingNode {
 public:
  static constexpr int kImgQueueSize = 10;

  explicit FeatureTrackingNode(
      ros::NodeHandle& node_handle,
      const feature_tracking_node_params& params);

  void SetMask(const cv::Mat& mask);
  void SetMask(const Eigen::MatrixXd& mask);

 private:

  void ImagePairCallback(
      const sensor_msgs::ImageConstPtr& color,
      const sensor_msgs::ImageConstPtr& depth);

  Eigen::Vector3d DeprojectLatest3d(
      const ov_core::Feature& feature, const cv::Mat& depth) const;

  // ROS
  ros::NodeHandle& node_handle_;

  // params
  feature_tracking_node_params params_;

  // internal data
  std::unique_ptr<ov_core::TrackKLT> tracker_;
  std::shared_ptr<ov_core::CamBase> camera_;
  std::list<size_t> ids_to_delete_;
  std::queue<double> timestamps_;
  const size_t queue_size_ = 100;

  cv::Mat mask_;
  const int cam_id_ = 0;

  // io
  message_filters::Subscriber<sensor_msgs::Image> color_img_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_img_sub_;
  message_filters::TimeSynchronizer<
  sensor_msgs::Image, sensor_msgs::Image> img_pair_sub_;

  lcm::LCM lcm_{"udpm://239.255.76.67:7667?ttl=1"};

};

}
}