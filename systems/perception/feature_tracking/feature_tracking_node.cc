#include "feature_tracking_node.h"

namespace dairlib {
namespace perception {

using ros::NodeHandle;

using ov_core::TrackKLT;
using ov_core::Feature;
using ov_core::FeatureDatabase;
using ov_core::CameraData;
using ov_core::CamRadtan;
using ov_core::CamBase;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

FeatureTrackingNode::FeatureTrackingNode(
    ros::NodeHandle& node_handle,
    const feature_tracking_node_params& params) :
    node_handle_(node_handle),
    params_(params),
    img_pair_sub_(kImgQueueSize) /* queue size */ {

  // initialize camera model
  camera_ = std::make_shared<CamRadtan>(
      static_cast<int>(params.intrinsics.at("width")),
      static_cast<int>(params.intrinsics.at("height")));

  VectorXd camera_calib = VectorXd::Zero(8);
  camera_calib(0) = params.intrinsics.at("focal_x");
  camera_calib(1) = params.intrinsics.at("focal_y");
  camera_calib(2) = params.intrinsics.at("center_x");
  camera_calib(3) = params.intrinsics.at("center_y");

  camera_->set_value(camera_calib); // who named these functions? lol

  std::unordered_map<size_t, std::shared_ptr<CamBase>> cams{{cam_id_, camera_}};

  // initialize tracker
  tracker_ = std::make_unique<TrackKLT>(
      cams,
      params.tracker_params.num_pts,
      params.tracker_params.num_aruco,
      params.tracker_params.use_stereo,
      params.tracker_params.histogram,
      params.tracker_params.fast_threshold,
      params.tracker_params.grid_x,
      params.tracker_params.grid_y,
      params.tracker_params.min_px_dist
  );

  // initialize mask
  mask_ = cv::Mat::zeros(
      static_cast<int>(params.intrinsics.at("height")),
      static_cast<int>(params.intrinsics.at("width")), CV_8UC1);

  // setup ros publishers and subscribers
  color_img_sub_.subscribe(node_handle, params.rgb_topic, kImgQueueSize);
  depth_img_sub_.subscribe(node_handle, params.depth_topic, kImgQueueSize);
  img_pair_sub_.connectInput(color_img_sub_, depth_img_sub_);
  img_pair_sub_.registerCallback(&FeatureTrackingNode::ImagePairCallback, this);

}

Eigen::Vector3d FeatureTrackingNode::Reproject3d(
    const cv::KeyPoint& point, const cv::Mat& depth) const {
  int u = point.x;
  int v = point.y;

  // Extracting depth from depth map
  float z = depth.at<float>(v, u);

  float cx = params_.intrinsics.at("center_x");
  float cy = params_.intrinsics.at("center_y");
  float fx = params_.intrinsics.at("focal_x");
  float fy = params_.intrinsics.at("focal_y");

  // TODO (@Brian-Acosta) check chat-gpt's work here
  // Reprojecting into 3D space
  float x = (u - cx) * depth / fx;
  float y = (v - cy) * depth / fy;

  return Vector3d(x, y, z);
}

void FeatureTrackingNode::ImagePairCallback(
    const sensor_msgs::ImageConstPtr& color,
    const sensor_msgs::ImageConstPtr& depth) {

  cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvShare(
      color, sensor_msgs::image_encodings::MONO8);

  double timestamp = img_ptr->header.stamp.toSec();

  // feed the rgb image to the tracker
  CameraData data;
  data.timestamp = timestamp;
  data.sensor_ids.push_back(cam_id_);
  data.images.push_back(img_ptr->image);
  data.masks.push_back(mask_);
  tracker_->feed_new_camera(data);

  // maybe display the tracking history
  if (params_.tracker_params.display) {
    cv::Mat img_active;
    tracker_->display_active(img_active, 255, 0, 0, 0, 0, 255);
    cv::imshow("Active Tracks", img_active);
  }

  // Delete lost and expired features
  std::shared_ptr<FeatureDatabase> features = tracker_->get_feature_database();
  std::vector<std::shared_ptr<Feature>> lost =
      features->features_not_containing_newer(timestamp);

  for (auto& feat : lost) {
    feat->to_delete = true;
    ids_to_delete_.push_back(feat->featid);
  }

  timestamps_.push(timestamp);

  if (timestamps_.size() > params_.tracker_params.clone_states) {
    std::vector<std::shared_ptr<Feature>> feats_marg =
        features->features_containing(timestamps_.front());
    for (auto& feat: feats_marg) {
      feat->to_delete = true;
      ids_to_delete_.push_back(feat->featid);
    }
    timestamps_.pop();
  }
  features->cleanup();

  // publish feature message
  lcmt_landmark_array landmarks;
  landmarks.num_landmarks = features->size();
  landmarks.num_expired = ids_to_delete_.size();
  landmarks.utime = color->header.stamp.toNSec() / 1e3;
  landmarks.expired_landmark_ids.clear();
  for (const auto & id : ids_to_delete_) {
    landmarks.expired_landmark_ids.push_back(id);
  }
  // TODO (@Brian-Acosta) do the depth re-projection here - handle edge case of
  //  bad depth info
  lcm_.publish(params_.landmark_channel, &landmarks);

  // finally, cleanup the deletion list
  while (ids_to_delete_.size() > queue_size_) {
    ids_to_delete_.pop_front();
  }
}

}
}