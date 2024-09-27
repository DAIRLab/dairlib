#include "feature_tracker.h"
#include "systems/perception/feature_tracking/ov_core/feat/Feature.h"

namespace dairlib {
namespace perception {

using ov_core::TrackKLT;
using ov_core::Feature;
using ov_core::FeatureDatabase;
using ov_core::CameraData;
using ov_core::CamRadtan;
using ov_core::CamBase;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

using drake::Value;
using drake::systems::Context;
using drake::systems::State;

FeatureTracker::FeatureTracker(const feature_tracking_node_params& params) :
    params_(params) {

  // initialize camera model
  camera_ = std::make_shared<CamRadtan>(
      static_cast<int>(params.intrinsics.at("width")),
      static_cast<int>(params.intrinsics.at("height")));

  VectorXd camera_calib = VectorXd::Zero(8);
  camera_calib(0) = params.intrinsics.at("focal_x");
  camera_calib(1) = params.intrinsics.at("focal_y");
  camera_calib(2) = params.intrinsics.at("center_x");
  camera_calib(3) = params.intrinsics.at("center_y");

  camera_->set_value(camera_calib);

  tracker_index_ = DeclareAbstractState(Value<std::shared_ptr<TrackKLT>>(nullptr));
  ids_to_delete_index_ = DeclareAbstractState(Value<std::list<size_t>>());
  prev_timestamp_index_ = DeclareAbstractState(Value<uint64_t>(0));
  current_landmarks_index_ = DeclareAbstractState(Value<lcmt_landmark_array>());

  DeclareStateOutputPort(
      drake::systems::kUseDefaultName, current_landmarks_index_);

  input_port_image_pair_ = DeclareAbstractInputPort(
      "image pairs", Value<std::shared_ptr<ImagePair>>(nullptr)).get_index();

  DeclareForcedUnrestrictedUpdateEvent(&FeatureTracker::UnrestrictedUpdate);

  // initialize mask
  mask_ = cv::Mat::zeros(
      static_cast<int>(params.intrinsics.at("height")),
      static_cast<int>(params.intrinsics.at("width")), CV_8UC1);

}

void FeatureTracker::SetDefaultState(
    const Context<double> &context, State<double> *state) const {
  std::unordered_map<size_t, std::shared_ptr<CamBase>> cams{{cam_id_, camera_}};

  auto& tracker = state->get_mutable_abstract_state<
      std::shared_ptr<TrackKLT>>(tracker_index_);

  tracker = std::make_unique<TrackKLT>(
      cams,
      params_.tracker_params.num_pts,
      params_.tracker_params.num_aruco,
      params_.tracker_params.use_stereo,
      params_.tracker_params.histogram,
      params_.tracker_params.fast_threshold,
      params_.tracker_params.grid_x,
      params_.tracker_params.grid_y,
      params_.tracker_params.min_px_dist
  );

}

drake::systems::EventStatus FeatureTracker::UnrestrictedUpdate(
    const drake::systems::Context<double> &context, drake::systems::State<double>* state) const {

  auto& tracker = state->get_mutable_abstract_state<std::shared_ptr<TrackKLT>>(tracker_index_);
  auto& ids_to_delete = state->get_mutable_abstract_state<std::list<size_t>>(ids_to_delete_index_);
  auto& prev_timestamp = state->get_mutable_abstract_state<uint64_t>(prev_timestamp_index_);
  auto& landmarks = state->get_mutable_abstract_state<lcmt_landmark_array>(current_landmarks_index_);

  auto images = EvalAbstractInput(context, input_port_image_pair_)->get_value<std::shared_ptr<ImagePair>>();

  if (images != nullptr and images->utime_ > prev_timestamp) {
    landmarks.landmarks.clear();
    ProcessImagePair(*images, *tracker, ids_to_delete, landmarks);
    prev_timestamp = images->utime_;
    return drake::systems::EventStatus::Succeeded();
  }
  return drake::systems::EventStatus::DidNothing();
}

// Reference:
// https://github.com/IntelRealSense/librealsense/blob/4673a37d981164af8eeb8e296e430fc1427e008d/src/rs.cpp#L3576
Eigen::Vector3d FeatureTracker::DeprojectLatest3d(
  const ov_core::Feature &feature, const cv::Mat& depth) const {
  const Eigen::VectorXf& uv = feature.uvs.at(cam_id_).back();

  // Extracting depth from depth map
  int u = static_cast<int>(uv(0));
  int v = static_cast<int>(uv(1));
  ushort z_int = depth.at<ushort>(u, v);
  float z = static_cast<float>(z_int) * params_.intrinsics.at("depth_scale");

  if (z > params_.max_depth or z < params_.min_depth) {
    return Vector3d::Constant(std::nan(""));
  }

  float cx = params_.intrinsics.at("center_x");
  float cy = params_.intrinsics.at("center_y");
  float fx = params_.intrinsics.at("focal_x");
  float fy = params_.intrinsics.at("focal_y");
  float c0 = params_.intrinsics.at("dc0");
  float c1 = params_.intrinsics.at("dc1");
  float c2 = params_.intrinsics.at("dc2");
  float c3 = params_.intrinsics.at("dc3");
  float c4 = params_.intrinsics.at("dc4");

  // Reprojecting into 3D space
  float x = (uv(0) - cx) / fx;
  float y = (uv(1) - cy) / fy;

  float xo = x;
  float yo = y;

  // need to loop until convergence
  for (int i = 0; i < 10; i++) {
    float r2 = x * x + y * y;
    float icdist = (float)1 / (float)(1 + ((c4 * r2 + c1) * r2 + c0) * r2);
    float xq = x / icdist;
    float yq = y / icdist;
    float delta_x = 2 * c2 * xq * yq + c3 * (r2 + 2 * xq * xq);
    float delta_y = 2 * c3 * xq * yq + c2 * (r2 + 2 * yq * yq);
    x = (xo - delta_x) * icdist;
    y = (yo - delta_y) * icdist;
  }
  
  Vector3d xyz(z * x, z * y, z);
//  std::cout << xyz.transpose() << std::endl;
  return params_.sensor_orientation * xyz + params_.sensor_translation;
}

void FeatureTracker::ProcessImagePair(
    const ImagePair& images, TrackKLT& tracker,
    std::list<size_t>& ids_to_delete, lcmt_landmark_array& landmarks) const {

  double timestamp = images.utime_ / 1e6;

  // feed the rgb image to the tracker
  CameraData data;
  data.timestamp = timestamp;
  data.sensor_ids.push_back(cam_id_);
  data.images.push_back(images.gray_);
  data.masks.push_back(mask_);
  tracker.feed_new_camera(data);

  // maybe display the tracking history
  if (params_.tracker_params.display) {
    cv::Mat img_active;
    tracker.display_active(img_active, 255, 0, 0, 0, 0, 255);
    cv::imshow("Active Tracks", img_active);
    cv::waitKey(10);
  }

  // Delete lost features
  std::shared_ptr<FeatureDatabase> features = tracker.get_feature_database();
  std::vector<std::shared_ptr<Feature>> lost =
      features->features_not_containing_newer(timestamp);

  for (auto& feat : lost) {
    feat->to_delete = true;
    ids_to_delete.push_back(feat->featid);
  }

  // Delete expired features
  features->cleanup();

  // Get 3d coords of valid features
  for (const auto& feat: features->features_containing(timestamp)) {
    Vector3d xyz = DeprojectLatest3d(*feat, images.depth_);
    lcmt_landmark landmark;
    if (not xyz.hasNaN()) {
      landmark.id = feat->featid;
      memcpy(landmark.position, xyz.data(), 3 * sizeof(double));
      landmarks.landmarks.push_back(landmark);
    } else {
      feat->to_delete = true;
      ids_to_delete.push_back(feat->featid);
    }
  }
  features->cleanup();

  // publish feature message
  landmarks.num_landmarks = landmarks.landmarks.size();
  landmarks.num_expired = ids_to_delete.size();
  landmarks.utime = images.utime_;
  landmarks.expired_landmark_ids.clear();

  for (const auto & id : ids_to_delete) {
    landmarks.expired_landmark_ids.push_back(id);
  }

  // finally, cleanup the deletion list
  while (ids_to_delete.size() > queue_size_) {
    ids_to_delete.pop_front();
  }
}

}
}