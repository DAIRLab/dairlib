#pragma once

// STL
#include <queue>
#include <list>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// open_vins
#include "cam/CamRadtan.h"
#include "track/TrackKLT.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"

// dairlib
#include "dairlib/lcmt_landmark_array.hpp"
#include "systems/perception/feature_tracking/klt_tracking_params.h"
#include "systems/perception/image_pair.h"

// drake
#include "drake/systems/framework/leaf_system.h"


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

 class FeatureTracker : public drake::systems::LeafSystem<double> {
 public:
  static constexpr int kImgQueueSize = 10;

  explicit FeatureTracker(
      const feature_tracking_node_params& params);

   explicit FeatureTracker(const std::string& params_file) :
   FeatureTracker(
       drake::yaml::LoadYamlFile<feature_tracking_node_params>(params_file)){};

  void SetMask(const cv::Mat& mask) {
    mask_ = mask;
  };

  void SetMask(const Eigen::MatrixXd& mask) {
    cv::Mat mat(mask.rows(), mask.cols(), CV_8UC1);

    for (int i = 0; i < mask.rows(); ++i) {
      for (int j = 0; j < mask.cols(); ++j) {
        if (mask(i, j) > 0) {
          mat.at<uchar>(i, j) = 255;
        }
      }
    }
    mask_ = mat;
  };

  Eigen::MatrixXd get_empty_mask() {
    return Eigen::MatrixXd::Zero(
        params_.intrinsics.at("height"), params_.intrinsics.at("width"));
  }

 private:

  drake::systems::EventStatus
  UnrestrictedUpdate(const drake::systems::Context<double> &context,
                     drake::systems::State<double>* state) const;

  void SetDefaultState(const drake::systems::Context<double>& context,
                        drake::systems::State<double>* state) const final;

  void ProcessImagePair(const ImagePair& images,
                        ov_core::TrackKLT& tracker,
                        std::list<size_t>& ids_to_delete,
                        lcmt_landmark_array& landmarks) const;

  Eigen::Vector3d DeprojectLatest3d(
      const ov_core::Feature& feature, const cv::Mat& depth) const;

 drake::systems::EventStatus Initialize(
    const drake::systems::Context<double>& context, drake::systems::State<double>* state) const {
  return drake::systems::EventStatus::Succeeded();
}

  drake::systems::InputPortIndex input_port_image_pair_;

  drake::systems::AbstractStateIndex tracker_index_;
  drake::systems::AbstractStateIndex ids_to_delete_index_;
  drake::systems::AbstractStateIndex prev_timestamp_index_;
  drake::systems::AbstractStateIndex current_landmarks_index_;

  // params
  feature_tracking_node_params params_;

  // internal data
  std::shared_ptr<ov_core::CamBase> camera_;

  const size_t queue_size_ = 100;

  cv::Mat mask_;
  const int cam_id_ = 0;

};

}
}
