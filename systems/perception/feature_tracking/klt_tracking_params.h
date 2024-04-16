#pragma once

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_io.h"
#include "track/TrackKLT.h"

namespace dairlib {
namespace perception {

struct klt_tracking_params {
  int num_pts = 50;
  int num_aruco = 1024;
  int clone_states = 20;
  int fast_threshold = 20;
  int grid_x = 5;
  int grid_y = 4;
  int min_px_dist = 20;
  bool do_downsizing = false;
  bool use_stereo = false;
  bool display = false;
  std::string histogram_method;
  ov_core::TrackBase::HistogramMethod histogram;

  template <typename Archive>
  void Serialize(Archive *a) {
    a->Visit(DRAKE_NVP(num_pts));
    a->Visit(DRAKE_NVP(num_aruco));
    a->Visit(DRAKE_NVP(clone_states));
    a->Visit(DRAKE_NVP(fast_threshold));
    a->Visit(DRAKE_NVP(grid_x));
    a->Visit(DRAKE_NVP(grid_y));
    a->Visit(DRAKE_NVP(min_px_dist));
    a->Visit(DRAKE_NVP(do_downsizing));
    a->Visit(DRAKE_NVP(use_stereo));
    a->Visit(DRAKE_NVP(histogram_method));
    a->Visit(DRAKE_NVP(display));

    if (histogram_method == "NONE") {
      histogram = ov_core::TrackBase::NONE;
    } else if (histogram_method == "HISTOGRAM") {
      histogram = ov_core::TrackBase::HISTOGRAM;
    } else if (histogram_method == "CLAHE") {
      histogram = ov_core::TrackBase::CLAHE;
    } else {
      throw std::runtime_error(
          "must specify NONE, HISTOGRAM, or CLAHE as a histogram method\n"
      );
    }
  }
};

}
}