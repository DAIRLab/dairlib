#pragma once

#include "opencv2/opencv.hpp"

namespace dairlib::perception {

struct ImagePair {
  uint64_t utime_;
  cv::Mat gray_;
  cv::Mat depth_;
};

}