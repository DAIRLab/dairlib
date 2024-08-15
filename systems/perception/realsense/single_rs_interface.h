#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

#include "librealsense2/rs.hpp"

namespace rs2_systems {

struct CassieRSFrames {
  rs2::frame color;
  rs2::frame color_aligned_depth;
  rs2::frame decimated_depth;
};

class SingleRSInterface {
 public:
  SingleRSInterface();
  ~SingleRSInterface();

  using HandlerFunction = std::function<void(const CassieRSFrames&)>;

  void Start();
  void Stop();

  void Subscribe(HandlerFunction handler_function) {
    if (run_) {
      throw std::logic_error(
          "Cannot subscribe to the realsense while it's running");
    }
    callbacks.push_back(std::move(handler_function));
  }

  std::vector<HandlerFunction> callbacks{};

 private:

  unsigned int frame_counter_{0};

  bool run_{false};

  void poll();

  rs2::pipeline pipeline_{};
  rs2::config config_{};
  rs2::align frame_aligner_{RS2_STREAM_COLOR};

  std::thread poll_thread_;
};
}