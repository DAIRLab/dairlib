#include "single_rs_interface.h"

namespace rs2_systems {

SingleRSInterface::SingleRSInterface() {
  config_.enable_stream(
      RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  config_.enable_stream(
      RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
}

void SingleRSInterface::Start() {
  // if already running, do nothing
  if (run_) {
    return;
  }
  run_ = true;
  pipeline_.start(config_);
  poll_thread_ = std::thread(&SingleRSInterface::poll, this);
}

void SingleRSInterface::Stop() {
  run_ = false;

  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }

  try {
    pipeline_.stop();
  } catch (rs2::wrong_api_call_sequence_error& e) {
    // make stop() essentially a no-op if pipeline is stopped
  }

}

void SingleRSInterface::poll() {
  rs2::frameset frameset;
  rs2::frameset aligned;
  rs2::decimation_filter decimation;
  decimation.set_option(RS2_OPTION_FILTER_MAGNITUDE, 4.0f);

  rs_frames frames;

  while (run_) {
    frameset = pipeline_.wait_for_frames();
    frames.color = frameset.get_color_frame();
    aligned = frame_aligner_.process(frameset);
    frames.color_aligned_depth = aligned.get_depth_frame();
    frames.decimated_depth = decimation.process(frameset.get_depth_frame());

    for (const auto& callback : callbacks) {
      callback(frames);
    }
  }
}

SingleRSInterface::~SingleRSInterface() {
 this->Stop();
}



}