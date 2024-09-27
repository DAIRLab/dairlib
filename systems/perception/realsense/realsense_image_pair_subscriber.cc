#include "realsense_image_pair_subscriber.h"
#include "systems/perception/image_pair.h"

#include <iostream>

namespace dairlib::perception {

using drake::Value;
using drake::systems::State;
using drake::systems::Context;
using drake::systems::EventStatus;

using rs2_systems::SingleRSInterface;


RealsenseImagePairSubscriber::RealsenseImagePairSubscriber(
    rs2_systems::SingleRSInterface *realsense) {
  DRAKE_DEMAND(realsense != nullptr);

  realsense->Subscribe(
      [this](const SingleRSInterface::rs_frames& frame) {
        this->HandleFrame(frame);
      }
  );

  frame_count_index_ = this->DeclareAbstractState(Value<int>(0));

  frame_pair_index_ = this->DeclareAbstractState(
      Value<std::shared_ptr<ImagePair>>(nullptr));

  this->DeclareStateOutputPort(
      drake::systems::kUseDefaultName, frame_pair_index_);

  this->DeclareForcedUnrestrictedUpdateEvent(
      &RealsenseImagePairSubscriber::ProcessFrameAndStoreToAbstractState);

  this->DeclareInitializationUnrestrictedUpdateEvent(
      &RealsenseImagePairSubscriber::Initialize);

  this->set_name("realsense_image_pair_subscriber");
}

void RealsenseImagePairSubscriber::SetDefaultState(
    const Context<double>& context, State<double>* state) const {

  auto& pc_ptr = state->get_mutable_abstract_state<
  std::shared_ptr<ImagePair>>(frame_pair_index_);
  pc_ptr = std::make_shared<ImagePair>();

}

void RealsenseImagePairSubscriber::DoCalcNextUpdateTime(
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
    const auto& self = dynamic_cast<const RealsenseImagePairSubscriber&>(system);
    return self.ProcessFrameAndStoreToAbstractState(callback_context, callback_state);
  };

  *time = context.get_time();
  drake::systems::EventCollection<drake::systems::UnrestrictedUpdateEvent<double>>& uu_events =
      events->get_mutable_unrestricted_update_events();
  uu_events.AddEvent(
      drake::systems::UnrestrictedUpdateEvent<double>(
          drake::systems::TriggerType::kTimed, callback));
}


void RealsenseImagePairSubscriber::HandleFrame(
    const rs2_systems::SingleRSInterface::rs_frames &frame) {
  frame_queue_.enqueue(frame.color_depth_pair);

  std::scoped_lock<std::mutex> lock(received_frame_mutex_);
  received_frames_count_++;
  received_frame_variable_.notify_one();
}


EventStatus RealsenseImagePairSubscriber::ProcessFrameAndStoreToAbstractState(
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

  rs2::frameset frameset;
  frame_queue_.poll_for_frame(&frameset);
  rs2::depth_frame depth_frame = frameset.get_depth_frame();
  rs2::video_frame color_frame = frameset.get_color_frame();

  const int width = depth_frame.get_width();
  const int height = depth_frame.get_height();

  ImagePair& image_pair = *(
      state->get_mutable_abstract_state<std::shared_ptr<ImagePair>>(frame_pair_index_));

  cv::Mat color_image(height, width, CV_8UC3, (void*)color_frame.get_data());
  cv::cvtColor(color_image, image_pair.gray_, cv::COLOR_RGB2GRAY);
  image_pair.depth_ = cv::Mat(height, width, CV_16UC1, (void*)depth_frame.get_data());

  cv::imshow("depth", image_pair.depth_);
  cv::imshow("gray", image_pair.gray_);
  cv::waitKey(1);

  // ms to us
  uint64_t now = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
  uint64_t frame_time = 1e3 * depth_frame.get_timestamp();

  image_pair.utime_= std::max<uint64_t>(
      0, static_cast<uint64_t>(context.get_time() * 1e6) - (now - frame_time)
  );

  return EventStatus::Succeeded();
}

EventStatus RealsenseImagePairSubscriber::Initialize(
    const drake::systems::Context<double>& context,
    drake::systems::State<double>* state) const {
  return EventStatus::Succeeded();
}
}

