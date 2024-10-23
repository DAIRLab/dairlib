#pragma once

#include "single_rs_interface.h"

#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::perception {

class RealsenseImagePairSubscriber : public drake::systems::LeafSystem<double> {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RealsenseImagePairSubscriber);
  RealsenseImagePairSubscriber(rs2_systems::SingleRSInterface *realsense);

    void WaitForFrame() {
      std::unique_lock<std::mutex> lock(received_frame_mutex_);
      auto received = [&]() {
        return received_frames_count_ > 0;
      };
      received_frame_variable_.wait(lock, received);
    }

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
    mutable std::condition_variable received_frame_variable_;

    int received_frames_count_{0};
    rs2::frame_queue frame_queue_{2};

    rs2_systems::SingleRSInterface* rs_interface_;

    drake::systems::AbstractStateIndex frame_count_index_;
    drake::systems::AbstractStateIndex frame_pair_index_;
};

}
