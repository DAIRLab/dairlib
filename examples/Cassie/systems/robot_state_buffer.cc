#include "robot_state_buffer.h"

namespace dairlib {

using inekf::RobotState;

void RobotStateBuffer::put(long utime, const RobotState &state) {
  // TODO (@Brian-Acosta) validate timestamps
  timestamps_[head_] = utime;
  state_history_[head_] = state;
  ++head_;
  size_ = std::min(size_ + 1, kBufSize);
  if (head_ == kBufSize) {
    head_ = 0;
  }
}

const inekf::RobotState& RobotStateBuffer::get(uint64_t utime) const {
  size_t earliest_timestamp = full() ? (head_ + 1) % kBufSize : 0;

  size_t start_idx = (head_ == 0) ? head_ : head_ - 1;
  for (size_t i = start_idx; i > 0; --i) {
    if (timestamps_.at(i) < utime) {
      return state_history_.at(i);
    }
  }

  if (full()) {
    for (size_t i = kBufSize - 1; i > head_; --i) {
      if (timestamps_.at(i) < utime) {
        return state_history_.at(i);
      }
    }
  }
  return state_history_.at(earliest_timestamp);
}

}