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

  std::cout << "pushing " << utime << std::endl;
}

const inekf::RobotState& RobotStateBuffer::get(uint64_t utime) const {
  size_t earliest_timestamp = full() ? head_ + 1 % 100 : 0;

  for (size_t i = head_; i > 0; --i) {
    if (timestamps_.at(i) < utime) {
      std::cout << "popping "<< timestamps_.at(i) << std::endl;
      return state_history_.at(i);
    }
  }

  if (full()) {
    for (size_t i = kBufSize; i > head_; --i) {
      if (timestamps_.at(i) < utime) {
        std::cout << "popping "<< timestamps_.at(i) << std::endl;
        return state_history_.at(i);
      }
    }
  }

  std::cout << "popping "<< timestamps_.at(earliest_timestamp) << std::endl;
  return state_history_.at(earliest_timestamp);
}

}