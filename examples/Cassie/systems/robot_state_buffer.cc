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
  size_t earliest_timestamp = full() ? (head_ + 1) % kBufSize : 0;

  std::cout << "searching for " << utime << std::endl;

  size_t start_idx = (head_ == 0) ? head_ : head_ - 1;
  for (size_t i = start_idx; i > 0; --i) {
    std::cout << "loop 1, index " << i << std::endl;
    if (timestamps_.at(i) < utime) {
      std::cout << "popping "<< timestamps_.at(i) << std::endl;
      return state_history_.at(i);
    }
  }

  if (full()) {
    for (size_t i = kBufSize - 1; i > head_; --i) {
      std::cout << "loop 2, index " << i << std::endl;
      if (timestamps_.at(i) < utime) {
        std::cout << "popping "<< timestamps_.at(i) << std::endl;
        return state_history_.at(i);
      }
    }
  }

  std::cout << "Final check, index " <<  earliest_timestamp << std::endl;
  std::cout << "popping "<< timestamps_.at(earliest_timestamp) << std::endl;
  return state_history_.at(earliest_timestamp);
}

}