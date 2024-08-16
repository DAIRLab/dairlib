#include "time_series_buffer.h"

#include "src/InEKF.h"
#include <Eigen/Dense>

namespace dairlib {

using inekf::RobotState;

template <typename T>
void TimeSeriesBuffer<T>::put(long utime, const T& state) {
  // silently reject out of order entries
  if (timestamps_[head_] >= utime) {
    return;
  }

  timestamps_[head_] = utime;
  state_history_[head_] = state;
  ++head_;
  size_ = std::min(size_ + 1, kBufSize);
  if (head_ == kBufSize) {
    head_ = 0;
  }
}

template <typename T>
const T& TimeSeriesBuffer<T>::get(uint64_t utime) const {
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

template class TimeSeriesBuffer<inekf::RobotState>;
template class TimeSeriesBuffer<Eigen::VectorXd>;

}