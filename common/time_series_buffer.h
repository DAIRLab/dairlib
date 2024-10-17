#pragma once
#include <array>

namespace dairlib {

template <typename T, size_t BufSize>
class TimeSeriesBuffer {
 public:
  // need this line to avoid runtime warnings when storing TimeSeriesBuffer in
  // a drake Context
  using NonTypeTemplateParameter = std::integral_constant<size_t, BufSize>;

  explicit TimeSeriesBuffer() = default;

  void put(uint64_t utime, const T& state) {
    // silently reject out of order entries
    if (timestamps_[head_] >= utime) {
      return;
    }

    timestamps_[head_] = utime;
    state_history_[head_] = state;
    ++head_;
    size_ = std::min(size_ + 1, BufSize);
    if (head_ == BufSize) {
      head_ = 0;
    }
  }

  void reset() {
    head_ = 0;
    size_ = 0;
  };

  [[nodiscard]] size_t size() const { return size_; }

  [[nodiscard]] bool empty() const {return size_ == 0;}

  [[nodiscard]] bool full() const {return size_ == BufSize;}

  /// get the item corresponding to the most recent time in the buffer
  /// before the requested time stamp. If no times are earlier than the
  /// requested time, returns the earliest available time
  [[nodiscard]] const T& get(uint64_t utime) const {
    size_t earliest_timestamp = full() ? (head_ + 1) % BufSize : 0;

    size_t start_idx = (head_ == 0) ? head_ : head_ - 1;
    for (size_t i = start_idx; i > 0; --i) {
      if (timestamps_.at(i) < utime) {
        return state_history_.at(i);
      }
    }

    if (full()) {
      for (size_t i = BufSize - 1; i > head_; --i) {
        if (timestamps_.at(i) < utime) {
          return state_history_.at(i);
        }
      }
    }
    return state_history_.at(earliest_timestamp);
  }

 private:
  std::array<uint64_t, BufSize> timestamps_{};
  std::array<T, BufSize> state_history_{};

  size_t head_{}; // most recently added element
  size_t size_{};

};

}