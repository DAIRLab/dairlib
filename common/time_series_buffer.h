#pragma once
#include <array>

namespace dairlib {

template <typename T>
class TimeSeriesBuffer {
 public:
  explicit TimeSeriesBuffer() = default;
  void put(long utime, const T& state);

  void reset() {
    head_ = 0;
    size_ = 0;
  };

  [[nodiscard]] size_t size() const { return size_; }

  [[nodiscard]] bool empty() const {return size_ == 0;}

  [[nodiscard]] bool full() const {return size_ == kBufSize;}

  /// get the item corresponding to the most recent time in the buffer
  /// before the requested time stamp. If no times are earlier than the
  /// requested time, returns the earliest available time
  [[nodiscard]] const T& get(uint64_t utime) const;

  static constexpr size_t kBufSize = 500;

 private:
  std::array<uint64_t, kBufSize> timestamps_{};
  std::array<T, kBufSize> state_history_{};

  size_t head_{}; // most recently added element
  size_t size_{};

};

}