#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"

namespace dairlib {
namespace lcm {

/**
 * A LCM interface for logging LCM messages to a file. Contains an internal
 * buffer used to store lcm_eventlog events until the user calls WriteLog to
 * write an lcm log to disk. Reimplements `DrakeLcmLog` as a write only
 * interface and avoids writing to disk on every call to Publish().
 */
 class LcmLogSink : public drake::lcm::DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmLogSink);

  /**
   * Constructs an LcmLogSink.
   * @param overwrite_publish_time_with_system_clock If true, override the
   * `second` parameter passed to Publish method, and use host system's clock
   * to generate the timestamp for the logged message. This is used to mimic
   * lcm-logger's behavior. It also implicitly records how fast the messages
   * are generated in real time.
   *
   */
  LcmLogSink(bool overwrite_publish_time_with_system_clock = false);

  ~LcmLogSink() override;

  /**
   * Writes an entry that occurred at @p timestamp with content @p data to the
   * internal event log buffer.
   * @param channel Channel name.
   * @param data Pointer to raw bytes.
   * @param data_size Number of bytes in @p data.
   * @param time_sec Time in seconds when the message is published. Since
   * messages are save to the log file in the order of Publish calls, this
   * function should only be called with non-decreasing @p second. Note that
   * this parameter can be overwritten by the host system's clock if
   * `overwrite_publish_time_with_system_clock` is true at construction time.
   *
   * @throws std::exception if a new event cannot be allocated to insert into
   * the buffer
   */
  void Publish(const std::string& channel, const void* data, int data_size,
               std::optional<double> time_sec) override;

  /**
   * Write the contents of this LcmLogSink to disk as an lcm log.
   * @param fname the filename of the lcmlog to write.
   */
  void WriteLog(const std::string& fname);

  /**
   * Erase the contents of this LcmLogSink
   */
  void clear();

  /**
   * Throws an exception because LcmLogSink is write-only
   */
  std::shared_ptr<drake::lcm::DrakeSubscriptionInterface> Subscribe(
      const std::string& channel, HandlerFunction handler) override;

  /** Throws an exception because LcmLogSink is write-only */
  std::shared_ptr<drake::lcm::DrakeSubscriptionInterface> SubscribeMultichannel(
      std::string_view regex, MultichannelHandlerFunction) override;

  /**
   * Throws an exception because LcmLogSink is write-only.
   */
  std::shared_ptr<drake::lcm::DrakeSubscriptionInterface> SubscribeAllChannels(
      MultichannelHandlerFunction) override;

  /**
   * Throws an exception because LcmLogSink is write-only
   */
  int HandleSubscriptions(int) override;


  /**
   * Converts @p timestamp (in microseconds) to time (in seconds) relative to
   * the starting time passed to the constructor.
   */
  double timestamp_to_second(uint64_t timestamp) const {
    return static_cast<double>(timestamp) / 1e6;
  }

  /**
   * Converts time (in seconds) relative to the starting time passed to the
   * constructor to a timestamp in microseconds.
   */
  uint64_t second_to_timestamp(double sec) const {
    return static_cast<uint64_t>(sec * 1e6);
  }

  std::string get_lcm_url() const override;

 private:
  void OnHandleSubscriptionsError(const std::string&) override;

  const bool overwrite_publish_time_with_system_clock_;
  const std::string url_;

  // TODO(jwnimmer-tri) It is not clear to me why this class needs a mutex
  // (i.e., where multiple threads are coming from).  That factor needs to be
  // re-discovered and then documented somewhere.

  // This mutex guards access to the Impl object.
  mutable std::mutex mutex_;
  class Impl;
  const std::unique_ptr<Impl> impl_;
};

}  // namespace lcm
}  // namespace drake