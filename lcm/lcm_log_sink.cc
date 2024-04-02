#include "lcm_log_sink.h"

#include <memory>
#include <chrono>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include "lcm/lcm.h"

#include "drake/common/drake_assert.h"

namespace dairlib {
namespace lcm {

using drake::lcm::DrakeLcmInterface;
using drake::lcm::DrakeSubscriptionInterface;

using HandlerFunction = DrakeLcmInterface::HandlerFunction;
using MultichannelHandlerFunction =
    DrakeLcmInterface::MultichannelHandlerFunction;

class LcmLogSink::Impl {
 public:
  std::vector<::lcm_eventlog_event_t> event_buf_{};

  void Append(const std::string& channel, const void* data,
              int data_size, unsigned long timestamp) {
    ::lcm_eventlog_event_t log_event{};
    log_event.timestamp = timestamp;
    log_event.channellen = channel.size();
    log_event.channel = static_cast<char*>(malloc(sizeof(char) * (channel.size() + 1)));
    DRAKE_DEMAND(log_event.channel != nullptr);
    strcpy(log_event.channel, channel.c_str());
    log_event.datalen = data_size;
    log_event.data = malloc(data_size);
    DRAKE_DEMAND(log_event.data != nullptr);
    memcpy(log_event.data, data, data_size);
    event_buf_.push_back(log_event);
  }

  void WriteLog(const std::string& filename) {
    lcm_eventlog_t* log = ::lcm_eventlog_create(filename.c_str(), "w");
    if (log == nullptr) {
      throw std::logic_error("Couldn't create lcm log " + filename);
    }
    for (::lcm_eventlog_event_t& event: event_buf_) {
      int status = ::lcm_eventlog_write_event(log, &event);
      if (status != 0) {
        throw std::logic_error("Message write failure");
      }
    }
    ::lcm_eventlog_destroy(log);
  }

  void FreeBuf() {
    for (auto& e: event_buf_) {
      if (e.channel != nullptr) {
        free(e.channel);
        e.channel = nullptr;
      }
      if (e.data != nullptr) {
        free(e.data);
        e.data  = nullptr;
      }
    }
  }
  ~Impl() { FreeBuf(); }
};

LcmLogSink::LcmLogSink(bool overwrite_publish_time_with_system_clock):
      overwrite_publish_time_with_system_clock_(
          overwrite_publish_time_with_system_clock),
      url_("lcmlogsink//:"),
      impl_(new Impl) {}

LcmLogSink::~LcmLogSink() = default;

std::string LcmLogSink::get_lcm_url() const {
  return url_;
}

void LcmLogSink::Publish(const std::string& channel, const void* data,
                          int data_size, std::optional<double> time_sec) {
  unsigned long timestamp;
  if (!overwrite_publish_time_with_system_clock_) {
    timestamp = second_to_timestamp(time_sec.value_or(0.0));
  } else {
    timestamp = std::chrono::steady_clock::now().time_since_epoch() /
        std::chrono::microseconds(1);
  }
  impl_->Append(channel, data, data_size, timestamp);
}

void LcmLogSink::WriteLog(const std::string &fname) {
  impl_->WriteLog(fname);
}

std::shared_ptr<DrakeSubscriptionInterface> LcmLogSink::Subscribe(
    const std::string& channel, HandlerFunction handler) {
  throw std::logic_error("You are trying to subscribe to an LCM channel using"
                         " LcmLogSink, which is a write only implementation of "
                         "DrakeLcmInterface.");
  return nullptr;
}

std::shared_ptr<DrakeSubscriptionInterface> LcmLogSink::SubscribeMultichannel(
    std::string_view /* regex */, MultichannelHandlerFunction /* handler */) {
  throw std::logic_error("You are trying to subscribe to an LCM channel using"
                         " LcmLogSink, which is a write only implementation of "
                         "DrakeLcmInterface.");
  return nullptr;
}

std::shared_ptr<DrakeSubscriptionInterface> LcmLogSink::SubscribeAllChannels(
    MultichannelHandlerFunction handler) {
  throw std::logic_error("You are trying to subscribe to an LCM channel using"
                         " LcmLogSink, which is a write only implementation of "
                         "DrakeLcmInterface.");
  return nullptr;
}

int LcmLogSink::HandleSubscriptions(int) {
  throw std::logic_error("You are trying to subscribe to an LCM channel using"
                         " LcmLogSink, which is a write only implementation of "
                         "DrakeLcmInterface.");
  return 0;
}

void LcmLogSink::OnHandleSubscriptionsError(const std::string& error_message) {
  // We are not called via LCM C code, so it's safe to throw there.
  throw std::runtime_error(error_message);
}

}  // namespace lcm
}  // namespace drake