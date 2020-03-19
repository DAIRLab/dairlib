#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/text_logging.h"
#include "examples/Cassie/datatypes/cassie_out_t.h"

namespace dairlib {

/**
 * Subscribes to and stores a copy of the most recent UDP message from Cassie.
 * This class does NOT provide any mutex behavior
 * for multi-threaded locking; it should only be used in cases where
 * Poll() and message() are called from the same thread.
 *
 * This class is a simpler (non-Drake-System) alternative to CassieUdpSubscriber
 * Poll()  and message() are meant to be called sequentially, where Poll()
 * blocks and message() retrieves a reference to the message
 */
class SimpleCassieUdpSubscriber final {
 public:
  // Not copyable, because this is not thread-safe.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleCassieUdpSubscriber)

  /**
   * Subscribes to the given address and port
   */
  SimpleCassieUdpSubscriber(const std::string& address, const int port);

  /**
   * Receives and stores the next message. This method will block until a
   * message is received. 
   */
  void Poll();

  /**
   * Returns the most recently received message, or a value-initialized (zeros)
   * message otherwise.
   */
  const cassie_out_t& message() const { return data_; }

  /** Returns the total number of received messages. */
  int64_t count() const { return count_; }

  /** 
   * Returns the time that the last message was received, relative to when
   * this class was constructed
  */
  double message_time() const { return time_; }

 private:
  // The channel on which to receive messages.
  const std::string address_;

  int socket_;
  struct sockaddr_in server_address_;
  cassie_out_t data_;
  int64_t count_;
  double time_;

  std::chrono::time_point<std::chrono::steady_clock> start_;
};

}  // namespace dairlib
