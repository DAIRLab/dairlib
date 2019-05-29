#include <poll.h>
#include <sys/ioctl.h>

#include "drake/common/drake_throw.h"

#include "examples/Cassie/networking/simple_cassie_udp_subscriber.h"

namespace dairlib {

using std::chrono::duration_cast;
using std::chrono::steady_clock;
using std::chrono::microseconds;

SimpleCassieUdpSubscriber::SimpleCassieUdpSubscriber(const std::string& address,
    const int port) :
    count_(0), time_(0) {
  // Creating socket file descriptor
  // todo: check buffer size
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  DRAKE_THROW_UNLESS(socket_ >= 0);
  drake::log()->info("Opened socket!");
  memset(&server_address_, 0, sizeof(server_address_));

  // Filling server information
  inet_aton(address_.c_str(), &server_address_.sin_addr);
  server_address_.sin_family = AF_INET;  // IPv4
  server_address_.sin_port = htons(port);

  // Bind the socket with the server address
  DRAKE_THROW_UNLESS(bind(socket_, (const struct sockaddr*) &server_address_,
      sizeof(server_address_)) >= 0);
  drake::log()->info("Bound socket!");

  start_ = steady_clock::now();
}

void SimpleCassieUdpSubscriber::Poll() {
  // Create cassie output struct
  char receive_buffer[2 + CASSIE_OUT_T_LEN];

  ssize_t des_len = (sizeof receive_buffer);

  // Poll for a new packet of the correct length
  ssize_t nbytes = 0;
  struct pollfd fd = {.fd = socket_, .events = POLLIN, .revents = 0};
  do {
      poll(&fd, 1, -1);
      // Get newest valid packet in RX buffer
      // Does not use sequence number for determining newest packet
      ioctl(socket_, FIONREAD, &nbytes);
      if (des_len <= nbytes) {
        nbytes = recv(socket_, receive_buffer, des_len, 0);
      } else {
        recv(socket_, receive_buffer, 0, 0);  // Discard packet
      }
  } while (des_len != nbytes);

  time_ =
    (duration_cast<microseconds>(steady_clock::now() - start_)).count()/1.0e6;

  // Split header and data
  const unsigned char *data_in =
    reinterpret_cast<const unsigned char *>(&receive_buffer[2]);

  unpack_cassie_out_t(data_in, &data_);
  count_++;
}

}  // namespace dairlib
