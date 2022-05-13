#include <gflags/gflags.h>

#include "dairlib/lcmt_cassie_out.hpp"
#include "examples/Cassie/networking/udp_lcm_translator.h"
#include "examples/Cassie/networking/simple_cassie_udp_subscriber.h"

#include "lcm/lcm-cpp.hpp"

#include <chrono>
#include <thread>

namespace dairlib {

DEFINE_bool(broadcast_robot_state, false, "broadcast to planner thread");

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to receive on.");
DEFINE_int64(port, 25001, "Port to receive on.");
DEFINE_double(pub_rate, 0.5, "Network LCM pubishing period (s).");
DEFINE_double(lcm_refresh_time, 60, "seconds between lcm publihser updates");
DEFINE_string(channel, "CASSIE_HEARTBEAT","lcm channel to send on");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Wait for the first message.
  SimpleCassieUdpSubscriber udp_sub(FLAGS_address, FLAGS_port);
  udp_sub.Poll();
  double t_last = udp_sub.message_time();

  auto lc = new lcm::LCM();

  while (true) {
    lcmt_cassie_out msg;
    udp_sub.Poll();
    const double time = udp_sub.message_time();
    cassieOutToLcm(udp_sub.message(), time, &msg);

    if (time - t_last >= FLAGS_lcm_refresh_time) {
      free(lc);
      lc = new lcm::LCM();
      t_last = time;
    }
    lc->publish(FLAGS_channel, &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
