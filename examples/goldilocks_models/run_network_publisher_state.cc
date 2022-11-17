#include <string>

#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "examples/goldilocks_models/controller/network_publisher.h"

/// This script is for improving the communication efficiency between cassie
/// onboard computer and the offboard station computer, by preventing from
/// spamming messages all the time even though the planner is still solving.
/// However, if I remember correctly, I haven't got this to work (due to racing
/// condition?)
/// This script takes care of sending robot state message from dispatcher_out to
/// the planner (which runs on the station), while the other script takes care
/// of sending the messages from controller thread to the planner (which runs on
/// the station).

namespace dairlib {

DEFINE_string(channel_in, "CASSIE_STATE_DISPATCHER", "");
DEFINE_string(channel_out, "NETWORK_CASSIE_STATE_DISPATCHER", "");
DEFINE_int32(n_publishes, 3, "number of publishes after getting a message");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  goldilocks_models::NetworkPublisher<dairlib::lcmt_robot_output>(
      FLAGS_channel_in, FLAGS_channel_out, FLAGS_n_publishes);

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
