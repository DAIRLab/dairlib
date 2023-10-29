#include "gflags/gflags.h"


namespace dairlib {

DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHED", "state lcm channel");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  return 0;
}
}

int main(int argc, char* argv[]) {
  return dairlib::DoMain(argc, argv);
}