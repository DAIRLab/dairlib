#include <iostream>
#include <Eigen/Dense>
#include <gflags/gflags.h>
#include "lcm/lcm-cpp.hpp"

#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_output.hpp"

DEFINE_string(file, "", "Log file name.");
DEFINE_int64(max_count, 5000, "Max number of messages to read.");

// A quick script to evaluate the delay caused by dispatcher_robot_out.
// Measures the difference between the logged time and the lcm utime field
// for bot CASSIE_OUTPUT (into the dispatcher) and CASSIE_STATE (out of the
// dispatcher). The difference between these two deltas is roughly the delay
// in the dispatcher. This approach to calculating delay does not require
// finding the one-to-one matches between messages on each channel, but instead
// relies on the fact that the message utime field should be identical.
int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  lcm::LogFile log(FLAGS_file, "r");

  auto event = log.readNextEvent();

  int count = 0;

  int count_cassie_output = 0;
  Eigen::VectorXd diff_cassie_output(FLAGS_max_count);

  int count_cassie_state = 0;
  Eigen::VectorXd diff_cassie_state(FLAGS_max_count);

  while (event != NULL &&
         count < FLAGS_max_count) {
     count++;

    if (event->channel == "CASSIE_OUTPUT") {
      dairlib::lcmt_cassie_out msg;
      msg.decode(event->data, 0, event->datalen);
      diff_cassie_output(count_cassie_output) = event->timestamp - msg.utime;
      count_cassie_output++;
    }

    if (event->channel == "CASSIE_STATE") {
      dairlib::lcmt_robot_output msg;
      msg.decode(event->data, 0, event->datalen);
      diff_cassie_state(count_cassie_state) = event->timestamp - msg.utime;
      count_cassie_state++;
    }

    event = log.readNextEvent();
  }

  diff_cassie_output.conservativeResize(count_cassie_output);
  diff_cassie_state.conservativeResize(count_cassie_state);

  std::cout << "CASSIE_OUTPUT (timestamp - utime) mean: " <<
      diff_cassie_output.mean() << std::endl;
  std::cout << "CASSIE_STATE (timestamp - utime) mean: " <<
      diff_cassie_state.mean() << std::endl;
  std::cout << "Difference of means (delay estimate, us): " <<
      diff_cassie_state.mean() - diff_cassie_output.mean() << std::endl;
}
