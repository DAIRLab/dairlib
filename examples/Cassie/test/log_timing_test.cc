#include <iostream>
#include <Eigen/Dense>
#include <gflags/gflags.h>
#include "lcm/lcm-cpp.hpp"

DEFINE_string(file, "", "Log file name.");
DEFINE_int64(max_count, 5000, "Max number of messages to read.");

// A quick script to test the frequency of messages in an LCM log.
// Intended to be run with a log that has only one channel in it. Could
// easily be modified to first check against a desired channel name, see
// dispatcher_log_timing_test.cc for an example of this.
// Prints out the mean and standard deviation of the delta-times
int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  lcm::LogFile log(FLAGS_file, "r");
  
  auto event = log.readNextEvent();

  double time = event->timestamp;
  double last_time = time;
  event = log.readNextEvent();
  time = event->timestamp;

  Eigen::VectorXd times(FLAGS_max_count);
  Eigen::VectorXd dt(FLAGS_max_count);
  int count = 0;

  while (event != NULL &&
         count < FLAGS_max_count) {
    time = event->timestamp;
    times(count) = time;
    dt(count) = time - last_time;
    last_time = time;
    count++;
    event = log.readNextEvent();
  }

  if (count < FLAGS_max_count) {
    times.conservativeResize(count);
    dt.conservativeResize(count);
  }

  auto std_dev = std::sqrt(
      (dt.rowwise() - dt.colwise().mean()).squaredNorm()/(dt.size() - 1));
  std::cout << "dt (microseconds) mean: " << dt.mean() << " std_dev: " <<
      std_dev << std::endl;
}
