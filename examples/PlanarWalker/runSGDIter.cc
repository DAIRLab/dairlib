#include "sgd_iter.h"
#include <gflags/gflags.h>

DEFINE_double(length, 0.5, "The stride length.");
DEFINE_double(duration, 1, "The stride duration");
DEFINE_int64(iter, 200, "Number of iterations");
DEFINE_string(dir, "data/", "Save directory");
DEFINE_string(init, "z_save.csv", "File name for initial guess");
DEFINE_string(weights, "theta.csv", "File name for weights guess");
DEFINE_string(prefix, "", "Output prefix for results");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::goldilocks_models::sgdIter(FLAGS_length, FLAGS_duration, FLAGS_iter, 
    FLAGS_dir, FLAGS_init, FLAGS_weights, FLAGS_prefix);
}