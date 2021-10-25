#include <random>
#include <gflags/gflags.h>

#include "drake/multibody/plant/multibody_plant.h"

#include "cassie_utils.h"
#include "cassie_fixed_point_solver.h"
#include "common/file_utils.h"

namespace dairlib {

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::default_random_engine;
using std::uniform_real_distribution;

DEFINE_bool(floating_base, true, "Fixed or floating base model");
DEFINE_bool(spring_model, true, "Use a URDF with or without legs springs");
DEFINE_double(mean_height, .75,"mean of initial pelvis heigh");
DEFINE_double(var_height, 0.25, "variance of initial pelvis height");
DEFINE_double(mean_toe_spread, 0.15, "mean of initial toe spread");
DEFINE_double(var_toe_spread, 0.075, "variance of initial toe spread");
DEFINE_int32(n_bin, 20, "number of bins for gridding approach");
DEFINE_string(savefile, "../cassie_initial_conditions.csv",
    "File to save ics to");


int get_random_ics_main(int argc, char*argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string urdf;
  if (FLAGS_spring_model) {
    urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  } else {
    urdf = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  }
  // Set initial conditions of the simulation
  VectorXd q_init, u_init, lambda_init;
  q_init = VectorXd::Zero(1);
  double mu_fp = 0;
  double min_normal_fp = 70;

  drake::multibody::MultibodyPlant<double> plant(0.0);
  addCassieMultibody(&plant, nullptr,
                     FLAGS_floating_base /*floating base*/, urdf,
                     FLAGS_spring_model, true);
  plant.Finalize();

  MatrixXd IC_sols = MatrixXd::Zero(
      plant.num_positions() + plant.num_velocities(),
      FLAGS_n_bin * FLAGS_n_bin);

  for (int i = 0 ; i < FLAGS_n_bin; i++) {
    for (int j = 0 ; j < FLAGS_n_bin; j++) {

      double init_height = FLAGS_mean_height +
          (2.0 * ((double) i) / ((double) FLAGS_n_bin)  - 1.0) * FLAGS_var_height;
      double toe_spread = FLAGS_mean_toe_spread +
          (2.0 * ((double) j) / ((double) FLAGS_n_bin)  - 1.0) * FLAGS_var_toe_spread;

      if (FLAGS_floating_base) {
        CassieFixedPointSolver(plant, init_height, mu_fp, min_normal_fp,
                               true, toe_spread, &q_init,
                               &u_init, &lambda_init);
      } else {
        CassieFixedBaseFixedPointSolver(plant, &q_init, &u_init, &lambda_init);
      }
      VectorXd x_init =
          VectorXd::Zero(plant.num_positions() + plant.num_velocities());
      x_init.head(plant.num_positions()) = q_init;
      IC_sols.col(i * FLAGS_n_bin + j) = x_init;
      std::cout << std::to_string(i) << std::endl;
    }
  }
  writeCSV(FLAGS_savefile, IC_sols);
  return 0;
}
}

int main(int argc, char* argv[]) {
  return dairlib::get_random_ics_main(argc, argv);
}