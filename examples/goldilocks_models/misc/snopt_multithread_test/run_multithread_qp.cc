#include <gflags/gflags.h>
#include <stdio.h>  // For removing files
#include <thread>  // multi-threading
#include <chrono>
#include <ctime>

#include "examples/goldilocks_models/misc/snopt_multithread_test/run_sample_qp.h"
#include "systems/goldilocks_models/file_utils.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/multibody/parsing/parser.h"

#include "common/find_resource.h"

using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::to_string;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXcd;
using Eigen::MatrixXd;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::Parser;
using drake::AutoDiffXd;
using dairlib::FindResourceOrThrow;

namespace dairlib {
namespace goldilocks_models {
namespace misc {

int runMultithreadQp(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Create MBP
  MultibodyPlant<double> plant;
  Parser parser(&plant);
  std::string full_name = FindResourceOrThrow(
                            "examples/goldilocks_models/PlanarWalkerWithTorso.urdf");
  parser.AddModelFromFile(full_name);
  plant.AddForceElement<drake::multibody::UniformGravityFieldElement>(
    -9.81 * Eigen::Vector3d::UnitZ());
  plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("base"),
    drake::math::RigidTransform<double>());
  plant.Finalize();

  // Create autoDiff version of the plant
  MultibodyPlant<AutoDiffXd> plant_autoDiff(plant);

  // Files parameters
  const string dir =
    "examples/goldilocks_models/misc/snopt_multithread_test/data/";
  string prefix = "";

  // Parameters
  int n_sample = 3;
  int max_outer_iter = 10;


  // Vectors/Matrices for the outer loop
  vector<VectorXd> c_vec;

  cout << "Start the iterating...\n";
  // Start the gradient descent
  int iter;
  for (iter = 0; iter <= max_outer_iter; iter++)  {
    // Print info about iteration # and current time
    auto end = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    cout << "Current time: " << std::ctime(&end_time);
    cout << "************ Iteration " << iter << " *************" << endl;

    // Clear the vectors/matrices before trajectory optimization
    c_vec.clear();

    bool samples_are_success = true;
    bool a_sample_is_success = false;

    // Create vector of threads for multithreading
    std::vector<std::thread> threads;
    for (int sample = 0; sample < n_sample; sample++) {
      // Trajectory optimization with fixed model paramters
      prefix = to_string(iter) +  "_" + to_string(sample) + "_";

      cout << "add task to thread\n";
      threads.push_back(std::thread(runSampleQp,
                                    dir, prefix));
      cout << "Finished adding task to thread\n";
    }  // for(int sample...)

    for (int sample = 0; sample < n_sample; sample++) {
      threads[sample].join();
      // delete threads[sample];

      prefix = to_string(iter) +  "_" + to_string(sample) + "_";
      int sample_success =
        (readCSV(dir + prefix + string("is_success.csv")))(0, 0);
      samples_are_success = (samples_are_success & (sample_success == 1));
      a_sample_is_success = (a_sample_is_success | (sample_success == 1));
    }  // for(int sample...)

    // Read in c_vec;
    for (int sample = 0; sample < n_sample; sample++) {
      prefix = to_string(iter) +  "_" + to_string(sample) + "_";
      VectorXd success =
        readCSV(dir + prefix + string("is_success.csv")).col(0);
      if (success(0)) {
        c_vec.push_back(readCSV(dir + prefix + string("c.csv")));
      }
    }

    int n_succ_sample = c_vec.size();
    // Get the total cost if it's successful
    double total_cost = 0;
    for (int sample = 0; sample < n_succ_sample; sample++)
      total_cost += c_vec[sample](0) / n_succ_sample;
    cout << "total_cost = " << total_cost << "\n\n";

  }  // end for


  return 0;
}  // int runMultithreadQp

}  // namespace misc
}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::misc::runMultithreadQp(argc, argv);
}
