/*
Benchmark done on Jan 24th, 2022.

Units below are [seconds per 100,000 calls]

Without resetting drake context every loop:
r_com time (Pinocchio):0.0406533
r_com time (Drake):0.0244223
0.600745times faster
v_com time (Pinocchio):0.0578227
v_com time (Drake):0.0902847
1.56141times faster
J_com time (Pinocchio):0.0692747
J_com time (Drake):0.602594
8.69861times faster

When resetting context in every loop:
r_com time (Pinocchio):0.0988584
r_com time (Drake):0.369517
3.73784times faster
v_com time (Pinocchio):0.137124
v_com time (Drake):0.711533
5.18897times faster
J_com time (Pinocchio):0.148367
J_com time (Drake):1.55255
10.4642times faster

 */

#include <fstream>
#include <string>

#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "multibody/pinocchio_plant.h"

#include "drake/multibody/parsing/parser.h"

namespace dairlib::goldilocks_models {

using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::MatrixX;
using drake::VectorX;

// Simulated robot
DEFINE_bool(spring_model, true, "Use a URDF with or without legs springs");
DEFINE_bool(ball, true, "");
DEFINE_bool(floating_base, true, "");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Set urdf
  std::string urdf;
  if (FLAGS_ball) {
    urdf = "multibody/ball.urdf";
  } else {
    urdf = FLAGS_spring_model
               ? "examples/Cassie/urdf/cassie_v2.urdf"
               : "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  }

  // Build MBPs
  dairlib::multibody::PinocchioPlant<double> plant_pino(0.0, urdf);
  if (FLAGS_ball) {
    drake::multibody::Parser parser(&plant_pino);
    parser.AddModelFromFile(FindResourceOrThrow(urdf));
  } else {
    addCassieMultibody(&plant_pino, nullptr, FLAGS_floating_base, urdf, false,
                       false, false /*add_reflected_inertia*/);
  }
  plant_pino.Finalize();
  const auto& world = plant_pino.world_frame();

  drake::multibody::MultibodyPlant<double> plant(0.0);
  if (FLAGS_ball) {
    drake::multibody::Parser parser(&plant);
    parser.AddModelFromFile(FindResourceOrThrow(urdf));
  } else {
    addCassieMultibody(&plant, nullptr, FLAGS_floating_base, urdf, false, false,
                       false);
  }
  plant.Finalize();
  const auto& world_drake = plant.world_frame();

  //
  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nx = nq + nv;
  VectorXd q = VectorXd::Zero(nq);
  if (FLAGS_floating_base) q(0) = 1;
  VectorXd v = VectorXd::Zero(nv);
  VectorXd x(nx);
  x << q, v;

  // benchmark
  auto context_pino = plant_pino.CreateDefaultContext();
  plant_pino.SetPositionsAndVelocities(context_pino.get(), x);
  auto context_drake = plant.CreateDefaultContext();
  plant.SetPositionsAndVelocities(context_drake.get(), x);

  int n_loop = 100000;

  Eigen::Vector3d r_com;
  auto break2 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < n_loop; i++) {
    plant_pino.SetPositions(context_pino.get(), q);
    plant_pino.CalcCenterOfMassPositionInWorld(*context_pino);
  }
  auto break3 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_pino = break3 - break2;
  cout << "r_com time (Pinocchio):" << elapsed_pino.count() << "\n";

  break2 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < n_loop; i++) {
    plant.SetPositions(context_drake.get(), q);
    plant.CalcCenterOfMassPositionInWorld(*context_drake);
  }
  break3 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = break3 - break2;
  cout << "r_com time (Drake):" << elapsed.count() << "\n";
  cout << elapsed.count() / elapsed_pino.count() << "times faster\n";

  Eigen::Vector3d v_com;
  break2 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < n_loop; i++) {
    plant_pino.SetPositionsAndVelocities(context_pino.get(), x);
    plant_pino.CalcCenterOfMassTranslationalVelocityInWorld(*context_pino);
  }
  break3 = std::chrono::high_resolution_clock::now();
  elapsed_pino = break3 - break2;
  cout << "v_com time (Pinocchio):" << elapsed_pino.count() << "\n";

  break2 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < n_loop; i++) {
    plant.SetPositionsAndVelocities(context_drake.get(), x);
    plant.CalcCenterOfMassTranslationalVelocityInWorld(*context_drake);
  }
  break3 = std::chrono::high_resolution_clock::now();
  elapsed = break3 - break2;
  cout << "v_com time (Drake):" << elapsed.count() << "\n";
  cout << elapsed.count() / elapsed_pino.count() << "times faster\n";

  Eigen::MatrixXd J_com(3, nv);
  break2 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < n_loop; i++) {
    plant_pino.SetPositions(context_pino.get(), q);
    plant_pino.CalcJacobianCenterOfMassTranslationalVelocity(
        *context_pino, drake::multibody::JacobianWrtVariable::kV, world, world,
        &J_com);
  }
  break3 = std::chrono::high_resolution_clock::now();
  elapsed_pino = break3 - break2;
  cout << "J_com time (Pinocchio):" << elapsed_pino.count() << "\n";

  break2 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < n_loop; i++) {
    plant.SetPositions(context_drake.get(),
                       VectorXd::Zero(plant.num_positions()));
    plant.CalcJacobianCenterOfMassTranslationalVelocity(
        *context_drake, drake::multibody::JacobianWrtVariable::kV, world_drake,
        world_drake, &J_com);
  }
  break3 = std::chrono::high_resolution_clock::now();
  elapsed = break3 - break2;
  cout << "J_com time (Drake):" << elapsed.count() << "\n";
  cout << elapsed.count() / elapsed_pino.count() << "times faster\n";

  return 0;
}

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::DoMain(argc, argv);
}
