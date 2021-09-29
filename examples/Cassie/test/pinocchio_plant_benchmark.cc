#include <chrono>
#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "drake/common/find_resource.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "multibody/pinocchio_plant.h"

using drake::multibody::MultibodyPlant;

using dairlib::multibody::PinocchioPlant;
using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace drake {
namespace examples {
namespace {

typedef std::chrono::steady_clock my_clock;

int do_main() {
  const int num_reps = 10000 ;
  const int num_autodiff_reps = 100;

  //
  // Build and test pinocchio plant
  //
  std::string urdf =
      dairlib::FindResourceOrThrow("examples/Cassie/urdf/cassie_v2.urdf");
  systems::DiagramBuilder<double> builder;
  PinocchioPlant<double>& plant =
      *builder.AddSystem<PinocchioPlant>(0, urdf);

  multibody::Parser parser(&plant);
  parser.AddModelFromFile(urdf);

  plant.WeldFrames(plant.world_frame(),
                             plant.GetFrameByName("pelvis"));
  plant.Finalize();

  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nu = plant.num_actuators();

  VectorXd x = VectorXd::Zero(nq + nv);
  VectorXd u = VectorXd::Zero(nu);

  auto context = plant.CreateDefaultContext();

  auto start = my_clock::now();
  MatrixXd M = MatrixXd(nv, nv);
  for (int i = 0; i < num_reps; i++) {
    x(0) = i;
    plant.SetPositionsAndVelocities(context.get(), x);
    plant.CalcMassMatrix(*context, &M);
  }
  auto stop = my_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(pinocchio_plant) " << std::to_string(num_reps)
            << "x inertia calculations took " << duration.count()
            << " miliseconds. " <<  1e6 * duration.count() / num_reps
            << " nanoseconds per." << std::endl;

  start = my_clock::now();
  for (int i = 0; i < num_reps; i++) {
    x(0) = i;
    plant.SetPositionsAndVelocities(context.get(), x);
    plant.MultibodyPlant::CalcMassMatrix(*context, &M);
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_reps)
            << "x inertia calculations took " << duration.count()
            << " miliseconds. " << 1e6 * duration.count() / num_reps
            << " nanoseconds per." << std::endl;

  VectorXd vdot;
  drake::multibody::MultibodyForces<double> forces(plant);
  start = my_clock::now();
  for (int i = 0; i < num_reps; i++) {
    x(0) = i;
    vdot = VectorXd::Random(nv);
    plant.SetPositionsAndVelocities(context.get(), x);

    plant.CalcInverseDynamics(*context, vdot, forces);
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(pinocchio_plant) " << std::to_string(num_reps)
            << "x inverse dynamics calculations took " << duration.count()
            << " miliseconds. " << 1e6 * duration.count() / num_reps
            << " nanoseconds per." << std::endl;


  start = my_clock::now();
  for (int i = 0; i < num_reps; i++) {
    x(0) = i;
    vdot = VectorXd::Random(nv);
    plant.SetPositionsAndVelocities(context.get(), x);

    plant.MultibodyPlant::CalcInverseDynamics(*context, vdot, forces);
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_reps)
            << "x inverse dynamics calculations took " << duration.count()
            << " miliseconds. " << 1e6 * duration.count() / num_reps
            << " nanoseconds per." << std::endl;


  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}