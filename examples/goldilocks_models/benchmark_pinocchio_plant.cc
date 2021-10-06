/*

Units below are [seconds per 1,000,000 calls]

Without resetting drake context every loop:
r_com time (Pinocchio):0.833135
r_com time (Drake):0.63641
v_com time (Pinocchio):1.38537
v_com time (Drake):1.50729

When resetting context in every loop:
r_com time (Pinocchio):1.09758
r_com time (Drake):3.59607
v_com time (Pinocchio):1.63794
v_com time (Drake):6.92368
J_com time (Pinocchio):1.29014
J_com time (Drake):13.6425

 */

#include <fstream>
#include <string>

#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "multibody/pinocchio_plant.h"

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

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // Build Cassie MBP
  std::string urdf = FLAGS_spring_model
                         ? "examples/Cassie/urdf/cassie_v2.urdf"
                         : "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  drake::multibody::MultibodyPlant<double> plant_feedback(0.0);
  addCassieMultibody(&plant_feedback, nullptr, true /*floating base*/, urdf,
                     FLAGS_spring_model, false /*loop closure*/);
  plant_feedback.Finalize();
  // Build fix-spring Cassie MBP
  //  drake::multibody::MultibodyPlant<double> plant(0.0);
  //  addCassieMultibody(&plant, nullptr, true,
  //                     "examples/Cassie/urdf/cassie_fixed_springs.urdf",
  //                     false, false);
  dairlib::multibody::PinocchioPlant<double> plant_pino(
      0.0, "examples/Cassie/urdf/cassie_fixed_springs.urdf");
  addCassieMultibody(&plant_pino, nullptr, false,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     false, false /*add_reflected_inertia*/);
  plant_pino.Finalize();

  drake::multibody::MultibodyPlant<double> plant(0.0);
  addCassieMultibody(&plant, nullptr, false,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     false, false);
  plant.Finalize();
  const auto& world_drake = plant.world_frame();

  //
  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nx = nq + nv;

  // benchmark
  auto context_pino = plant_pino.CreateDefaultContext();
  plant_pino.SetPositionsAndVelocities(context_pino.get(), VectorXd::Zero(nx));
  auto context_drake = plant.CreateDefaultContext();
  plant.SetPositionsAndVelocities(context_drake.get(), VectorXd::Zero(nx));

  int n_loop = 1000000;

  Eigen::Vector3d r_com;
  auto break2 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < n_loop; i++) {
    plant_pino.SetPositions(context_pino.get(), VectorXd::Zero(nq));
    plant_pino.CalcCenterOfMassPositionInWorld(*context_pino, &r_com);
  }
  auto break3 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = break3 - break2;
  cout << "r_com time (Pinocchio):" << elapsed.count() << "\n";

  break2 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < n_loop; i++) {
    plant.SetPositions(context_drake.get(), VectorXd::Zero(nq));
    plant.CalcCenterOfMassPositionInWorld(*context_drake);
  }
  break3 = std::chrono::high_resolution_clock::now();
  elapsed = break3 - break2;
  cout << "r_com time (Drake):" << elapsed.count() << "\n";

  Eigen::Vector3d v_com;
  break2 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < n_loop; i++) {
    plant_pino.SetPositionsAndVelocities(context_pino.get(),
                                         VectorXd::Zero(nx));
    plant_pino.CalcCenterOfMassTranslationalVelocityInWorld(*context_pino,
                                                            &v_com);
  }
  break3 = std::chrono::high_resolution_clock::now();
  elapsed = break3 - break2;
  cout << "v_com time (Pinocchio):" << elapsed.count() << "\n";

  break2 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < n_loop; i++) {
    plant.SetPositionsAndVelocities(context_drake.get(), VectorXd::Zero(nx));
    plant.CalcCenterOfMassTranslationalVelocityInWorld(*context_drake);
  }
  break3 = std::chrono::high_resolution_clock::now();
  elapsed = break3 - break2;
  cout << "v_com time (Drake):" << elapsed.count() << "\n";

  Eigen::MatrixXd J_com(3, nv);
  break2 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < n_loop; i++) {
    plant_pino.SetPositions(context_pino.get(), VectorXd::Zero(nq));
    plant_pino.CalcJacobianCenterOfMassTranslationalVelocity(*context_pino,
                                                             &J_com);
  }
  break3 = std::chrono::high_resolution_clock::now();
  elapsed = break3 - break2;
  cout << "J_com time (Pinocchio):" << elapsed.count() << "\n";

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

  return 0;  // Currently PinocchioPlant doesn't support floating base joint.
             // Need to implemtnat this. Also, urdf file doesn't have reflected
             // inertia
}

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::DoMain(argc, argv);
}
