//
// Created by brian on 2/3/21.
//

#include <gflags/gflags.h>
#include <vector>
#include "planar_centroidal_traj_opt.h"
#include "drake/solvers/mathematical_program_result.h"
#include "systems/trajectory_optimization/centroidal_to/planar_rigid_body_dynamics_constraint.h"

namespace dairlib {
namespace centroidal_to {
namespace planar{

using drake::AutoDiffXd;
using drake::Vector2;
using drake::Vector1;
using drake::VectorX;


DEFINE_int32(iteration_limit, 2500, "Max SNOPT iterations");

int doMain(int argc, char** argv){
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  double mass = 20;
  double I = 2;
  double mu = 1.0;
  double h = 0.025;

  std::vector<stance> mode_sequence = {
      stance::D, stance::R, stance::D, stance::L, stance::D
  };
  std::vector<double> times = {0.05, 0.3, 0.05, 0.3, 0.05};
  Eigen::Vector2d com0;
  com0 << 0, 1;
  Eigen::Vector2d com1;
  com1 << 1, 1;
  Eigen::Vector2d dev;
  dev << 0.25, 0;

  PlanarCentroidalTrajOpt prog(I, mass, h, mu);
  prog.SetModeSequence(mode_sequence, times);
  prog.SetNominalStance(-com0, -com0);
  prog.SetMaxDeviationConstraint(dev);
  prog.SetInitialPose(com0, 0);
  prog.SetFinalPose(com1, 0.2);
  prog.SetInitialVel(Eigen::Vector2d::Zero(), 0);
  prog.SetFinalVel(Eigen::Vector2d::Zero(), 0);
  prog.SetInitialStateGuess();
  prog.SetInitialForceGuess();

  auto result = prog.SolveProg(FLAGS_iteration_limit);

  if(result.is_success()) {
    std::cout << "Success!" << std::endl;
    std::cout << result.GetSolution() << std::endl;
  } else {

  }
  return 0;
}


void testRBDConstraint() {
  double mass = 20;
  double I = 2;
  double h = 0.025;

  auto constraint = PlanarRigidBodyDynamicsConstraint(I, mass, h, 1);
  Vector2<AutoDiffXd> r;
  Vector1<AutoDiffXd> theta;
  Vector2<AutoDiffXd> v;
  Vector1<AutoDiffXd> omega;
  Vector2<AutoDiffXd> p;
  Vector2<AutoDiffXd> f;
}

}
}
}


int main(int argc, char** argv) {
  return dairlib::centroidal_to::planar::doMain(argc, argv);
}