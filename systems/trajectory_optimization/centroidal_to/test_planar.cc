//
// Created by brian on 2/3/21.
//

#include <vector>
#include "planar_centroidal_traj_opt.h"
#include "drake/solvers/mathematical_program_result.h"

namespace dairlib {
namespace centroidal_to {
namespace planar{

int doMain(){
  double mass = 30;
  double I = 2;
  double mu = 0.5;
  double h = 0.025;

  std::vector<stance> mode_sequence = {
      stance::D, stance::R, stance::D, stance::L, stance::D
  };
  std::vector<double> times = {0.05, 0.35, 0.05, 0.35, 0.05};
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

  auto result = prog.SolveProg();
  return 0;
}

}
}
}


int main(int argc, char** argv) {
  return dairlib::centroidal_to::planar::doMain();
}