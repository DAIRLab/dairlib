//
// Created by brian on 2/3/21.
//

#include <gflags/gflags.h>
#include <vector>
#include <iomanip>
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
  std::cout << std::setprecision(4);
  std::cout << std::fixed;
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  double mass = 10;
  double I = 2;
  double mu = 1.0;
  double h = 0.025;

  std::vector<stance> mode_sequence = { stance::D, stance::R, stance::D , stance::L, stance::D};
  std::vector<double> times = {0.05, 0.3, 0.05, 0.3, 0.05};
  Eigen::Vector2d com0;
  com0 << 0, 1;
  Eigen::Vector2d com1;
  com1 << 0.15, 1;
  Eigen::Vector2d dev;
  dev << 0.1, 0.1;

  PlanarCentroidalTrajOpt prog(I, mass, h, mu);
  prog.SetModeSequence(mode_sequence, times);
  prog.SetNominalStance(-com0, -com0);
  prog.SetMaxDeviationConstraint(dev);
  //prog.SetFootPlacementContinuityConstraint();
  prog.SetInitialPose(com0, 0);
  prog.SetFinalPose(com1, 0, 0.03);
  prog.SetInitialVel(Eigen::Vector2d::Zero(), 0);
  prog.SetFinalVel(Eigen::Vector2d::Zero(), 0);
  prog.SetInitialStateGuess();
  prog.SetInitialForceGuess();
  prog.AddBoundingBoxConstraint(Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(), prog.modes()[0].stance_vars_.head(kLinearDim));
  prog.AddBoundingBoxConstraint(Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(), prog.modes()[0].stance_vars_.tail(kLinearDim));

  for (int i = 0; i < prog.modes().size(); i++) {
    for (int j = 0; j < prog.modes()[i].state_vars_.size(); j++) {
      prog.AddQuadraticCost(Eigen::MatrixXd::Identity(1, 1),
                            Eigen::VectorXd::Ones(1),
                            prog.modes()[i].state_vars_[j].segment(
                                kStateVars - kAngularDim, kAngularDim));
    }
  }



  auto modes = prog.modes();

  std::cout << "\n\nInitial Guesses:" << std::endl;
  std::cout << "Position (x, z): " << std::endl;
  for (int i = 0; i < modes.size(); i++) {
    for (int j = 0; j < modes[i].state_vars_.size(); j++) {
      auto res = prog.GetInitialGuess(modes[i].state_vars_[j].head(kLinearDim));
      std::cout << i << ", " << j << ":   (" << res.head(1) << ", " << res.tail(1) << ")"
                << std::endl;
    }
  }

  std::cout << "Stance pos (x, z): " << std::endl;
  for (int i = 0; i < modes.size(); i++) {

    auto res1 = prog.GetInitialGuess(modes[i].stance_vars_.head(kLinearDim));
    std::cout << i << ":   (" << res1.head(1) << ", " << res1.tail(1) << ")";

    if (mode_sequence[i] == stance::D) {
      auto res2 = prog.GetInitialGuess(modes[i].stance_vars_.tail(kLinearDim));
      std::cout << ",  (" << res2.head(1) << ", " << res2.tail(1) << ")";
    }
    std::cout << std::endl;
  }

  std::cout << "\nForce (x, y)" << std::endl;

  for (int i = 0; i < modes.size(); i++) {
    for (int j = 0; j < modes[i].force_vars_.size(); j++) {
      auto res = prog.GetInitialGuess(modes[i].force_vars_[j].head(kLinearDim));
      std::cout << i << ", " << j << ":   (" << res.head(1) << ", " << res.tail(1) << ")";
      if (mode_sequence[i] == stance::D) {
        auto res2 = prog.GetInitialGuess(modes[i].force_vars_[j].tail(kLinearDim));
        std::cout << ",  (" << res2.head(1) << ", " << res2.tail(1) << ")";
      }
      std::cout << std::endl;
    } std::cout << std::endl;
  }

  auto result = prog.SolveProg(FLAGS_iteration_limit);

  if(result.is_success()) {
    std::cout << "\nSuccess!\n" << std::endl;
  } else {
    std::cout << "\nProblem Failed with Code :\n" <<
    result.get_solution_result() << std::endl;
  }


  std::cout << "Position (x, z), vel(x, y): " << std::endl;

  for (int i = 0; i < modes.size(); i++) {
    for (int j = 0; j < modes[i].state_vars_.size(); j++) {
      auto res = result.GetSolution(modes[i].state_vars_[j].head(kLinearDim));
      auto vel = result.GetSolution(modes[i].state_vars_[j].segment(kLinearDim + kAngularDim, kLinearDim));
      std::cout << i << ", " << j << ":   (" << res.head(1) << ", " << res.tail(1) << ") | (" <<
      vel.head(1) << ", " << vel.tail(1) << ")" << std::endl;
    }
  }


  std::cout << "Stance pos (x, z): " << std::endl;
  for (int i = 0; i < modes.size(); i++) {
    auto res1 = result.GetSolution(modes[i].stance_vars_.head(kLinearDim));
    std::cout << i << ":   (" << res1.head(1) << ", " << res1.tail(1) << ")";
    if (mode_sequence[i] == stance::D) {
      auto res2 = result.GetSolution(modes[i].stance_vars_.tail(kLinearDim));
      std::cout << ",  (" << res2.head(1) << ", " << res2.tail(1) << ")";
    }
    std::cout << std::endl;
  }

  std::cout << "\nForce (x, z): " << std::endl;

  for (int i = 0; i < modes.size(); i++) {
    for (int j = 0; j < modes[i].force_vars_.size(); j++) {
      auto res = result.GetSolution(modes[i].force_vars_[j].head(kLinearDim));
      std::cout << i << ", " << j << ":   (" << res.head(1) << ", " << res.tail(1) << ")";
      if (mode_sequence[i] == stance::D) {
        auto res2 = result.GetSolution(modes[i].force_vars_[j].tail(kLinearDim));
        std::cout << ",  (" << res2.head(1) << ", " << res2.tail(1) << ")";
      }
      std::cout << std::endl;
    } std::cout << std::endl;
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