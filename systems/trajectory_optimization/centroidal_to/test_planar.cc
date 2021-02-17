//
// Created by brian on 2/3/21.
//

#include <gflags/gflags.h>
#include <vector>
#include <iomanip>
#include "planar_centroidal_traj_opt.h"
#include "drake/solvers/mathematical_program_result.h"
#include "systems/trajectory_optimization/centroidal_to/planar_rigid_body_dynamics_constraint.h"
#include "lcm/lcm_trajectory.h"

namespace dairlib {
namespace centroidal_to {
namespace planar{

using drake::AutoDiffXd;
using drake::Vector2;
using drake::Vector1;
using drake::VectorX;
using dairlib::LcmTrajectory;


DEFINE_int32(iteration_limit, 2500, "Max SNOPT iterations");
DEFINE_double(height_cost, 0.1, "cost on height deviation");
DEFINE_double(angular_vel_cost, 0.1, "Cost on angular velocity");
DEFINE_double(final_pos_cost, 0.05, "Cost on final position");
DEFINE_double(final_pos_tol, 0.075, "Final Position Error Tolerance");
DEFINE_double(theta, 0.2, "final_angle");
DEFINE_double(h, 0.025, "timestep");
DEFINE_int32(step_pairs, 1, "number of pairs of steps");
DEFINE_double(t_ds, 0.05, "double stance time");
DEFINE_double(t_ss, 0.25, "single stance time");


void print_guess(PlanarCentroidalTrajOpt& prog);
void print_result(PlanarCentroidalTrajOpt& prog, drake::solvers::MathematicalProgramResult& result);

int doMain(int argc, char** argv){
  std::cout << std::setprecision(4);
  std::cout << std::fixed;

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  double mass = 5;
  double I = 2;
  double mu = 1.0;
  double h = FLAGS_h;

  std::vector<stance> mode_sequence = { stance::D };
  std::vector<double> times = {FLAGS_t_ds};

  for (int i = 0; i < FLAGS_step_pairs; i++) {
    mode_sequence.push_back(stance::L);
    mode_sequence.push_back(stance::D);
    mode_sequence.push_back(stance::R);
    mode_sequence.push_back(stance::D);
    times.push_back(FLAGS_t_ss);
    times.push_back(FLAGS_t_ds);
    times.push_back(FLAGS_t_ss);
    times.push_back(FLAGS_t_ds);
  }

  Eigen::Vector2d com0;
  com0 << 0, 1;
  Eigen::Vector2d com1;
  com1 << 0.1 * FLAGS_step_pairs, 1;
  Eigen::Vector2d dev;
  dev << 0.05, 0.05;

  PlanarCentroidalTrajOpt prog(I, mass, h, mu);
  prog.SetModeSequence(mode_sequence, times);
  prog.SetNominalStance(-com0, -com0);
  prog.SetMaxDeviationConstraint(dev);
  prog.SetInitialPose(com0, 0);
  prog.SetFinalPose(com1, FLAGS_theta, FLAGS_final_pos_tol);
  prog.SetInitialVel(Eigen::Vector2d::Zero(), 0);
  //prog.SetFinalVel(Eigen::Vector2d::Zero(), 0);
  prog.SetInitialStateGuess();
  prog.SetInitialForceGuess();
  prog.SetInitialStanceGuess();
  prog.AddBoundingBoxConstraint(Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(), prog.modes()[0].stance_vars_.head(kLinearDim));
  prog.AddBoundingBoxConstraint(Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(), prog.modes()[0].stance_vars_.tail(kLinearDim));

  prog.AddQuadraticErrorCost(
      FLAGS_final_pos_cost*Eigen::Matrix2d::Identity(),
      com1, prog.modes().back().state_vars_.back().head(kLinearDim));


  for (int i = 0; i < prog.modes().size(); i++) {
    for (int j = 0; j < prog.modes()[i].state_vars_.size(); j++) {

      prog.AddQuadraticErrorCost(FLAGS_height_cost * Eigen::MatrixXd::Identity(1, 1),
          com0.tail(1), prog.modes()[i].state_vars_[j].segment(1,1));

      prog.AddQuadraticCost(FLAGS_angular_vel_cost * Eigen::MatrixXd::Identity(1, 1),
                            Eigen::VectorXd::Zero(1),
                            prog.modes()[i].state_vars_[j].segment(
                                kStateVars - kAngularDim, kAngularDim));
    }
  }

  // print_guess(prog);
  auto result = prog.SolveProg(FLAGS_iteration_limit);
  // print_result(prog, result);
  std::cout << result.get_solution_result() << std::endl;

  if (result.is_success()) {
    LcmTrajectory state_traj = prog.GetStateTrajectories(result);
    LcmTrajectory stance_traj = prog.GetFootTrajectories(result);
    LcmTrajectory force_traj = prog.GetForceTrajectories(result);

    state_traj.WriteToFile("/home/brian/workspace/dairlib/systems/trajectory_optimization/centroidal_to/CoMtraj.lcmtraj");
    stance_traj.WriteToFile("/home/brian/workspace/dairlib/systems/trajectory_optimization/centroidal_to/stancetraj.lcmtraj");
    force_traj.WriteToFile("/home/brian/workspace/dairlib/systems/trajectory_optimization/centroidal_to/forcetraj.lcmtraj");
  }

  return 0;
}


void print_guess(PlanarCentroidalTrajOpt &prog) {
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

    if (prog.sequence()[i] == stance::D) {
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
      if (prog.sequence()[i] == stance::D) {
        auto res2 = prog.GetInitialGuess(modes[i].force_vars_[j].tail(kLinearDim));
        std::cout << ",  (" << res2.head(1) << ", " << res2.tail(1) << ")";
      }
      std::cout << std::endl;
    } std::cout << std::endl;
  }
}

void print_result(PlanarCentroidalTrajOpt& prog, drake::solvers::MathematicalProgramResult& result) {
  auto modes = prog.modes();

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
    if (prog.sequence()[i] == stance::D) {
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
      if (prog.sequence()[i] == stance::D) {
        auto res2 = result.GetSolution(modes[i].force_vars_[j].tail(kLinearDim));
        std::cout << ",  (" << res2.head(1) << ", " << res2.tail(1) << ")";
      }
      std::cout << std::endl;
    } std::cout << std::endl;
  }
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