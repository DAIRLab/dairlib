#include <chrono>
#include <iostream>

#include "examples/Cassie/cassie_utils.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"

#include "drake/common/text_logging.h"
#include "drake/multibody/parsing/parser.h"

/// This is a timing test to determine how effective caching is using
/// MultibodyPlant kinematics caching via a context.
/// Tests phi, J, Jdotv in a few scenarios
/// Results show that kinematics effectively track dependencies, and setting
///  context.q = context.q
/// will not make the cache dirty.
///
/// As of 5/31/2020, phi, J make effective use of the cache, across bodies,
/// but Jdotv is only partially effective.
///
/// Sample results from 5/31/2020 are below.

/*
SINGLE EVALUATOR.
***** baseline tests (10000) *****
phi:  5.6616
J:  10.4989
Jdotv:  18.0919
set qv: 0.8371
***** 2X tests (10000) *****
phi:  4.4867
J:  9.3347
Jdotv:  28.4073
***** sequence tests (10000) *****
phi,J:  7.9302
all:  22.6165
***** sequence tests resetting state (10000) *****
phi,J:  15.0376
all:  23.113

TWO EVALUATORS.
***** baseline tests (10000) *****
phi:  4.7404
J:  9.7794
Jdotv:  25.9269
set qv: 0.9056
***** 2X tests (10000) *****
phi:  5.1799
J:  12.9043
Jdotv:  41.439
***** sequence tests (10000) *****
phi,J:  9.9857
all:  27.4305
***** sequence tests resetting state (10000) *****
phi,J:  14.2107
all:  33.7885
*/

namespace dairlib {
namespace multibody {

using Eigen::VectorXd;
using std::cout;
using std::endl;
typedef std::chrono::steady_clock my_clock;

void TestEvaluatorSet(const KinematicEvaluatorSet<double> evaluators) {
  int N = 10000;
  const auto& plant = evaluators.plant();
  auto context = plant.CreateDefaultContext();

  VectorXd q, v;

  cout << "***** baseline tests (" << std::to_string(N) << ") *****" << endl;
  auto start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    Eigen::MatrixXd M(plant.num_velocities(), plant.num_velocities());
    q = Eigen::VectorXd::Random(plant.num_positions());
    plant.SetPositions(context.get(), q);
    evaluators.plant().CalcMassMatrix(*context, &M);
  }
  auto stop =  my_clock::now();
  auto duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "M:\t" << (1.0*duration.count())/N << endl;

  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    v = Eigen::VectorXd::Random(plant.num_velocities());
    plant.SetPositions(context.get(), q);
    Eigen::MatrixXd M(plant.num_velocities(), plant.num_velocities());
    evaluators.plant().CalcMassMatrix(*context, &M);
    auto res = M.llt();
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "M.llt:\t" << (1.0*duration.count())/N << endl;

  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    v = Eigen::VectorXd::Random(plant.num_velocities());
    plant.SetPositions(context.get(), q);
    plant.SetVelocities(context.get(), v);
    VectorXd C(plant.num_velocities());
    plant.CalcBiasTerm(*context, &C);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "C:\t" << (1.0*duration.count())/N << endl;

  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    v = Eigen::VectorXd::Random(plant.num_velocities());
    plant.SetPositions(context.get(), q);
    plant.SetVelocities(context.get(), v);
    VectorXd C(plant.num_velocities());
    plant.CalcBiasTerm(*context, &C);
    Eigen::MatrixXd M(plant.num_velocities(), plant.num_velocities());
    evaluators.plant().CalcMassMatrix(*context, &M);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "C,M:\t" << (1.0*duration.count())/N << endl;

  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    plant.SetPositions(context.get(), q);
    auto tmp = evaluators.EvalFull(*context);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "phi:\t" << (1.0*duration.count())/N << endl;

  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    plant.SetPositions(context.get(), q);
    auto tmp = evaluators.EvalFullJacobian(*context);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "J:\t" << (1.0*duration.count())/N << endl;

  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    v = Eigen::VectorXd::Random(plant.num_velocities());
    plant.SetPositions(context.get(), q);
    plant.SetVelocities(context.get(), v);
    auto tmp = evaluators.EvalFullJacobianDotTimesV(*context);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "Jdotv:\t" << (1.0*duration.count())/N << endl;


  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    v = Eigen::VectorXd::Random(plant.num_velocities());
    plant.SetPositions(context.get(), q);
    plant.SetVelocities(context.get(), v);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "set qv:\t" << (1.0*duration.count())/N << endl;

 cout << "***** 2X tests (" << std::to_string(N) << ") *****" << endl;
  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    Eigen::MatrixXd M(plant.num_velocities(), plant.num_velocities());
    q = Eigen::VectorXd::Random(plant.num_positions());
    plant.SetPositions(context.get(), q);
    evaluators.plant().CalcMassMatrix(*context, &M);
    evaluators.plant().CalcMassMatrix(*context, &M);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "M:\t" << (1.0*duration.count())/N << endl;

  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    v = Eigen::VectorXd::Random(plant.num_velocities());
    plant.SetPositions(context.get(), q);
    plant.SetVelocities(context.get(), v);
    VectorXd C(plant.num_velocities());
    plant.CalcBiasTerm(*context, &C);
    plant.CalcBiasTerm(*context, &C);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "C:\t" << (1.0*duration.count())/N << endl;

  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    plant.SetPositions(context.get(), q);
    auto tmp = evaluators.EvalFull(*context);
    tmp = evaluators.EvalFull(*context);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "phi:\t" << (1.0*duration.count())/N << endl;

  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    plant.SetPositions(context.get(), q);
    auto tmp = evaluators.EvalFullJacobian(*context);
    tmp = evaluators.EvalFullJacobian(*context);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "J:\t" << (1.0*duration.count())/N << endl;

  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    v = Eigen::VectorXd::Random(plant.num_velocities());
    plant.SetPositions(context.get(), q);
    plant.SetVelocities(context.get(), v);
    auto tmp = evaluators.EvalFullJacobianDotTimesV(*context);
    tmp = evaluators.EvalFullJacobianDotTimesV(*context);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "Jdotv:\t" << (1.0*duration.count())/N << endl;

  cout << "***** sequence tests (" << std::to_string(N) << ") *****" << endl;
  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    plant.SetPositions(context.get(), q);
    auto tmp = evaluators.EvalFull(*context);
    auto tmpJ = evaluators.EvalFullJacobian(*context);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "phi,J:\t" << (1.0*duration.count())/N << endl;

  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    v = Eigen::VectorXd::Random(plant.num_velocities());
    plant.SetPositions(context.get(), q);
    plant.SetVelocities(context.get(), v);
    auto tmp = evaluators.EvalFull(*context);
    auto tmpJ = evaluators.EvalFullJacobian(*context);
    auto tmpJdotV = evaluators.EvalFullJacobianDotTimesV(*context);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "all:\t" << (1.0*duration.count())/N << endl;

  cout << "***** sequence tests resetting state (" << std::to_string(N) << ") *****" << endl;
  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    plant.SetPositions(context.get(), q);
    auto tmp = evaluators.EvalFull(*context);
    plant.SetPositions(context.get(), q);
    auto tmpJ = evaluators.EvalFullJacobian(*context);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "phi,J:\t" << (1.0*duration.count())/N << endl;

  start =  my_clock::now();
  for (int i = 0; i < N; i++) {
    q = Eigen::VectorXd::Random(plant.num_positions());
    v = Eigen::VectorXd::Random(plant.num_velocities());
    plant.SetPositions(context.get(), q);
    auto tmp = evaluators.EvalFull(*context);
    auto tmpJ = evaluators.EvalFullJacobian(*context);
    plant.SetVelocities(context.get(), v);
    auto tmpJdotV = evaluators.EvalFullJacobianDotTimesV(*context);
  }
  stop =  my_clock::now();
  duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  cout << "all:\t" << (1.0*duration.count())/N << endl;
}

int DoMain(int argc, char* argv[]) {
  srand((unsigned int) time(0));
  drake::logging::set_log_level("err");  // ignore warnings about joint limit
  
  std::string filename = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  // Build plant
  drake::multibody::MultibodyPlant<double> plant(0);
  drake::multibody::Parser parser(&plant);
  std::string full_name = FindResourceOrThrow(filename);
  parser.AddModelFromFile(full_name);
  plant.Finalize();

  multibody::KinematicEvaluatorSet<double> evaluators(plant);

  // Add loop closures
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  

  cout << "STARTING EVALUATION TESTS. ALL TIMES AVG TIME IN MICROSECONDS." << endl;
  cout << endl << "SINGLE EVALUATOR." << endl;
  evaluators.add_evaluator(&left_loop);
  TestEvaluatorSet(evaluators);

  cout << endl << "TWO EVALUATORS." << endl;
  evaluators.add_evaluator(&right_loop);
  TestEvaluatorSet(evaluators);
  return 0;
}



}  // namespace multibody
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::multibody::DoMain(argc, argv);
}
