#include "examples/Cassie/cassie_state_estimator.h"

#include <gtest/gtest.h>

#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_solvers.h"

#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace systems {
namespace {

using dairlib::multibody::MultibodyProgram;
using drake::solvers::SolutionResult;
using drake::solvers::Solve;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::DistanceEvaluator;
using multibody::KinematicEvaluatorSet;
using multibody::WorldPointEvaluator;

class CassieStateEstimatorTest : public ::testing::Test {};

class ContactEstimationTest : public ::testing::Test {
 protected:
  ContactEstimationTest()
      : plant_(drake::multibody::MultibodyPlant<double>(1e-3)) {
    AddCassieMultibody(&plant_, nullptr, true /*floating base*/,
                       "examples/Cassie/urdf/cassie_v2.urdf",
                       true /*spring model*/, false /*loop closure*/);
    plant_.Finalize();

    output_ = std::make_unique<OutputVector<double>>(plant_.num_positions(),
                                                     plant_.num_velocities(),
                                                     plant_.num_actuators());

    // Evaluators for fourbar linkages
    fourbar_evaluator_ =
        std::make_unique<multibody::KinematicEvaluatorSet<double>>(plant_);
    left_loop_ = std::make_unique<multibody::DistanceEvaluator<double>>(
        LeftLoopClosureEvaluator(plant_));
    right_loop_ = std::make_unique<multibody::DistanceEvaluator<double>>(
        RightLoopClosureEvaluator(plant_));
    fourbar_evaluator_->add_evaluator(left_loop_.get());
    fourbar_evaluator_->add_evaluator(right_loop_.get());

    // Evaluators for foot contacts
    std::vector<int> inds = {0, 1, 2};
    left_contact_evaluator_ =
        std::make_unique<multibody::KinematicEvaluatorSet<double>>(plant_);
    auto left_toe = LeftToeFront(plant_);
    auto left_heel = LeftToeRear(plant_);
    left_toe_evaluator_ =
        std::make_unique<multibody::WorldPointEvaluator<double>>(
            plant_, left_toe.first, left_toe.second, Matrix3d::Identity(),
            Vector3d::Zero(), inds);
    left_heel_evaluator_ =
        std::make_unique<multibody::WorldPointEvaluator<double>>(
            plant_, left_heel.first, left_heel.second, Matrix3d::Identity(),
            Vector3d::Zero(), inds);
    left_contact_evaluator_->add_evaluator(left_toe_evaluator_.get());
    left_contact_evaluator_->add_evaluator(left_heel_evaluator_.get());
    right_contact_evaluator_ =
        std::make_unique<multibody::KinematicEvaluatorSet<double>>(plant_);
    auto right_toe = RightToeFront(plant_);
    auto right_heel = RightToeRear(plant_);
    right_toe_evaluator_ =
        std::make_unique<multibody::WorldPointEvaluator<double>>(
            plant_, right_toe.first, right_toe.second, Matrix3d::Identity(),
            Vector3d::Zero(), inds);
    right_heel_evaluator_ =
        std::make_unique<multibody::WorldPointEvaluator<double>>(
            plant_, right_heel.first, right_heel.second, Matrix3d::Identity(),
            Vector3d::Zero(), inds);
    right_contact_evaluator_->add_evaluator(right_toe_evaluator_.get());
    right_contact_evaluator_->add_evaluator(right_heel_evaluator_.get());

    // state estimator
    estimator_ = std::make_unique<CassieStateEstimator>(
        plant_, fourbar_evaluator_.get(), left_contact_evaluator_.get(),
        right_contact_evaluator_.get());
  }
  drake::multibody::MultibodyPlant<double> plant_;
  std::unique_ptr<OutputVector<double>> output_;
  std::unique_ptr<multibody::KinematicEvaluatorSet<double>> fourbar_evaluator_;
  std::unique_ptr<multibody::KinematicEvaluatorSet<double>>
      left_contact_evaluator_;
  std::unique_ptr<multibody::KinematicEvaluatorSet<double>>
      right_contact_evaluator_;
  std::unique_ptr<CassieStateEstimator> estimator_;
  double dt_;
  int left_contact_;
  int right_contact_;

  std::unique_ptr<multibody::DistanceEvaluator<double>> left_loop_;
  std::unique_ptr<multibody::DistanceEvaluator<double>> right_loop_;
  std::unique_ptr<multibody::WorldPointEvaluator<double>> left_toe_evaluator_;
  std::unique_ptr<multibody::WorldPointEvaluator<double>> left_heel_evaluator_;
  std::unique_ptr<multibody::WorldPointEvaluator<double>> right_toe_evaluator_;
  std::unique_ptr<multibody::WorldPointEvaluator<double>> right_heel_evaluator_;
};

TEST_F(ContactEstimationTest, solveFourbarLinkageTest) {
  // Example position (floating base position doesn't affect the result)
  VectorXd q_init(plant_.num_positions());
  q_init << 1, VectorXd::Zero(6), -0.084017,  -0.00120735, 0.366012, -0.6305,
      0.00205363, 0.838878, 0, 0.205351,
      0.084017,  0.00120735, 0.366012, -0.6305, 0.00205363,  0.838878, 0, 0.205351;

  // Get the angles analytically
  double calc_left_heel_spring, calc_right_heel_spring;
  estimator_->solveFourbarLinkage(q_init, &calc_left_heel_spring,
                                  &calc_right_heel_spring);

  // Get the fix joint indices
  std::map<std::string, int> positionIndexMap =
      multibody::MakeNameToPositionsMap(plant_);
  std::vector<int> fixed_joint_inds;
  fixed_joint_inds.push_back(positionIndexMap.at("knee_left"));
  fixed_joint_inds.push_back(positionIndexMap.at("knee_joint_left"));
  fixed_joint_inds.push_back(positionIndexMap.at("ankle_joint_left"));
  fixed_joint_inds.push_back(positionIndexMap.at("knee_right"));
  fixed_joint_inds.push_back(positionIndexMap.at("knee_joint_right"));
  fixed_joint_inds.push_back(positionIndexMap.at("ankle_joint_right"));

  // Construct MultibodyProgram
  multibody::MultibodyProgram<double> program(plant_);
  auto q = program.AddPositionVariables();
  auto u = program.AddInputVariables();
  auto lambda = program.AddConstraintForceVariables(*fourbar_evaluator_);
  program.AddKinematicPositionConstraint(*fourbar_evaluator_, q);
  for (auto& ind : fixed_joint_inds) {
    program.AddConstraint(q(ind) == q_init(ind));
  }

  program.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Major feasibility tolerance", 1e-6);
  program.SetInitialGuess(q, q_init);
  const auto result = Solve(program);
  VectorXd q_sol = result.GetSolution(q);

  double nlp_left_heel_spring =
      q_sol(positionIndexMap.at("ankle_spring_joint_left"));
  double nlp_right_heel_spring =
      q_sol(positionIndexMap.at("ankle_spring_joint_right"));

  EXPECT_NEAR(calc_left_heel_spring, nlp_left_heel_spring, 1e-5);
  EXPECT_NEAR(calc_right_heel_spring, nlp_right_heel_spring, 1e-5);
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
