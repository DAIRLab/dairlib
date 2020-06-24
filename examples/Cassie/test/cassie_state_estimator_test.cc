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
    addCassieMultibody(&plant_, nullptr, true /*floating base*/,
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
  q_init << 1, VectorXd::Zero(6), -0.084017, 0.084017, -0.00120735, 0.00120735,
      0.366012, 0.366012, -0.6305, -0.6305, 0.00205363, 0.00205363, 0.838878,
      0.838878, 0, 0.205351, 0, 0.205351;

  // Get the angles analytically
  double calc_left_heel_spring, calc_right_heel_spring;
  estimator_->solveFourbarLinkage(q_init, &calc_left_heel_spring,
                                  &calc_right_heel_spring);

  // Get the fix joint indices
  std::map<std::string, int> positionIndexMap =
      multibody::makeNameToPositionsMap(plant_);
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
  program.AddKinematicConstraint(*fourbar_evaluator_, q);
  for (auto& ind : fixed_joint_inds) {
    program.AddConstraint(q(ind) == q_init(ind));
  }

  program.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Major feasibility tolerance", 1e-8);
  program.SetInitialGuess(q, q_init);
  const auto result = Solve(program);
  VectorXd q_sol = result.GetSolution(q);

  double nlp_left_heel_spring =
      q_sol(positionIndexMap.at("ankle_spring_joint_left"));
  double nlp_right_heel_spring =
      q_sol(positionIndexMap.at("ankle_spring_joint_right"));

  EXPECT_TRUE((calc_left_heel_spring - nlp_left_heel_spring) < 1e-10);
  EXPECT_TRUE((calc_right_heel_spring - nlp_right_heel_spring) < 1e-10);
  EXPECT_TRUE((calc_left_heel_spring - nlp_left_heel_spring) > -1e-10);
  EXPECT_TRUE((calc_right_heel_spring - nlp_right_heel_spring) > -1e-10);
}

// Double support contact estimation test
// Checks if the contactEstimation returns the correct contacts for a
// configuration of the robot in double stance.
// This case is chosen in such a way that the stance can be estimated using just
// springs.
TEST_F(ContactEstimationTest, DoubleSupportContactEstimationTest) {
  VectorXd q(plant_.num_positions());
  VectorXd v(plant_.num_velocities());
  VectorXd u(plant_.num_actuators());
  VectorXd alpha_imu(3);

  q << 0.990065, 0.000339553, 0.00444831, 0.00085048, 0.00836164, -0.000249535,
      1.03223, -0.000810813, 6.8811e-05, 0.00177426, -0.00514383, 0.447568,
      0.44727, -1.01775, -1.01819, -0.044873, -0.0450231, 1.29924, 1.30006,
      0.00780166, -1.56023, 0.00757446, -1.56018;
  v << -0.00112262, -0.130482, -0.00168999, -0.0162583, 0.000174138, 0.036476,
      -0.00928683, 0.0104115, 0.0705929, -0.0664504, -2.19116, -2.20063,
      7.09707, 7.12947, -5.46722, -5.49114, 1.1875, 1.1884, 0, -5.8359, 0,
      -5.83556;
  u << 0.969754, -1.05396, 0.188614, -0.196147, -7.71725, -7.51014, 54.0345,
      54.2956, -17.5288, -17.5305;
  alpha_imu << 4.44637, -0.015849, -6.28362;

  output_->SetPositions(q);
  output_->SetVelocities(v);
  output_->SetEfforts(u);
  output_->SetIMUAccelerations(alpha_imu);

  dt_ = 0.001;
  left_contact_ = 0;
  right_contact_ = 0;

  auto context = estimator_->CreateDefaultContext();

  std::vector<double> optimal_cost(3, 0.0);
  estimator_->UpdateContactEstimationCosts(
      *output_, dt_, &context->get_mutable_discrete_state(), &optimal_cost);
  estimator_->EstimateContactForController(*output_, optimal_cost,
                                           &left_contact_, &right_contact_);

  EXPECT_TRUE(left_contact_) << "Left contact error during double support.";
  EXPECT_TRUE(right_contact_) << "Right contact error during double support.";
}
// Left support contact estimation test
// Checks if the contactEstimation returns the correct contacts for a
// configuration of the robot in left stance.
// This case is chosen in such a way that the stance can be estimated using just
// springs.
TEST_F(ContactEstimationTest, LeftSupportContactEstimationTest) {
  VectorXd q(plant_.num_positions());
  VectorXd v(plant_.num_velocities());
  VectorXd u(plant_.num_actuators());
  VectorXd alpha_imu(3);

  q << 0.989849, -0.000815987, -0.017933, -0.0111588, 0.344537, -0.148108,
      1.00902, -0.0357916, -0.0422061, -0.0068692, -0.0355008, 0.274222,
      0.644396, -1.00482, -1.50496, -0.0745786, -0.000565784, 1.36746, 1.73074,
      -0.043625, -1.45868, -0.000678207, -0.936994;
  v << -0.110601, -0.0521661, -0.00286609, 0.910837, -0.0174017, -0.00158473,
      0.124156, 0.8427, 0.0224065, 0.0678774, -1.22403, 2.89698, 0.32455,
      2.21075, -0.03638, 0.247213, -0.333968, -2.51737, 0, 1.36041, 0, -4.312;
  u << 43.4069, -3.11852, -0.18289, -0.315572, -7.86163, -0.12787, 78.4677,
      2.16671, -3.37293, -4.57146;
  alpha_imu << 0.807881, -0.889675, -9.63094;

  output_->SetPositions(q);
  output_->SetVelocities(v);
  output_->SetEfforts(u);
  output_->SetIMUAccelerations(alpha_imu);

  dt_ = 0.001;
  left_contact_ = 0;
  right_contact_ = 0;

  auto context = estimator_->CreateDefaultContext();

  std::vector<double> optimal_cost(3, 0.0);
  estimator_->UpdateContactEstimationCosts(
      *output_, dt_, &context->get_mutable_discrete_state(), &optimal_cost);
  estimator_->EstimateContactForController(*output_, optimal_cost,
                                           &left_contact_, &right_contact_);

  EXPECT_TRUE(left_contact_) << "Left contact error during left support.";
  EXPECT_TRUE(!right_contact_) << "Right contact error during left support.";
}

// Right support contact estimation test
// Checks if the contactEstimation returns the correct contacts for a
// configuration of the robot in right stance.
// This case is chosen in such a way that the stance can be estimated using just
// springs.
TEST_F(ContactEstimationTest, RightSupportContactEstimationTest) {
  VectorXd q(plant_.num_positions());
  VectorXd v(plant_.num_velocities());
  VectorXd u(plant_.num_actuators());
  VectorXd alpha_imu(3);

  q << 0.98987, -0.00811052, -0.00949625, 0.015811, 0.01949, -0.157343, 1.00405,
      0.123047, -0.0753356, 0.0288855, -0.0330248, 0.832632, 0.0262067,
      -1.52869, -0.882942, -0.00078387, -0.0740736, 1.74919, 1.23608,
      0.00556074, -1.49203, -0.0305797, -1.2012;
  v << 0.156632, -0.0502397, 0.101071, 0.232441, -0.296125, -0.0559459,
      -0.663525, 0.116557, -0.0264677, -0.107556, 2.18153, -0.0230963, -1.65117,
      -1.02961, -0.0386449, 0.735924, 1.75789, -0.0410481, 0, -1.46269, 0,
      0.482573;
  u << 1.17905, -39.036, -0.698845, 10.992, 4.76689, -7.84931, -0.399509, 71.98,
      -1.56492, -9.37228;
  alpha_imu << 2.06368, 2.54482, -9.64308;

  output_->SetPositions(q);
  output_->SetVelocities(v);
  output_->SetEfforts(u);
  output_->SetIMUAccelerations(alpha_imu);

  dt_ = 0.001;
  left_contact_ = 0;
  right_contact_ = 0;

  auto context = estimator_->CreateDefaultContext();

  std::vector<double> optimal_cost(3, 0.0);
  estimator_->UpdateContactEstimationCosts(
      *output_, dt_, &context->get_mutable_discrete_state(), &optimal_cost);
  estimator_->EstimateContactForController(*output_, optimal_cost,
                                           &left_contact_, &right_contact_);

  EXPECT_TRUE(!left_contact_) << "Left contact error during right support.";
  EXPECT_TRUE(right_contact_) << "Right contact error during right support.";
}
}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
