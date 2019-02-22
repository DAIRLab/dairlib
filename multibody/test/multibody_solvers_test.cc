#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_solvers.h"
#include "multibody/rbt_utils.h"

namespace dairlib {
namespace systems {
namespace {

using std::unique_ptr;
using std::shared_ptr;
using std::make_unique;
using std::make_shared;
using std::map;
using std::string;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::AutoDiffXd;
using drake::VectorX;
using drake::MatrixX;
using drake::math::initializeAutoDiff;
using drake::math::DiscardGradient;
using drake::multibody::joints::FloatingBaseType;
using drake::multibody::joints::kRollPitchYaw;
using drake::multibody::AddFlatTerrainToWorld;
using drake::solvers::MathematicalProgram;
using dairlib::buildCassieTree;
using dairlib::multibody::ContactInfo;
using dairlib::multibody::ContactToolkit;
using dairlib::multibody::GetBodyIndexFromName;
using dairlib::multibody::PositionConstraint;
using dairlib::multibody::FixedPointConstraint;
using dairlib::multibody::GroundContactConstraint;
using dairlib::multibody::PositionSolver;
using dairlib::multibody::FixedPointSolver;
using dairlib::multibody::GroundContactSolver;

class MultibodySolversTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setting the random seed
    std::srand((unsigned int)time(0));

    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf";
    FloatingBaseType base_type = kRollPitchYaw;
    buildCassieTree(tree_, filename, base_type);

    // Adding the ground
    AddFlatTerrainToWorld(&tree_, 4, 0.05);

    num_positions_ = tree_.get_num_positions();
    num_velocities_ = tree_.get_num_velocities();
    num_states_ = num_positions_ + num_velocities_;
    num_efforts_ = tree_.get_num_actuators();

    // Setting the initial Cassie joint angles

    map<string, int> position_map = tree_.computePositionNameToIndexMap();
    x0_ = VectorXd::Zero(num_states_);
    x0_(position_map.at("hip_roll_left")) = 0.1;
    x0_(position_map.at("hip_roll_right")) = -0.1;
    x0_(position_map.at("hip_yaw_left")) = 0.01;
    x0_(position_map.at("hip_yaw_right")) = -0.01;
    x0_(position_map.at("hip_pitch_left")) = .269;
    x0_(position_map.at("hip_pitch_right")) = .269;
    x0_(position_map.at("knee_left")) = -.744;
    x0_(position_map.at("knee_right")) = -.744;
    x0_(position_map.at("ankle_joint_left")) = .81;
    x0_(position_map.at("ankle_joint_right")) = .81;
    x0_(position_map.at("toe_left")) = 0;
    x0_(position_map.at("toe_right")) = 0;
    // x0_(position_map.at("toe_left")) = -30.0 * M_PI / 180.0;
    // x0_(position_map.at("toe_right")) = -60.0 * M_PI / 180.0;

    // Collison detect
    VectorXd phi_total;
    Matrix3Xd normal_total, xA_total, xB_total;
    vector<int> idxA_total, idxB_total;
    KinematicsCache<double> k_cache =
        tree_.doKinematics(x0_.head(num_positions_), x0_.tail(num_velocities_));

    tree_.collisionDetect(k_cache, phi_total, normal_total, xA_total, xB_total,
                          idxA_total, idxB_total);

    const int world_ind = GetBodyIndexFromName(tree_, "world");
    const int toe_left_ind = GetBodyIndexFromName(tree_, "toe_left");
    const int toe_right_ind = GetBodyIndexFromName(tree_, "toe_right");

    // Extracting information into the four contacts.
    VectorXd phi(4);
    Matrix3Xd normal(3, 4), xA(3, 4), xB(3, 4);
    vector<int> idxA(4), idxB(4);

    int k = 0;
    for (unsigned i = 0; i < idxA_total.size(); ++i) {
      int ind_a = idxA_total.at(i);
      int ind_b = idxB_total.at(i);
      if ((ind_a == world_ind && ind_b == toe_left_ind) ||
          (ind_a == world_ind && ind_b == toe_right_ind) ||
          (ind_a == toe_left_ind && ind_b == world_ind) ||
          (ind_a == toe_right_ind && ind_b == world_ind)) {
        xA.col(k) = xA_total.col(i);
        xB.col(k) = xB_total.col(i);
        idxA.at(k) = idxA_total.at(i);
        idxB.at(k) = idxB_total.at(i);
        ++k;
      }
    }

    // Creating the contact info
    // idxB is the vector index for the body which is accessed through
    // ContactInfo.idxA
    // In this case xA corresponds to the points on the ground and hence xA
    // and
    // xB must be interchanged when constructing contact_info_
    contact_info_ = {xB, xA, idxB};

    num_contacts_ = contact_info_.idxA.size();
    num_position_constraints_ = tree_.getNumPositionConstraints();
    num_forces_ = num_contacts_ * 3 + num_position_constraints_;
  }

  RigidBodyTree<double> tree_;
  int num_positions_;
  int num_velocities_;
  int num_states_;
  int num_efforts_;
  int num_contacts_;
  int num_position_constraints_;
  int num_forces_;
  ContactInfo contact_info_;
  VectorXd x0_;
};

TEST_F(MultibodySolversTest, InitializationTest) {
  // Constraint class initializations
  // Position constraint

  PositionConstraint position_constraint(tree_);

  // Fixed point constraint with and without contact
  FixedPointConstraint fp_constraint1(tree_);
  FixedPointConstraint fp_constraint2(tree_, contact_info_);

  // Contact constraint
  GroundContactConstraint contact_constraint(tree_, contact_info_);

  // Solver class initializations

  // PositionSolver
  // Testing basic getters and setters
  PositionSolver position_solver(tree_);
  position_solver.set_filename("position_log");
  position_solver.set_major_tolerance(0.001);
  position_solver.set_minor_tolerance(0.01);

  ASSERT_EQ(position_solver.get_filename(), "position_log");
  ASSERT_EQ(position_solver.get_major_tolerance(), 0.001);
  ASSERT_EQ(position_solver.get_minor_tolerance(), 0.01);

  // Testing mathematical program getter
  shared_ptr<MathematicalProgram> prog_position = position_solver.get_program();

  // ContactSolver
  // Testing basic getters and setters
  GroundContactSolver contact_solver(tree_, contact_info_);
  contact_solver.set_filename("contact_log");
  contact_solver.set_major_tolerance(0.002);
  contact_solver.set_minor_tolerance(0.02);

  ASSERT_EQ(contact_solver.get_filename(), "contact_log");
  ASSERT_EQ(contact_solver.get_major_tolerance(), 0.002);
  ASSERT_EQ(contact_solver.get_minor_tolerance(), 0.02);

  // Testing mathematical program getter
  shared_ptr<MathematicalProgram> prog_contact = contact_solver.get_program();

  // FixedPointSolver
  // Testing basic getters and setters
  FixedPointSolver fp_solver(tree_);
  fp_solver.set_filename("fp_log");
  fp_solver.set_major_tolerance(0.003);
  fp_solver.set_minor_tolerance(0.03);

  ASSERT_EQ(fp_solver.get_filename(), "fp_log");
  ASSERT_EQ(fp_solver.get_major_tolerance(), 0.003);
  ASSERT_EQ(fp_solver.get_minor_tolerance(), 0.03);

  // Testing mathematical program getter
  shared_ptr<MathematicalProgram> prog_fp = fp_solver.get_program();
}

TEST_F(MultibodySolversTest, SolveTest) {
  // Testing the solvers

  VectorXd q = x0_.head(num_positions_);
  VectorXd u = VectorXd::Zero(num_efforts_);
  VectorXd lambda = VectorXd::Zero(num_forces_);
  // PositionSolver
  PositionSolver position_solver(tree_);
  position_solver.SetInitialGuessQ(q);

  std::cout << "Position solver result: " << position_solver.Solve(q)
            << std::endl;

  VectorXd q_sol = position_solver.GetSolutionQ();

  // Solution dimension check
  ASSERT_EQ(q_sol.size(), num_positions_);
  // Checking if the solution constraints have been satisfied
  ASSERT_TRUE(position_solver.CheckConstraint(q_sol));

  // GroundContactSolver
  GroundContactSolver contact_solver(tree_, contact_info_);
  contact_solver.SetInitialGuessQ(q);

  std::cout << "Contact solver result: " << contact_solver.Solve(q)
            << std::endl;

  q_sol = contact_solver.GetSolutionQ();

  // Solution dimension check
  ASSERT_EQ(q_sol.size(), num_positions_);
  // Checking if the solution constraints have been satisfied
  ASSERT_TRUE(contact_solver.CheckConstraint(q_sol));

  // FixedPointSolver
  FixedPointSolver fp_solver(tree_, contact_info_);
  fp_solver.SetInitialGuessQ(q);
  fp_solver.SetInitialGuessU(u);
  fp_solver.AddSpreadNormalForcesCost();
  //fp_solver.AddFrictionConeConstraint(0.8);
  fp_solver.SetInitialGuessLambda(lambda);

  std::cout << "Fixed point solver result: " << fp_solver.Solve(q, u)
            << std::endl;

  q_sol = fp_solver.GetSolutionQ();
  VectorXd u_sol = fp_solver.GetSolutionU();
  VectorXd lambda_sol = fp_solver.GetSolutionLambda();

  VectorXd x_sol = VectorXd::Zero(num_positions_ + num_velocities_);
  x_sol.head(num_positions_) = q_sol;
  ContactToolkit<double> ct(tree_, contact_info_);
  std::cout << ct.CalcMVDot(x_sol, u_sol, lambda_sol).transpose() << std::endl;
  std::cout << "q ----------------------" << std::endl;
  std::cout << ct.CalcTimeDerivatives(x_sol, u_sol, lambda_sol).transpose()
            << std::endl;
  std::cout << "u ----------------------" << std::endl;
  std::cout << u_sol << std::endl;
  std::cout << "lambda ----------------------" << std::endl;
  std::cout << lambda_sol << std::endl;

  // Solution dimension check
  ASSERT_EQ(q_sol.size(), num_positions_);
  ASSERT_EQ(u_sol.size(), num_efforts_);
  ASSERT_EQ(lambda_sol.size(), num_forces_);
  // Solution constraints check
  ASSERT_TRUE(fp_solver.CheckConstraint(q_sol, u_sol, lambda_sol));
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
