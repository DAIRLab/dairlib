#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_solvers.h"
#include "multibody/rbt_utils.h"

namespace dairlib {
namespace systems {
namespace {

using std::cout;
using std::endl;
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
using drake::multibody::joints::kFixed;
using drake::multibody::AddFlatTerrainToWorld;
using drake::solvers::MathematicalProgram;
using dairlib::buildCassieTree;
using dairlib::multibody::ContactInfo;
using dairlib::multibody::ContactToolkit;
using dairlib::multibody::GetBodyIndexFromName;
using dairlib::multibody::PositionConstraint;
using dairlib::multibody::FixedPointConstraint;
using dairlib::multibody::ContactConstraint;
using dairlib::multibody::PositionSolver;
using dairlib::multibody::FixedPointSolver;
using dairlib::multibody::ContactSolver;

class MultibodySolversTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setting the random seed
    std::srand((unsigned int)time(0));

    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf";
    buildCassieTree(tree_floating_, filename, kRollPitchYaw);
    buildCassieTree(tree_fixed_, filename, kFixed);

    // Adding the ground
    AddFlatTerrainToWorld(&tree_floating_, 4, 0.05);

    num_positions_fixed_ = tree_fixed_.get_num_positions();
    num_velocities_fixed_ = tree_fixed_.get_num_velocities();
    num_states_fixed_ = num_positions_fixed_ + num_velocities_fixed_;
    num_efforts_fixed_ = tree_fixed_.get_num_actuators();

    num_positions_floating_ = tree_floating_.get_num_positions();
    num_velocities_floating_ = tree_floating_.get_num_velocities();
    num_states_floating_ = num_positions_floating_ + num_velocities_floating_;
    num_efforts_floating_ = tree_floating_.get_num_actuators();

    // Setting the initial Cassie joint angles
    map<string, int> position_map_fixed =
        tree_fixed_.computePositionNameToIndexMap();
    x0_fixed_ = VectorXd::Zero(num_states_floating_);
    x0_fixed_(position_map_fixed.at("hip_roll_left")) = 0.1;
    x0_fixed_(position_map_fixed.at("hip_roll_right")) = -0.1;
    x0_fixed_(position_map_fixed.at("hip_yaw_left")) = 0.01;
    x0_fixed_(position_map_fixed.at("hip_yaw_right")) = -0.01;
    x0_fixed_(position_map_fixed.at("hip_pitch_left")) = .269;
    x0_fixed_(position_map_fixed.at("hip_pitch_right")) = .269;
    x0_fixed_(position_map_fixed.at("knee_left")) = -.744;
    x0_fixed_(position_map_fixed.at("knee_right")) = -.744;
    x0_fixed_(position_map_fixed.at("ankle_joint_left")) = .81;
    x0_fixed_(position_map_fixed.at("ankle_joint_right")) = .81;
    x0_fixed_(position_map_fixed.at("toe_left")) = 0;
    x0_fixed_(position_map_fixed.at("toe_right")) = 0;
    // x0_fixed_(position_map_fixed.at("toe_left")) = -30.0 * M_PI / 180.0;
    // x0_fixed_(position_map_fixed.at("toe_right")) = -60.0 * M_PI / 180.0;

    map<string, int> position_map_floating =
        tree_floating_.computePositionNameToIndexMap();
    x0_floating_ = VectorXd::Zero(num_states_floating_);
    x0_floating_(position_map_floating.at("hip_roll_left")) = 0.1;
    x0_floating_(position_map_floating.at("hip_roll_right")) = -0.1;
    x0_floating_(position_map_floating.at("hip_yaw_left")) = 0.01;
    x0_floating_(position_map_floating.at("hip_yaw_right")) = -0.01;
    x0_floating_(position_map_floating.at("hip_pitch_left")) = .269;
    x0_floating_(position_map_floating.at("hip_pitch_right")) = .269;
    x0_floating_(position_map_floating.at("knee_left")) = -.744;
    x0_floating_(position_map_floating.at("knee_right")) = -.744;
    x0_floating_(position_map_floating.at("ankle_joint_left")) = .81;
    x0_floating_(position_map_floating.at("ankle_joint_right")) = .81;
    x0_floating_(position_map_floating.at("toe_left")) = 0;
    x0_floating_(position_map_floating.at("toe_right")) = 0;
    // x0_floating_(position_map_floating.at("toe_left")) = -30.0 * M_PI /
    // 180.0;
    // x0_floating_(position_map_floating.at("toe_right")) = -60.0 * M_PI /
    // 180.0;

    // Collison detect
    // Contact information is specific to the floating base RBT
    VectorXd phi_total;
    Matrix3Xd normal_total, xA_total, xB_total;
    vector<int> idxA_total, idxB_total;
    KinematicsCache<double> k_cache = tree_floating_.doKinematics(
        x0_floating_.head(num_positions_floating_),
        x0_floating_.tail(num_velocities_floating_));

    tree_floating_.collisionDetect(k_cache, phi_total, normal_total, xA_total,
                                   xB_total, idxA_total, idxB_total);

    const int world_ind = GetBodyIndexFromName(tree_floating_, "world");
    const int toe_left_ind = GetBodyIndexFromName(tree_floating_, "toe_left");
    const int toe_right_ind = GetBodyIndexFromName(tree_floating_, "toe_right");

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
    contact_info_ = {xB, idxB};

    num_contacts_ = contact_info_.num_contacts;
    num_position_constraints_ = tree_floating_.getNumPositionConstraints();
    num_forces_ = num_contacts_ * 3 + num_position_constraints_;
  }

  RigidBodyTree<double> tree_fixed_;
  RigidBodyTree<double> tree_floating_;
  int num_positions_fixed_;
  int num_positions_floating_;
  int num_velocities_fixed_;
  int num_velocities_floating_;
  int num_states_fixed_;
  int num_states_floating_;
  int num_efforts_fixed_;
  int num_efforts_floating_;
  int num_contacts_;
  int num_position_constraints_;
  int num_forces_;
  ContactInfo contact_info_;
  VectorXd x0_fixed_;
  VectorXd x0_floating_;
};

TEST_F(MultibodySolversTest, InitializationTest) {
  // Constraint class initializations
  // Position constraint

  PositionConstraint position_constraint_fixed(tree_fixed_);
  PositionConstraint position_constraint_floating(tree_floating_);

  // Contact constraint
  ContactConstraint contact_constraint(tree_floating_, contact_info_);

  // Fixed point constraint with and without contact
  FixedPointConstraint fp_constraint1(tree_fixed_);
  FixedPointConstraint fp_constraint2(tree_floating_, contact_info_);

  // Solver class initializations

  // PositionSolver
  // Testing basic getters and setters
  PositionSolver position_solver_fixed(tree_floating_);
  position_solver_fixed.set_filename("position_log");
  position_solver_fixed.set_major_tolerance(0.001);
  position_solver_fixed.set_minor_tolerance(0.01);

  ASSERT_EQ(position_solver_fixed.get_filename(), "position_log");
  ASSERT_EQ(position_solver_fixed.get_major_tolerance(), 0.001);
  ASSERT_EQ(position_solver_fixed.get_minor_tolerance(), 0.01);

  PositionSolver position_solver_floating(tree_floating_);
  position_solver_floating.set_filename("position_log");
  position_solver_floating.set_major_tolerance(0.001);
  position_solver_floating.set_minor_tolerance(0.01);

  ASSERT_EQ(position_solver_floating.get_filename(), "position_log");
  ASSERT_EQ(position_solver_floating.get_major_tolerance(), 0.001);
  ASSERT_EQ(position_solver_floating.get_minor_tolerance(), 0.01);

  // Testing mathematical program getter
  shared_ptr<MathematicalProgram> prog_position_fixed =
      position_solver_fixed.get_program();
  shared_ptr<MathematicalProgram> prog_position_floating =
      position_solver_floating.get_program();

  // ContactSolver
  // Testing basic getters and setters
  ContactSolver contact_solver(tree_floating_, contact_info_);
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
  FixedPointSolver fp_solver_fixed(tree_fixed_);
  fp_solver_fixed.set_filename("fp_log");
  fp_solver_fixed.set_major_tolerance(0.003);
  fp_solver_fixed.set_minor_tolerance(0.03);

  ASSERT_EQ(fp_solver_fixed.get_filename(), "fp_log");
  ASSERT_EQ(fp_solver_fixed.get_major_tolerance(), 0.003);
  ASSERT_EQ(fp_solver_fixed.get_minor_tolerance(), 0.03);

  FixedPointSolver fp_solver_floating(tree_floating_, contact_info_);
  fp_solver_floating.set_filename("fp_log");
  fp_solver_floating.set_major_tolerance(0.003);
  fp_solver_floating.set_minor_tolerance(0.03);

  ASSERT_EQ(fp_solver_floating.get_filename(), "fp_log");
  ASSERT_EQ(fp_solver_floating.get_major_tolerance(), 0.003);
  ASSERT_EQ(fp_solver_floating.get_minor_tolerance(), 0.03);

  // Testing mathematical program getter
  shared_ptr<MathematicalProgram> prog_fp_fixed = fp_solver_fixed.get_program();
  shared_ptr<MathematicalProgram> prog_fp_floating =
      fp_solver_floating.get_program();
}

TEST_F(MultibodySolversTest, SolveTest) {
  // Testing the solvers

  VectorXd q_fixed = x0_fixed_.head(num_positions_fixed_);
  VectorXd u_fixed = VectorXd::Zero(num_efforts_fixed_);
  VectorXd lambda_fixed = VectorXd::Zero(num_position_constraints_);

  VectorXd q_floating = x0_floating_.head(num_positions_floating_);
  VectorXd u_floating = VectorXd::Zero(num_efforts_floating_);
  VectorXd lambda_floating = VectorXd::Zero(num_position_constraints_);

  // PositionSolver
  // Fixed base
  PositionSolver position_solver_fixed(tree_fixed_);
  position_solver_fixed.SetInitialGuessQ(q_fixed);

  cout << "Position solver result (Fixed base): "
       << position_solver_fixed.Solve(q_fixed) << endl;

  VectorXd q_sol_fixed = position_solver_fixed.GetSolutionQ();

  // Solution dimension check
  ASSERT_EQ(q_sol_fixed.size(), num_positions_fixed_);
  // Checking if the solution constraints have been satisfied
  ASSERT_TRUE(position_solver_fixed.CheckConstraint(q_sol_fixed));

  // Floating base
  PositionSolver position_solver_floating(tree_floating_);
  position_solver_floating.SetInitialGuessQ(q_floating);

  cout << "Position solver result (Floating base): "
       << position_solver_floating.Solve(q_floating) << endl;

  VectorXd q_sol_floating = position_solver_floating.GetSolutionQ();

  // Solution dimension check
  ASSERT_EQ(q_sol_floating.size(), num_positions_floating_);
  // Checking if the solution constraints have been satisfied
  ASSERT_TRUE(position_solver_floating.CheckConstraint(q_sol_floating));

  // ContactSolver
  ContactSolver contact_solver(tree_floating_, contact_info_);
  contact_solver.SetInitialGuessQ(q_floating);

  std::cout << "Contact solver result (Floating base): "
            << contact_solver.Solve(q_floating) << std::endl;

  q_sol_floating = contact_solver.GetSolutionQ();

  // Solution dimension check
  ASSERT_EQ(q_sol_floating.size(), num_positions_floating_);
  // Checking if the solution constraints have been satisfied
  ASSERT_TRUE(contact_solver.CheckConstraint(q_sol_floating));

  // FixedPointSolver
  // Fixed base
  FixedPointSolver fp_solver_fixed(tree_fixed_);
  // fp_solver.SetInitialGuess(q, u, lambda);
  // fp_solver.AddSpreadNormalForcesCost();
  // fp_solver.AddFrictionConeConstraint(0.8);

  cout << "Fixed point solver result: "
       << fp_solver_fixed.Solve(q_fixed, u_fixed) << endl;

  q_sol_fixed = fp_solver_fixed.GetSolutionQ();
  VectorXd u_sol_fixed = fp_solver_fixed.GetSolutionU();
  VectorXd lambda_sol_fixed = fp_solver_fixed.GetSolutionLambda();

  VectorXd x_sol_fixed =
      VectorXd::Zero(num_positions_fixed_ + num_velocities_fixed_);
  x_sol_fixed.head(num_positions_fixed_) = q_sol_fixed;
  // ContactToolkit object with empty contacts (Default)
  ContactToolkit<double> ct_fixed(tree_fixed_, ContactInfo());
  cout << ct_fixed.CalcMVDot(x_sol_fixed, u_sol_fixed, lambda_sol_fixed)
              .transpose()
       << endl;
  cout << "qdot ----------------------" << endl;
  cout << ct_fixed
              .CalcTimeDerivatives(x_sol_fixed, u_sol_fixed, lambda_sol_fixed)
              .transpose()
       << endl;
  cout << endl;
  cout << "q ----------------------" << endl;
  cout << q_sol_fixed.transpose() << endl;
  cout << std::endl;
  cout << "u ----------------------" << endl;
  cout << u_sol_fixed.transpose() << endl;
  cout << endl;
  cout << "lambda ----------------------" << endl;
  cout << lambda_sol_fixed.transpose() << endl;
  cout << endl;

  // Solution dimension check
  ASSERT_EQ(q_sol_fixed.size(), num_positions_fixed_);
  ASSERT_EQ(u_sol_fixed.size(), num_efforts_fixed_);
  ASSERT_EQ(lambda_sol_fixed.size(), num_position_constraints_);
  // Solution constraints check
  ASSERT_TRUE(fp_solver_fixed.CheckConstraint(q_sol_fixed, u_sol_fixed,
                                              lambda_sol_fixed));
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
