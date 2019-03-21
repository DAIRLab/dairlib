#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include "attic/multibody/multibody_solvers.h"
#include "attic/multibody/rigidbody_utils.h"
#include "examples/Cassie/cassie_utils.h"
#include "systems/controllers/constrained_lqr_controller.h"

namespace dairlib {
namespace systems {
namespace {

using std::map;
using std::string;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using std::make_unique;
using std::unique_ptr;
using drake::multibody::joints::kRollPitchYaw;
using drake::multibody::joints::kFixed;
using drake::multibody::AddFlatTerrainToWorld;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using dairlib::buildCassieTree;
using dairlib::multibody::ContactInfo;
using dairlib::multibody::ContactToolkit;
using dairlib::multibody::FixedPointSolver;
using dairlib::multibody::GetBodyIndexFromName;
using dairlib::systems::ConstrainedLQRController;

class ConstrainedLQRControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    string filename = "examples/Cassie/urdf/cassie_v2.urdf";
    buildCassieTree(tree_fixed_, filename, kFixed);
    buildCassieTree(tree_floating_, filename, kRollPitchYaw);

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
    x0_fixed_ = VectorXd::Zero(num_states_fixed_);
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

    map<string, int> position_map_floating =
        tree_floating_.computePositionNameToIndexMap();
    x0_floating_ = VectorXd::Zero(num_states_floating_);
    x0_floating_(position_map_floating.at("base_z")) = 3;
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
    x0_floating_(position_map_floating.at("toe_left")) = -60.0 * M_PI / 180.0;
    x0_floating_(position_map_floating.at("toe_right")) = -60.0 * M_PI / 180.0;

    // Setting up fixed_joints_vector_ and fixed_joints_map_
    fixed_joints_vector_.push_back(position_map_floating.at("base_roll"));
    fixed_joints_vector_.push_back(position_map_floating.at("base_yaw"));
    fixed_joints_vector_.push_back(position_map_floating.at("hip_pitch_left"));
    fixed_joints_vector_.push_back(position_map_floating.at("hip_pitch_right"));

    for (auto& ind : fixed_joints_vector_) {
      fixed_joints_map_[ind] = x0_floating_(ind);
    }

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

    num_forces_fixed_ = tree_fixed_.getNumPositionConstraints();

    num_forces_floating_ =
        tree_floating_.getNumPositionConstraints() + 3 * num_contacts_;

    q0_fixed_ = x0_fixed_.head(num_positions_fixed_);
    u0_fixed_ = VectorXd::Zero(num_efforts_fixed_);
    lambda0_fixed_ = VectorXd::Zero(num_forces_fixed_);

    q0_floating_ = x0_floating_.head(num_positions_floating_);
    u0_floating_ = VectorXd::Zero(num_efforts_floating_);
    lambda0_floating_ = VectorXd::Zero(num_forces_floating_);

    // Solving for the fixed point (Fixed base).
    FixedPointSolver fp_solver_fixed(tree_fixed_, q0_fixed_, u0_fixed_);
    fp_solver_fixed.SetInitialGuess(q0_fixed_, u0_fixed_, lambda0_fixed_);
    fp_solver_fixed.AddJointLimitConstraint(0.001);

    MathematicalProgramResult program_result_fixed = fp_solver_fixed.Solve();
    q_fixed_ = fp_solver_fixed.GetSolutionQ();
    u_fixed_ = fp_solver_fixed.GetSolutionU();
    lambda_fixed_ = fp_solver_fixed.GetSolutionLambda();

    Q_fixed_ = MatrixXd::Identity(num_states_fixed_, num_states_fixed_);
    R_fixed_ = MatrixXd::Identity(num_efforts_fixed_, num_efforts_fixed_);

    // Initialized the fixed base constrained LQR controller.
    clqr_controller_fixed_ = make_unique<ConstrainedLQRController>(
        tree_fixed_, q_fixed_, u_fixed_, lambda_fixed_, Q_fixed_, R_fixed_);

    // Solving for the fixed point (Floating base).
    FixedPointSolver fp_solver_floating(tree_floating_, contact_info_,
                                        q0_floating_, u0_floating_);
    fp_solver_floating.SetInitialGuess(q0_floating_, u0_floating_,
                                       lambda0_floating_);
    fp_solver_floating.AddSpreadNormalForcesCost();
    fp_solver_floating.AddFixedJointsConstraint(fixed_joints_map_);
    fp_solver_floating.AddJointLimitConstraint(0.001);

    MathematicalProgramResult program_result_floating =
        fp_solver_floating.Solve();
    q_floating_ = fp_solver_floating.GetSolutionQ();
    u_floating_ = fp_solver_floating.GetSolutionU();
    lambda_floating_ = fp_solver_floating.GetSolutionLambda();

    Q_floating_ =
        MatrixXd::Identity(num_states_floating_, num_states_floating_);
    R_floating_ =
        MatrixXd::Identity(num_efforts_floating_, num_efforts_floating_);

    // Initialized the floating base constrained LQR controller.
    clqr_controller_floating_ = make_unique<ConstrainedLQRController>(
        tree_floating_, q_floating_, u_floating_, lambda_floating_, Q_floating_,
        R_floating_, contact_info_);
  }

  RigidBodyTree<double> tree_fixed_;
  RigidBodyTree<double> tree_floating_;
  vector<int> fixed_joints_vector_;
  map<int, double> fixed_joints_map_;
  int num_positions_fixed_;
  int num_velocities_fixed_;
  int num_states_fixed_;
  int num_efforts_fixed_;
  int num_forces_fixed_;
  int num_positions_floating_;
  int num_velocities_floating_;
  int num_states_floating_;
  int num_efforts_floating_;
  int num_forces_floating_;
  int num_contacts_;
  ContactInfo contact_info_;
  VectorXd x0_fixed_;
  VectorXd x0_floating_;
  VectorXd q0_fixed_;
  VectorXd q0_floating_;
  VectorXd u0_fixed_;
  VectorXd u0_floating_;
  VectorXd lambda0_fixed_;
  VectorXd lambda0_floating_;
  VectorXd q_fixed_;
  VectorXd q_floating_;
  VectorXd u_fixed_;
  VectorXd u_floating_;
  VectorXd lambda_fixed_;
  VectorXd lambda_floating_;
  MatrixXd Q_fixed_;
  MatrixXd Q_floating_;
  MatrixXd R_fixed_;
  MatrixXd R_floating_;
  unique_ptr<ConstrainedLQRController> clqr_controller_fixed_;
  unique_ptr<ConstrainedLQRController> clqr_controller_floating_;
};

TEST_F(ConstrainedLQRControllerTest, TestGettersFixed) {
  // Testing the getters for the fixed base controller.
  // Dimensions of K and E
  ASSERT_EQ(clqr_controller_fixed_->get_K().rows(), num_efforts_fixed_);
  ASSERT_EQ(clqr_controller_fixed_->get_K().cols(), num_states_fixed_);

  ASSERT_EQ(clqr_controller_fixed_->get_E().size(), num_efforts_fixed_);

  // Verifying the desired state
  ASSERT_TRUE(clqr_controller_fixed_->get_desired_state()
                  .head(num_positions_fixed_)
                  .isApprox(q_fixed_));
  ASSERT_TRUE(clqr_controller_fixed_->get_desired_state()
                  .tail(num_velocities_fixed_)
                  .isApprox(VectorXd::Zero(num_velocities_fixed_)));

  // Dimensions of the A and B matrices that are in the reduced coordinates
  ASSERT_EQ(clqr_controller_fixed_->get_A().rows(),
            num_states_fixed_ - 2 * num_forces_fixed_);
  ASSERT_EQ(clqr_controller_fixed_->get_A().cols(),
            num_states_fixed_ - 2 * num_forces_fixed_);
  ASSERT_EQ(clqr_controller_fixed_->get_B().rows(),
            num_states_fixed_ - 2 * num_forces_fixed_);
  ASSERT_EQ(clqr_controller_fixed_->get_B().cols(), num_efforts_fixed_);
}

TEST_F(ConstrainedLQRControllerTest, TestGettersFloating) {
  // Running similar tests for the floating base getters.
  // Dimensions of K and E
  ASSERT_EQ(clqr_controller_floating_->get_K().rows(), num_efforts_floating_);
  ASSERT_EQ(clqr_controller_floating_->get_K().cols(), num_states_floating_);

  ASSERT_EQ(clqr_controller_floating_->get_E().size(), num_efforts_floating_);

  // Verifying the desired state
  ASSERT_TRUE(clqr_controller_floating_->get_desired_state()
                  .head(num_positions_floating_)
                  .isApprox(q_floating_));
  ASSERT_TRUE(clqr_controller_floating_->get_desired_state()
                  .tail(num_velocities_floating_)
                  .isApprox(VectorXd::Zero(num_velocities_floating_)));

  // Dimensions of the A and B matrices that are in the reduced coordinates
  ASSERT_EQ(clqr_controller_floating_->get_A().rows(),
            num_states_floating_ - 2 * num_forces_floating_);
  ASSERT_EQ(clqr_controller_floating_->get_A().cols(),
            num_states_floating_ - 2 * num_forces_floating_);
  ASSERT_EQ(clqr_controller_floating_->get_B().rows(),
            num_states_floating_ - 2 * num_forces_floating_);
  ASSERT_EQ(clqr_controller_floating_->get_B().cols(), num_efforts_floating_);
}

TEST_F(ConstrainedLQRControllerTest, TestPortsFixed) {
  // Verifying the number of input and output ports, along with their size.
  ASSERT_EQ(
      clqr_controller_fixed_->CreateDefaultContext()->get_num_input_ports(), 1);
  ASSERT_EQ(clqr_controller_fixed_->get_num_input_ports(), 1);

  ASSERT_EQ(clqr_controller_fixed_->get_num_output_ports(), 1);

  ASSERT_EQ(clqr_controller_fixed_->get_output_port_efforts().size(),
            num_efforts_fixed_ + 1);
}

TEST_F(ConstrainedLQRControllerTest, TestPortsFloating) {
  // Running the same port tests for the floating base controller.
  ASSERT_EQ(
      clqr_controller_floating_->CreateDefaultContext()->get_num_input_ports(),
      1);
  ASSERT_EQ(clqr_controller_floating_->get_num_input_ports(), 1);

  ASSERT_EQ(clqr_controller_floating_->get_num_output_ports(), 1);

  ASSERT_EQ(clqr_controller_floating_->get_output_port_efforts().size(),
            num_efforts_floating_ + 1);
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
