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
using drake::multibody::joints::kFixed;
using drake::multibody::joints::kRollPitchYaw;
using drake::multibody::joints::kQuaternion;
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
    buildCassieTree(tree_rpy_, filename, kRollPitchYaw);
    buildCassieTree(tree_quaternion_, filename, kQuaternion);

    // Adding the ground
    AddFlatTerrainToWorld(&tree_rpy_, 4, 0.05);
    AddFlatTerrainToWorld(&tree_quaternion_, 4, 0.05);

    // Dimensions for fixed and floating based models
    num_positions_fixed_ = tree_fixed_.get_num_positions();
    num_velocities_fixed_ = tree_fixed_.get_num_velocities();
    num_states_fixed_ = num_positions_fixed_ + num_velocities_fixed_;
    num_efforts_fixed_ = tree_fixed_.get_num_actuators();

    num_positions_rpy_ = tree_rpy_.get_num_positions();
    num_velocities_rpy_ = tree_rpy_.get_num_velocities();
    num_states_rpy_ = num_positions_rpy_ + num_velocities_rpy_;
    num_efforts_rpy_ = tree_rpy_.get_num_actuators();

    num_positions_quaternion_ = tree_quaternion_.get_num_positions();
    num_velocities_quaternion_ = tree_quaternion_.get_num_velocities();
    num_states_quaternion_ =
        num_positions_quaternion_ + num_velocities_quaternion_;
    num_efforts_quaternion_ = tree_quaternion_.get_num_actuators();

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

    map<string, int> position_map_rpy =
        tree_rpy_.computePositionNameToIndexMap();
    x0_rpy_ = VectorXd::Zero(num_states_rpy_);
    x0_rpy_(position_map_rpy.at("base_z")) = 3;
    x0_rpy_(position_map_rpy.at("hip_roll_left")) = 0.1;
    x0_rpy_(position_map_rpy.at("hip_roll_right")) = -0.1;
    x0_rpy_(position_map_rpy.at("hip_yaw_left")) = 0.01;
    x0_rpy_(position_map_rpy.at("hip_yaw_right")) = -0.01;
    x0_rpy_(position_map_rpy.at("hip_pitch_left")) = .269;
    x0_rpy_(position_map_rpy.at("hip_pitch_right")) = .269;
    x0_rpy_(position_map_rpy.at("knee_left")) = -.744;
    x0_rpy_(position_map_rpy.at("knee_right")) = -.744;
    x0_rpy_(position_map_rpy.at("ankle_joint_left")) = .81;
    x0_rpy_(position_map_rpy.at("ankle_joint_right")) = .81;
    x0_rpy_(position_map_rpy.at("toe_left")) = -60.0 * M_PI / 180.0;
    x0_rpy_(position_map_rpy.at("toe_right")) = -60.0 * M_PI / 180.0;

    map<string, int> position_map_quaternion_ =
        tree_quaternion_.computePositionNameToIndexMap();
    x0_quaternion_ = VectorXd::Zero(num_states_quaternion_);
    x0_quaternion_(position_map_quaternion_.at("base_z")) = 3;
    x0_quaternion_(position_map_quaternion_.at("base_qw")) = 1.0;
    x0_quaternion_(position_map_quaternion_.at("hip_roll_left")) = 0.1;
    x0_quaternion_(position_map_quaternion_.at("hip_roll_right")) = -0.1;
    x0_quaternion_(position_map_quaternion_.at("hip_yaw_left")) = 0.01;
    x0_quaternion_(position_map_quaternion_.at("hip_yaw_right")) = -0.01;
    x0_quaternion_(position_map_quaternion_.at("hip_pitch_left")) = .269;
    x0_quaternion_(position_map_quaternion_.at("hip_pitch_right")) = .269;
    x0_quaternion_(position_map_quaternion_.at("knee_left")) = -.744;
    x0_quaternion_(position_map_quaternion_.at("knee_right")) = -.744;
    x0_quaternion_(position_map_quaternion_.at("ankle_joint_left")) = .81;
    x0_quaternion_(position_map_quaternion_.at("ankle_joint_right")) = .81;
    x0_quaternion_(position_map_quaternion_.at("toe_left")) =
        -60.0 * M_PI / 180.0;
    x0_quaternion_(position_map_quaternion_.at("toe_right")) =
        -60.0 * M_PI / 180.0;

    // Setting up fixed_joints_vector_ and fixed_joints_map_
    fixed_joints_vector_rpy_.push_back(position_map_rpy.at("base_roll"));
    fixed_joints_vector_rpy_.push_back(position_map_rpy.at("base_pitch"));
    fixed_joints_vector_rpy_.push_back(position_map_rpy.at("base_yaw"));
    fixed_joints_vector_rpy_.push_back(position_map_rpy.at("hip_pitch_left"));
    fixed_joints_vector_rpy_.push_back(position_map_rpy.at("hip_pitch_right"));

    fixed_joints_vector_quaternion_.push_back(
        position_map_quaternion_.at("base_qw"));
    fixed_joints_vector_quaternion_.push_back(
        position_map_quaternion_.at("base_qx"));
    fixed_joints_vector_quaternion_.push_back(
        position_map_quaternion_.at("base_qy"));
    fixed_joints_vector_quaternion_.push_back(
        position_map_quaternion_.at("base_qz"));
    fixed_joints_vector_quaternion_.push_back(
        position_map_quaternion_.at("hip_pitch_left"));
    fixed_joints_vector_quaternion_.push_back(
        position_map_quaternion_.at("hip_pitch_right"));

    for (auto& ind : fixed_joints_vector_rpy_) {
      fixed_joints_map_rpy_[ind] = x0_rpy_(ind);
    }

    for (auto& ind : fixed_joints_vector_quaternion_) {
      fixed_joints_map_quaternion_[ind] = x0_quaternion_(ind);
    }

    // Collison detect
    // Contact information for the rpy and quaternion models.
    VectorXd phi_total_rpy;
    Matrix3Xd normal_total_rpy, xA_total_rpy, xB_total_rpy;
    vector<int> idxA_total_rpy, idxB_total_rpy;
    KinematicsCache<double> k_cache_rpy = tree_rpy_.doKinematics(
        x0_rpy_.head(num_positions_rpy_), x0_rpy_.tail(num_velocities_rpy_));

    VectorXd phi_total_quaternion;
    Matrix3Xd normal_total_quaternion, xA_total_quaternion, xB_total_quaternion;
    vector<int> idxA_total_quaternion, idxB_total_quaternion;
    KinematicsCache<double> k_cache_quaternion = tree_quaternion_.doKinematics(
        x0_quaternion_.head(num_positions_quaternion_),
        x0_quaternion_.tail(num_velocities_quaternion_));

    tree_rpy_.collisionDetect(k_cache_rpy, phi_total_rpy, normal_total_rpy,
                              xA_total_rpy, xB_total_rpy, idxA_total_rpy,
                              idxB_total_rpy);

    tree_quaternion_.collisionDetect(
        k_cache_quaternion, phi_total_quaternion, normal_total_quaternion,
        xA_total_quaternion, xB_total_quaternion, idxA_total_quaternion,
        idxB_total_quaternion);

    const int world_ind_rpy = GetBodyIndexFromName(tree_rpy_, "world");
    const int toe_left_ind_rpy = GetBodyIndexFromName(tree_rpy_, "toe_left");
    const int toe_right_ind_rpy = GetBodyIndexFromName(tree_rpy_, "toe_right");

    const int world_ind_quaternion =
        GetBodyIndexFromName(tree_quaternion_, "world");
    const int toe_left_ind_quaternion =
        GetBodyIndexFromName(tree_quaternion_, "toe_left");
    const int toe_right_ind_quaternion =
        GetBodyIndexFromName(tree_quaternion_, "toe_right");

    // Extracting information into the four contacts.
    VectorXd phi_rpy(4);
    Matrix3Xd normal_rpy(3, 4), xA_rpy(3, 4), xB_rpy(3, 4);
    vector<int> idxA_rpy(4), idxB_rpy(4);

    VectorXd phi_quaternion(4);
    Matrix3Xd normal_quaternion(3, 4), xA_quaternion(3, 4), xB_quaternion(3, 4);
    vector<int> idxA_quaternion(4), idxB_quaternion(4);

    int k = 0;
    for (unsigned i = 0; i < idxA_total_rpy.size(); ++i) {
      int ind_a = idxA_total_rpy.at(i);
      int ind_b = idxB_total_rpy.at(i);
      if ((ind_a == world_ind_rpy && ind_b == toe_left_ind_rpy) ||
          (ind_a == world_ind_rpy && ind_b == toe_right_ind_rpy) ||
          (ind_a == toe_left_ind_rpy && ind_b == world_ind_rpy) ||
          (ind_a == toe_right_ind_rpy && ind_b == world_ind_rpy)) {
        xA_rpy.col(k) = xA_total_rpy.col(i);
        xB_rpy.col(k) = xB_total_rpy.col(i);
        idxA_rpy.at(k) = idxA_total_rpy.at(i);
        idxB_rpy.at(k) = idxB_total_rpy.at(i);
        ++k;
      }
    }

    k = 0;
    for (unsigned i = 0; i < idxA_total_quaternion.size(); ++i) {
      int ind_a = idxA_total_quaternion.at(i);
      int ind_b = idxB_total_quaternion.at(i);
      if ((ind_a == world_ind_quaternion && ind_b == toe_left_ind_quaternion) ||
          (ind_a == world_ind_quaternion &&
           ind_b == toe_right_ind_quaternion) ||
          (ind_a == toe_left_ind_quaternion && ind_b == world_ind_quaternion) ||
          (ind_a == toe_right_ind_quaternion &&
           ind_b == world_ind_quaternion)) {
        xA_quaternion.col(k) = xA_total_quaternion.col(i);
        xB_quaternion.col(k) = xB_total_quaternion.col(i);
        idxA_quaternion.at(k) = idxA_total_quaternion.at(i);
        idxB_quaternion.at(k) = idxB_total_quaternion.at(i);
        ++k;
      }
    }

    // Creating the contact info
    // idxB is the vector index for the body which is accessed through
    // ContactInfo.idxA
    // In this case xA corresponds to the points on the ground and hence xA
    // and
    // xB must be interchanged when constructing contact_info_
    contact_info_rpy_ = {xB_rpy, idxB_rpy};
    contact_info_quaternion_ = {xB_quaternion, idxB_quaternion};

    num_contacts_rpy_ = contact_info_rpy_.num_contacts;
    num_contacts_quaternion_ = contact_info_quaternion_.num_contacts;

    num_forces_fixed_ = tree_fixed_.getNumPositionConstraints();
    num_forces_rpy_ =
        tree_rpy_.getNumPositionConstraints() + 3 * num_contacts_rpy_;
    num_forces_quaternion_ = tree_quaternion_.getNumPositionConstraints() +
                             3 * num_contacts_quaternion_;

    q0_fixed_ = x0_fixed_.head(num_positions_fixed_);
    u0_fixed_ = VectorXd::Zero(num_efforts_fixed_);
    lambda0_fixed_ = VectorXd::Zero(num_forces_fixed_);

    q0_rpy_ = x0_rpy_.head(num_positions_rpy_);
    u0_rpy_ = VectorXd::Zero(num_efforts_rpy_);
    lambda0_rpy_ = VectorXd::Zero(num_forces_rpy_);

    q0_quaternion_ = x0_quaternion_.head(num_positions_quaternion_);
    u0_quaternion_ = VectorXd::Zero(num_efforts_quaternion_);
    lambda0_quaternion_ = VectorXd::Zero(num_forces_quaternion_);

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

    // Solving for the fixed point (RPY base).
    FixedPointSolver fp_solver_rpy(tree_rpy_, contact_info_rpy_, q0_rpy_,
                                   u0_rpy_);
    fp_solver_rpy.SetInitialGuess(q0_rpy_, u0_rpy_, lambda0_rpy_);
    fp_solver_rpy.AddSpreadNormalForcesCost();
    fp_solver_rpy.AddFixedJointsConstraint(fixed_joints_map_rpy_);
    fp_solver_rpy.AddJointLimitConstraint(0.001);

    MathematicalProgramResult program_result_rpy = fp_solver_rpy.Solve();
    q_rpy_ = fp_solver_rpy.GetSolutionQ();
    u_rpy_ = fp_solver_rpy.GetSolutionU();
    lambda_rpy_ = fp_solver_rpy.GetSolutionLambda();

    Q_rpy_ = MatrixXd::Identity(num_states_rpy_, num_states_rpy_);
    R_rpy_ = MatrixXd::Identity(num_efforts_rpy_, num_efforts_rpy_);

    // Initialized the floating base constrained LQR controller.
    clqr_controller_rpy_ = make_unique<ConstrainedLQRController>(
        tree_rpy_, q_rpy_, u_rpy_, lambda_rpy_, Q_rpy_, R_rpy_,
        contact_info_rpy_);

    // Solving for the fixed point (Quaternion base).
    FixedPointSolver fp_solver_quaternion(tree_quaternion_,
                                          contact_info_quaternion_,
                                          q0_quaternion_, u0_quaternion_);
    fp_solver_quaternion.SetInitialGuess(q0_quaternion_, u0_quaternion_,
                                         lambda0_quaternion_);
    fp_solver_quaternion.AddSpreadNormalForcesCost();
    fp_solver_quaternion.AddFixedJointsConstraint(fixed_joints_map_quaternion_);
    fp_solver_quaternion.AddJointLimitConstraint(0.001);

    MathematicalProgramResult program_result_quaternion =
        fp_solver_quaternion.Solve();
    q_quaternion_ = fp_solver_quaternion.GetSolutionQ();
    u_quaternion_ = fp_solver_quaternion.GetSolutionU();
    lambda_quaternion_ = fp_solver_quaternion.GetSolutionLambda();

    Q_quaternion_ =
        MatrixXd::Identity(num_states_quaternion_, num_states_quaternion_);
    R_quaternion_ =
        MatrixXd::Identity(num_efforts_quaternion_, num_efforts_quaternion_);

    // Initialized the floating base constrained LQR controller.
    clqr_controller_quaternion_ = make_unique<ConstrainedLQRController>(
        tree_quaternion_, q_quaternion_, u_quaternion_, lambda_quaternion_,
        Q_quaternion_, R_quaternion_, contact_info_quaternion_);
  }

  RigidBodyTree<double> tree_fixed_;
  RigidBodyTree<double> tree_rpy_;
  RigidBodyTree<double> tree_quaternion_;
  vector<int> fixed_joints_vector_rpy_;
  vector<int> fixed_joints_vector_quaternion_;
  map<int, double> fixed_joints_map_rpy_;
  map<int, double> fixed_joints_map_quaternion_;
  int num_positions_fixed_;
  int num_velocities_fixed_;
  int num_states_fixed_;
  int num_efforts_fixed_;
  int num_forces_fixed_;
  int num_positions_rpy_;
  int num_velocities_rpy_;
  int num_states_rpy_;
  int num_efforts_rpy_;
  int num_forces_rpy_;
  int num_positions_quaternion_;
  int num_velocities_quaternion_;
  int num_states_quaternion_;
  int num_efforts_quaternion_;
  int num_forces_quaternion_;
  int num_contacts_rpy_;
  int num_contacts_quaternion_;
  ContactInfo contact_info_rpy_;
  ContactInfo contact_info_quaternion_;
  VectorXd x0_fixed_;
  VectorXd x0_rpy_;
  VectorXd x0_quaternion_;
  VectorXd q0_fixed_;
  VectorXd q0_rpy_;
  VectorXd q0_quaternion_;
  VectorXd u0_fixed_;
  VectorXd u0_rpy_;
  VectorXd u0_quaternion_;
  VectorXd lambda0_fixed_;
  VectorXd lambda0_rpy_;
  VectorXd lambda0_quaternion_;
  VectorXd q_fixed_;
  VectorXd q_rpy_;
  VectorXd q_quaternion_;
  VectorXd u_fixed_;
  VectorXd u_rpy_;
  VectorXd u_quaternion_;
  VectorXd lambda_fixed_;
  VectorXd lambda_rpy_;
  VectorXd lambda_quaternion_;
  MatrixXd Q_fixed_;
  MatrixXd Q_rpy_;
  MatrixXd Q_quaternion_;
  MatrixXd R_fixed_;
  MatrixXd R_rpy_;
  MatrixXd R_quaternion_;
  unique_ptr<ConstrainedLQRController> clqr_controller_fixed_;
  unique_ptr<ConstrainedLQRController> clqr_controller_rpy_;
  unique_ptr<ConstrainedLQRController> clqr_controller_quaternion_;
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
  // Rpy base
  // Running similar tests for the floating base getters.
  // Dimensions of K and E
  ASSERT_EQ(clqr_controller_rpy_->get_K().rows(), num_efforts_rpy_);
  ASSERT_EQ(clqr_controller_rpy_->get_K().cols(), num_states_rpy_);
  ASSERT_EQ(clqr_controller_rpy_->get_E().size(), num_efforts_rpy_);

  // Verifying the desired state
  ASSERT_TRUE(clqr_controller_rpy_->get_desired_state()
                  .head(num_positions_rpy_)
                  .isApprox(q_rpy_));
  ASSERT_TRUE(clqr_controller_rpy_->get_desired_state()
                  .tail(num_velocities_rpy_)
                  .isApprox(VectorXd::Zero(num_velocities_rpy_)));

  // Dimensions of the A and B matrices that are in the reduced coordinates
  ASSERT_EQ(clqr_controller_rpy_->get_A().rows(),
            num_states_rpy_ - 2 * num_forces_rpy_);
  ASSERT_EQ(clqr_controller_rpy_->get_A().cols(),
            num_states_rpy_ - 2 * num_forces_rpy_);
  ASSERT_EQ(clqr_controller_rpy_->get_B().rows(),
            num_states_rpy_ - 2 * num_forces_rpy_);
  ASSERT_EQ(clqr_controller_rpy_->get_B().cols(), num_efforts_rpy_);

  // Quaternion base
  // Running similar tests for the floating base getters.
  // Dimensions of K and E
  ASSERT_EQ(clqr_controller_quaternion_->get_K().rows(),
            num_efforts_quaternion_);
  ASSERT_EQ(clqr_controller_quaternion_->get_K().cols(),
            num_states_quaternion_);
  ASSERT_EQ(clqr_controller_quaternion_->get_E().size(),
            num_efforts_quaternion_);

  // Verifying the desired state
  ASSERT_TRUE(clqr_controller_quaternion_->get_desired_state()
                  .head(num_positions_quaternion_)
                  .isApprox(q_quaternion_));
  ASSERT_TRUE(clqr_controller_quaternion_->get_desired_state()
                  .tail(num_velocities_quaternion_)
                  .isApprox(VectorXd::Zero(num_velocities_quaternion_)));

  // Dimensions of the A and B matrices that are in the reduced coordinates
  ASSERT_EQ(clqr_controller_quaternion_->get_A().rows(),
            num_states_quaternion_ - 2 * num_forces_quaternion_);
  ASSERT_EQ(clqr_controller_quaternion_->get_A().cols(),
            num_states_quaternion_ - 2 * num_forces_quaternion_);
  ASSERT_EQ(clqr_controller_quaternion_->get_B().rows(),
            num_states_quaternion_ - 2 * num_forces_quaternion_);
  ASSERT_EQ(clqr_controller_quaternion_->get_B().cols(),
            num_efforts_quaternion_);
}

TEST_F(ConstrainedLQRControllerTest, TestPortsFixed) {
  // Verifying the number of input and output ports, along with their size.
  ASSERT_EQ(
      clqr_controller_fixed_->CreateDefaultContext()->num_input_ports(), 1);
  ASSERT_EQ(clqr_controller_fixed_->num_input_ports(), 1);

  ASSERT_EQ(clqr_controller_fixed_->num_output_ports(), 1);

  ASSERT_EQ(clqr_controller_fixed_->get_output_port_efforts().size(),
            num_efforts_fixed_ + 1);
}

TEST_F(ConstrainedLQRControllerTest, TestPortsFloating) {
  // Rpy base
  // Running the same port tests for the floating base controller.
  ASSERT_EQ(clqr_controller_rpy_->CreateDefaultContext()->num_input_ports(),
            1);
  ASSERT_EQ(clqr_controller_rpy_->num_input_ports(), 1);

  ASSERT_EQ(clqr_controller_rpy_->num_output_ports(), 1);

  ASSERT_EQ(clqr_controller_rpy_->get_output_port_efforts().size(),
            num_efforts_rpy_ + 1);

  // Quaternion base
  // Running the same port tests for the floating base controller.
  ASSERT_EQ(clqr_controller_quaternion_->CreateDefaultContext()
                ->num_input_ports(), 1);
  ASSERT_EQ(clqr_controller_quaternion_->num_input_ports(), 1);

  ASSERT_EQ(clqr_controller_quaternion_->num_output_ports(), 1);

  ASSERT_EQ(clqr_controller_quaternion_->get_output_port_efforts().size(),
            num_efforts_quaternion_ + 1);
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
