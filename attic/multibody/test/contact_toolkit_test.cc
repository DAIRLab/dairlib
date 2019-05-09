#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include "attic/multibody/contact_toolkit.h"
#include "attic/multibody/rigidbody_utils.h"
#include "examples/Cassie/cassie_utils.h"

namespace dairlib {
namespace multibody {
namespace {

using std::unique_ptr;
using std::make_unique;
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
using drake::multibody::joints::kRollPitchYaw;
using drake::multibody::joints::kQuaternion;
using drake::multibody::AddFlatTerrainToWorld;
using dairlib::buildCassieTree;
using dairlib::multibody::ContactToolkit;
using dairlib::multibody::ContactInfo;
using dairlib::multibody::GetBodyIndexFromName;

class ContactToolkitTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setting the random seed
    std::srand((unsigned int)time(0));

    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf";
    buildCassieTree(tree_rpy_, filename, kRollPitchYaw);
    buildCassieTree(tree_quaternion_, filename, kQuaternion);

    // Adding the ground
    AddFlatTerrainToWorld(&tree_rpy_, 100, 0.2);
    AddFlatTerrainToWorld(&tree_quaternion_, 100, 0.2);

    const int num_positions_rpy = tree_rpy_.get_num_positions();
    const int num_velocities_rpy = tree_rpy_.get_num_velocities();
    const int num_states_rpy = num_positions_rpy + num_velocities_rpy;

    const int num_positions_quaternion = tree_quaternion_.get_num_positions();
    const int num_velocities_quaternion = tree_quaternion_.get_num_velocities();
    const int num_states_quaternion =
        num_positions_quaternion + num_velocities_quaternion;

    // Setting the initial Cassie joint angles

    map<string, int> position_map_rpy =
        tree_rpy_.computePositionNameToIndexMap();
    x0_rpy_ = VectorXd::Zero(num_states_rpy);
    x0_rpy_(position_map_rpy.at("hip_roll_left")) = 0.1;
    x0_rpy_(position_map_rpy.at("hip_roll_right")) = -0.01;
    x0_rpy_(position_map_rpy.at("hip_yaw_left")) = 0.01;
    x0_rpy_(position_map_rpy.at("hip_yaw_right")) = 0.01;
    x0_rpy_(position_map_rpy.at("hip_pitch_left")) = -.169;
    x0_rpy_(position_map_rpy.at("hip_pitch_right")) = .269;
    x0_rpy_(position_map_rpy.at("knee_left")) = -.744;
    x0_rpy_(position_map_rpy.at("knee_right")) = -.744;
    x0_rpy_(position_map_rpy.at("ankle_joint_left")) = .81;
    x0_rpy_(position_map_rpy.at("ankle_joint_right")) = .81;
    x0_rpy_(position_map_rpy.at("toe_left")) = -60.0 * M_PI / 180.0;
    x0_rpy_(position_map_rpy.at("toe_right")) = -60.0 * M_PI / 180.0;

    map<string, int> position_map_quaternion =
        tree_quaternion_.computePositionNameToIndexMap();
    x0_quaternion_ = VectorXd::Zero(num_states_quaternion);
    x0_quaternion_(position_map_quaternion.at("base_qw")) = 1.0;
    x0_quaternion_(position_map_quaternion.at("hip_roll_left")) = 0.1;
    x0_quaternion_(position_map_quaternion.at("hip_roll_right")) = -0.01;
    x0_quaternion_(position_map_quaternion.at("hip_yaw_left")) = 0.01;
    x0_quaternion_(position_map_quaternion.at("hip_yaw_right")) = 0.01;
    x0_quaternion_(position_map_quaternion.at("hip_pitch_left")) = -.169;
    x0_quaternion_(position_map_quaternion.at("hip_pitch_right")) = .269;
    x0_quaternion_(position_map_quaternion.at("knee_left")) = -.744;
    x0_quaternion_(position_map_quaternion.at("knee_right")) = -.744;
    x0_quaternion_(position_map_quaternion.at("ankle_joint_left")) = .81;
    x0_quaternion_(position_map_quaternion.at("ankle_joint_right")) = .81;
    x0_quaternion_(position_map_quaternion.at("toe_left")) =
        -60.0 * M_PI / 180.0;
    x0_quaternion_(position_map_quaternion.at("toe_right")) =
        -60.0 * M_PI / 180.0;

    // Collison detect
    // Contact information for the rpy and quaternion models
    VectorXd phi_total_rpy;
    Matrix3Xd normal_total_rpy, xA_total_rpy, xB_total_rpy;
    vector<int> idxA_total_rpy, idxB_total_rpy;
    KinematicsCache<double> k_cache_rpy = tree_rpy_.doKinematics(
        x0_rpy_.head(num_positions_rpy), x0_rpy_.tail(num_velocities_rpy));

    VectorXd phi_total_quaternion;
    Matrix3Xd normal_total_quaternion, xA_total_quaternion, xB_total_quaternion;
    vector<int> idxA_total_quaternion, idxB_total_quaternion;
    KinematicsCache<double> k_cache_quaternion = tree_quaternion_.doKinematics(
        x0_quaternion_.head(num_positions_quaternion),
        x0_quaternion_.tail(num_velocities_quaternion));

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

    // Extracting information into the four contacts for the floating base types
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
    // In this case xA corresponds to the points on the ground and hence xB must
    // be used
    contact_info_rpy_ = {xB_rpy, idxB_rpy};
    contact_info_quaternion_ = {xB_quaternion, idxB_quaternion};

    // ContactToolkit objects for both templates and floating bases
    contact_toolkit_rpy_double_ =
        make_unique<ContactToolkit<double>>(tree_rpy_, contact_info_rpy_);
    contact_toolkit_quaternion_double_ = make_unique<ContactToolkit<double>>(
        tree_quaternion_, contact_info_quaternion_);
    contact_toolkit_rpy_autodiff_ =
        make_unique<ContactToolkit<AutoDiffXd>>(tree_rpy_, contact_info_rpy_);
    contact_toolkit_quaternion_autodiff_ =
        make_unique<ContactToolkit<AutoDiffXd>>(tree_quaternion_,
                                                contact_info_quaternion_);
  }

  RigidBodyTree<double> tree_rpy_;
  RigidBodyTree<double> tree_quaternion_;
  unique_ptr<ContactToolkit<double>> contact_toolkit_rpy_double_;
  unique_ptr<ContactToolkit<double>> contact_toolkit_quaternion_double_;
  unique_ptr<ContactToolkit<AutoDiffXd>> contact_toolkit_rpy_autodiff_;
  unique_ptr<ContactToolkit<AutoDiffXd>> contact_toolkit_quaternion_autodiff_;
  ContactInfo contact_info_rpy_;
  ContactInfo contact_info_quaternion_;
  VectorXd x0_rpy_;
  VectorXd x0_quaternion_;
};

// Tests the contact toolkit constructor initializations, getters and setters
TEST_F(ContactToolkitTest, InitializationTest) {
  // ContactInfo default values
  ContactInfo contact_info_default;
  ASSERT_TRUE(contact_info_default.xA.isApprox(Matrix3Xd::Zero(3, 1)));
  ASSERT_EQ(contact_info_default.num_contacts, 0);

  // ContactInfo construction validity check
  Matrix3Xd xa = Matrix3Xd::Zero(3, 2);
  vector<int> idxa(2, 1);
  ContactInfo contact_info_test = {xa, idxa};
  ASSERT_TRUE(contact_info_test.xA.isApprox(xa));
  ASSERT_TRUE(contact_info_test.idxA == idxa);
  ASSERT_TRUE(contact_info_test.num_contacts == 2);

  // ContactInfo getter
  contact_info_test = contact_toolkit_rpy_double_->get_contact_info();
  ASSERT_TRUE(contact_info_test.xA.isApprox(contact_info_rpy_.xA));
  ASSERT_TRUE(contact_info_test.idxA == contact_info_rpy_.idxA);
  ASSERT_TRUE(contact_info_test.num_contacts == contact_info_rpy_.num_contacts);

  contact_info_test = contact_toolkit_quaternion_double_->get_contact_info();
  ASSERT_TRUE(contact_info_test.xA.isApprox(contact_info_quaternion_.xA));
  ASSERT_TRUE(contact_info_test.idxA == contact_info_quaternion_.idxA);
  ASSERT_TRUE(contact_info_test.num_contacts ==
              contact_info_quaternion_.num_contacts);

  contact_info_test = contact_toolkit_rpy_autodiff_->get_contact_info();
  ASSERT_TRUE(contact_info_test.xA.isApprox(contact_info_rpy_.xA));
  ASSERT_TRUE(contact_info_test.idxA == contact_info_rpy_.idxA);
  ASSERT_TRUE(contact_info_test.num_contacts == contact_info_rpy_.num_contacts);

  contact_info_test = contact_toolkit_quaternion_autodiff_->get_contact_info();
  ASSERT_TRUE(contact_info_test.xA.isApprox(contact_info_quaternion_.xA));
  ASSERT_TRUE(contact_info_test.idxA == contact_info_quaternion_.idxA);
  ASSERT_TRUE(contact_info_test.num_contacts ==
              contact_info_quaternion_.num_contacts);

  // num contacts getter
  ASSERT_EQ(contact_toolkit_rpy_double_->get_num_contacts(), 4);
  ASSERT_EQ(contact_toolkit_quaternion_double_->get_num_contacts(), 4);
  ASSERT_EQ(contact_toolkit_rpy_autodiff_->get_num_contacts(), 4);
  ASSERT_EQ(contact_toolkit_quaternion_autodiff_->get_num_contacts(), 4);

  // Verifying the contact info setter
  Matrix3Xd xA(3, 2);
  vector<int> idxA = {2, 0};
  xA << 0.1, 0.2, 0.3, 0.2, -0.3, 2.3;
  contact_info_test = {xA, idxA};

  contact_toolkit_rpy_double_->set_contact_info(contact_info_test);
  ASSERT_TRUE(xA.isApprox(contact_toolkit_rpy_double_->get_contact_info().xA));
  ASSERT_TRUE(contact_toolkit_rpy_double_->get_contact_info().idxA == idxA);
  ASSERT_TRUE(contact_toolkit_rpy_double_->get_contact_info().num_contacts ==
              2);

  contact_toolkit_quaternion_double_->set_contact_info(contact_info_test);
  ASSERT_TRUE(
      xA.isApprox(contact_toolkit_quaternion_double_->get_contact_info().xA));
  ASSERT_TRUE(contact_toolkit_quaternion_double_->get_contact_info().idxA ==
              idxA);
  ASSERT_TRUE(
      contact_toolkit_quaternion_double_->get_contact_info().num_contacts == 2);

  contact_toolkit_rpy_autodiff_->set_contact_info(contact_info_test);
  ASSERT_TRUE(
      xA.isApprox(contact_toolkit_rpy_autodiff_->get_contact_info().xA));
  ASSERT_TRUE(contact_toolkit_rpy_autodiff_->get_contact_info().idxA == idxA);
  ASSERT_TRUE(contact_toolkit_rpy_autodiff_->get_contact_info().num_contacts ==
              2);

  contact_toolkit_quaternion_autodiff_->set_contact_info(contact_info_test);
  ASSERT_TRUE(
      xA.isApprox(contact_toolkit_quaternion_autodiff_->get_contact_info().xA));
  ASSERT_TRUE(contact_toolkit_quaternion_autodiff_->get_contact_info().idxA ==
              idxA);
  ASSERT_TRUE(
      contact_toolkit_quaternion_autodiff_->get_contact_info().num_contacts ==
      2);
}

// Contact Jacobian test
TEST_F(ContactToolkitTest, ContactJacobianTest) {
  VectorX<double> x_rpy_double = x0_rpy_;
  VectorX<AutoDiffXd> x_rpy_autodiff = initializeAutoDiff(x0_rpy_);
  VectorX<double> x_quaternion_double = x0_quaternion_;
  VectorX<AutoDiffXd> x_quaternion_autodiff =
      initializeAutoDiff(x0_quaternion_);
  MatrixX<double> jac_rpy_double =
      contact_toolkit_rpy_double_->CalcContactJacobian(x_rpy_double);
  MatrixX<double> jac_quaternion_double =
      contact_toolkit_quaternion_double_->CalcContactJacobian(
          x_quaternion_double);
  MatrixX<AutoDiffXd> jac_rpy_autodiff =
      contact_toolkit_rpy_autodiff_->CalcContactJacobian(x_rpy_autodiff);
  MatrixX<AutoDiffXd> jac_quaternion_autodiff =
      contact_toolkit_quaternion_autodiff_->CalcContactJacobian(
          x_quaternion_autodiff);

  // Checking dimensions of the jacobians
  // Each contact has three directional jacobian componenets - the normal and
  // two surface tangents.
  ASSERT_EQ(jac_rpy_double.rows(),
            contact_toolkit_rpy_double_->get_num_contacts() * 3);
  ASSERT_EQ(jac_quaternion_double.rows(),
            contact_toolkit_quaternion_double_->get_num_contacts() * 3);
  ASSERT_EQ(jac_rpy_double.cols(), tree_rpy_.get_num_velocities());
  ASSERT_EQ(jac_quaternion_double.cols(),
            tree_quaternion_.get_num_velocities());

  ASSERT_EQ(jac_rpy_autodiff.rows(),
            contact_toolkit_rpy_autodiff_->get_num_contacts() * 3);
  ASSERT_EQ(jac_quaternion_autodiff.rows(),
            contact_toolkit_quaternion_autodiff_->get_num_contacts() * 3);
  ASSERT_EQ(jac_rpy_autodiff.cols(), tree_rpy_.get_num_velocities());
  ASSERT_EQ(jac_quaternion_autodiff.cols(),
            tree_quaternion_.get_num_velocities());

  // Verifying that both templates return the same value
  ASSERT_TRUE(jac_rpy_double.isApprox(DiscardGradient(jac_rpy_autodiff)));
  ASSERT_TRUE(
      jac_quaternion_double.isApprox(DiscardGradient(jac_quaternion_autodiff)));
}

// MVDot test
TEST_F(ContactToolkitTest, MVDotTest) {
  VectorX<double> x_rpy_double = x0_rpy_;
  VectorX<double> u_rpy_double =
      VectorXd::Random(tree_rpy_.get_num_actuators());
  VectorX<double> lambda_rpy_double =
      VectorXd::Random(tree_rpy_.getNumPositionConstraints() +
                       contact_toolkit_rpy_double_->get_num_contacts() * 3);
  VectorX<double> x_quaternion_double = x0_quaternion_;
  VectorX<double> u_quaternion_double =
      VectorXd::Random(tree_rpy_.get_num_actuators());
  VectorX<double> lambda_quaternion_double = VectorXd::Random(
      tree_quaternion_.getNumPositionConstraints() +
      contact_toolkit_quaternion_double_->get_num_contacts() * 3);

  VectorX<double> mvdot_rpy_double = contact_toolkit_rpy_double_->CalcMVDot(
      x_rpy_double, u_rpy_double, lambda_rpy_double);
  VectorX<double> mvdot_quaternion_double =
      contact_toolkit_quaternion_double_->CalcMVDot(
          x_quaternion_double, u_quaternion_double, lambda_quaternion_double);

  VectorX<AutoDiffXd> x_rpy_autodiff = initializeAutoDiff(x_rpy_double);
  VectorX<AutoDiffXd> u_rpy_autodiff = initializeAutoDiff(u_rpy_double);
  VectorX<AutoDiffXd> lambda_rpy_autodiff =
      initializeAutoDiff(lambda_rpy_double);
  VectorX<AutoDiffXd> x_quaternion_autodiff =
      initializeAutoDiff(x_quaternion_double);
  VectorX<AutoDiffXd> u_quaternion_autodiff =
      initializeAutoDiff(u_quaternion_double);
  VectorX<AutoDiffXd> lambda_quaternion_autodiff =
      initializeAutoDiff(lambda_quaternion_double);

  VectorX<AutoDiffXd> mvdot_rpy_autodiff =
      contact_toolkit_rpy_autodiff_->CalcMVDot(x_rpy_autodiff, u_rpy_autodiff,
                                               lambda_rpy_autodiff);
  VectorX<AutoDiffXd> mvdot_quaternion_autodiff =
      contact_toolkit_quaternion_autodiff_->CalcMVDot(
          x_quaternion_autodiff, u_quaternion_autodiff,
          lambda_quaternion_autodiff);

  // Verifying the dimensions
  ASSERT_EQ(mvdot_rpy_double.rows(), tree_rpy_.get_num_velocities());
  ASSERT_EQ(mvdot_quaternion_double.rows(),
            tree_quaternion_.get_num_velocities());
  ASSERT_EQ(mvdot_rpy_autodiff.rows(), tree_rpy_.get_num_velocities());
  ASSERT_EQ(mvdot_quaternion_autodiff.rows(),
            tree_quaternion_.get_num_velocities());

  // Verifying that both templates return the same value
  ASSERT_TRUE(mvdot_rpy_double.isApprox(DiscardGradient(mvdot_rpy_autodiff)));
  ASSERT_TRUE(mvdot_quaternion_double.isApprox(
      DiscardGradient(mvdot_quaternion_autodiff)));
}

// Time derivatives test
TEST_F(ContactToolkitTest, TimeDerivativesTest) {
  VectorX<double> x_rpy_double = x0_rpy_;
  VectorX<double> u_rpy_double =
      VectorXd::Random(tree_rpy_.get_num_actuators());
  VectorX<double> lambda_rpy_double =
      VectorXd::Random(tree_rpy_.getNumPositionConstraints() +
                       contact_toolkit_rpy_double_->get_num_contacts() * 3);
  VectorX<double> x_quaternion_double = x0_quaternion_;
  VectorX<double> u_quaternion_double =
      VectorXd::Random(tree_quaternion_.get_num_actuators());
  VectorX<double> lambda_quaternion_double = VectorXd::Random(
      tree_quaternion_.getNumPositionConstraints() +
      contact_toolkit_quaternion_double_->get_num_contacts() * 3);

  VectorX<double> xdot_rpy_double =
      contact_toolkit_rpy_double_->CalcTimeDerivatives(
          x_rpy_double, u_rpy_double, lambda_rpy_double);
  VectorX<double> xdot_quaternion_double =
      contact_toolkit_quaternion_double_->CalcTimeDerivatives(
          x_quaternion_double, u_quaternion_double, lambda_quaternion_double);

  VectorX<AutoDiffXd> x_rpy_autodiff = initializeAutoDiff(x_rpy_double);
  VectorX<AutoDiffXd> u_rpy_autodiff = initializeAutoDiff(u_rpy_double);
  VectorX<AutoDiffXd> lambda_rpy_autodiff =
      initializeAutoDiff(lambda_rpy_double);
  VectorX<AutoDiffXd> x_quaternion_autodiff =
      initializeAutoDiff(x_quaternion_double);
  VectorX<AutoDiffXd> u_quaternion_autodiff =
      initializeAutoDiff(u_quaternion_double);
  VectorX<AutoDiffXd> lambda_quaternion_autodiff =
      initializeAutoDiff(lambda_quaternion_double);

  VectorX<AutoDiffXd> xdot_rpy_autodiff =
      contact_toolkit_rpy_autodiff_->CalcTimeDerivatives(
          x_rpy_autodiff, u_rpy_autodiff, lambda_rpy_autodiff);
  VectorX<AutoDiffXd> xdot_quaternion_autodiff =
      contact_toolkit_quaternion_autodiff_->CalcTimeDerivatives(
          x_quaternion_autodiff, u_quaternion_autodiff,
          lambda_quaternion_autodiff);

  // Verifying the dimensions
  ASSERT_EQ(xdot_rpy_double.rows(),
            tree_rpy_.get_num_positions() + tree_rpy_.get_num_velocities());
  ASSERT_EQ(xdot_quaternion_double.rows(),
            tree_quaternion_.get_num_positions() +
                tree_quaternion_.get_num_velocities());
  ASSERT_EQ(xdot_rpy_autodiff.rows(),
            tree_rpy_.get_num_positions() + tree_rpy_.get_num_velocities());
  ASSERT_EQ(xdot_quaternion_autodiff.rows(),
            tree_quaternion_.get_num_positions() +
                tree_quaternion_.get_num_velocities());

  // Verifying that both templates return the same value
  ASSERT_TRUE(xdot_rpy_double.isApprox(DiscardGradient(xdot_rpy_autodiff)));
  ASSERT_TRUE(xdot_quaternion_double.isApprox(
      DiscardGradient(xdot_quaternion_autodiff)));
}

}  // namespace
}  // namespace multibody
}  // namespace dairlib

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
