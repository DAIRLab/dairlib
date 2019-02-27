#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include "examples/Cassie/cassie_utils.h"
#include "attic/multibody/contact_toolkit.h"
#include "attic/multibody/rigidbody_utils.h"

namespace dairlib {
namespace systems {
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
using drake::multibody::joints::FloatingBaseType;
using drake::multibody::joints::kRollPitchYaw;
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
    FloatingBaseType base_type = kRollPitchYaw;
    buildCassieTree(tree_, filename, base_type);

    // Adding the ground
    AddFlatTerrainToWorld(&tree_, 4, 0.05);

    const int num_positions = tree_.get_num_positions();
    const int num_velocities = tree_.get_num_velocities();
    const int num_states = num_positions + num_velocities;

    // Setting the initial Cassie joint angles

    map<string, int> position_map = tree_.computePositionNameToIndexMap();
    x0_ = VectorXd::Zero(num_states);
    x0_(position_map.at("hip_roll_left")) = 0.1;
    x0_(position_map.at("hip_roll_right")) = -0.01;
    x0_(position_map.at("hip_yaw_left")) = 0.01;
    x0_(position_map.at("hip_yaw_right")) = 0.01;
    x0_(position_map.at("hip_pitch_left")) = -.169;
    x0_(position_map.at("hip_pitch_right")) = .269;
    x0_(position_map.at("knee_left")) = -.744;
    x0_(position_map.at("knee_right")) = -.744;
    x0_(position_map.at("ankle_joint_left")) = .81;
    x0_(position_map.at("ankle_joint_right")) = .81;
    x0_(position_map.at("toe_left")) = -30.0 * M_PI / 180.0;
    x0_(position_map.at("toe_right")) = -60.0 * M_PI / 180.0;

    // Colliison detect
    VectorXd phi_total;
    Matrix3Xd normal_total, xA_total, xB_total;
    vector<int> idxA_total, idxB_total;
    KinematicsCache<double> k_cache =
        tree_.doKinematics(x0_.head(num_positions), x0_.tail(num_velocities));

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
    // In this case xA corresponds to the points on the ground and hence xB must
    // be used
    contact_info_ = {xB, idxB};

    // ContactToolkit objects for both templates
    contact_toolkit_double_ =
        make_unique<ContactToolkit<double>>(tree_, contact_info_);
    contact_toolkit_autodiff_ =
        make_unique<ContactToolkit<AutoDiffXd>>(tree_, contact_info_);
  }

  RigidBodyTree<double> tree_;
  unique_ptr<ContactToolkit<double>> contact_toolkit_double_;
  unique_ptr<ContactToolkit<AutoDiffXd>> contact_toolkit_autodiff_;
  ContactInfo contact_info_;
  VectorXd x0_;
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
  contact_info_test = contact_toolkit_double_->get_contact_info();
  ASSERT_TRUE(contact_info_test.xA.isApprox(contact_info_.xA));
  ASSERT_TRUE(contact_info_test.idxA == contact_info_.idxA);
  ASSERT_TRUE(contact_info_test.num_contacts == contact_info_.num_contacts);

  contact_info_test = contact_toolkit_autodiff_->get_contact_info();
  ASSERT_TRUE(contact_info_test.xA.isApprox(contact_info_.xA));
  ASSERT_TRUE(contact_info_test.idxA == contact_info_.idxA);
  ASSERT_TRUE(contact_info_test.num_contacts == contact_info_.num_contacts);

  // num contacts getter
  ASSERT_EQ(contact_toolkit_double_->get_num_contacts(), 4);
  ASSERT_EQ(contact_toolkit_autodiff_->get_num_contacts(), 4);

  // Verifying the contact info setter
  Matrix3Xd xA(3, 2);
  vector<int> idxA = {2, 0};
  xA << 0.1, 0.2, 0.3, 0.2, -0.3, 2.3;
  contact_info_test = {xA, idxA};

  contact_toolkit_double_->set_contact_info(contact_info_test);
  ASSERT_TRUE(xA.isApprox(contact_toolkit_double_->get_contact_info().xA));
  ASSERT_TRUE(contact_toolkit_double_->get_contact_info().idxA == idxA);
  ASSERT_TRUE(contact_toolkit_double_->get_contact_info().num_contacts == 2);

  contact_toolkit_autodiff_->set_contact_info(contact_info_test);
  ASSERT_TRUE(xA.isApprox(contact_toolkit_autodiff_->get_contact_info().xA));
  ASSERT_TRUE(contact_toolkit_autodiff_->get_contact_info().idxA == idxA);
  ASSERT_TRUE(contact_toolkit_autodiff_->get_contact_info().num_contacts == 2);
}

// Contact Jacobian test
TEST_F(ContactToolkitTest, ContactJacobianTest) {
  VectorX<double> x_double = x0_;
  VectorX<AutoDiffXd> x_autodiff = initializeAutoDiff(x0_);
  MatrixX<double> jac_double =
      contact_toolkit_double_->CalcContactJacobian(x_double);
  MatrixX<AutoDiffXd> jac_autodiff =
      contact_toolkit_autodiff_->CalcContactJacobian(x_autodiff);

  // Checking dimensions of the jacobian
  // Each contact has three directional jacobian componenets - the normal and
  // two surface tangents.
  ASSERT_EQ(jac_double.rows(), contact_toolkit_double_->get_num_contacts() * 3);
  ASSERT_EQ(jac_double.cols(), tree_.get_num_positions());

  ASSERT_EQ(jac_autodiff.rows(),
            contact_toolkit_autodiff_->get_num_contacts() * 3);
  ASSERT_EQ(jac_autodiff.cols(), tree_.get_num_positions());
}

// MVDot test
TEST_F(ContactToolkitTest, MVDotTest) {
  VectorX<double> x_double = x0_;
  VectorX<double> u_double = VectorXd::Random(tree_.get_num_actuators());
  VectorX<double> lambda_double =
      VectorXd::Random(tree_.getNumPositionConstraints() +
                       contact_toolkit_double_->get_num_contacts() * 3);
  VectorX<double> mvdot_double =
      contact_toolkit_double_->CalcMVDot(x_double, u_double, lambda_double);

  VectorX<AutoDiffXd> x_autodiff = initializeAutoDiff(x_double);
  VectorX<AutoDiffXd> u_autodiff = initializeAutoDiff(u_double);
  VectorX<AutoDiffXd> lambda_autodiff = initializeAutoDiff(lambda_double);
  VectorX<AutoDiffXd> mvdot_autodiff = contact_toolkit_autodiff_->CalcMVDot(
      x_autodiff, u_autodiff, lambda_autodiff);

  // Verifying the dimensions
  ASSERT_EQ(mvdot_double.rows(), tree_.get_num_velocities());
  ASSERT_EQ(mvdot_autodiff.rows(), tree_.get_num_velocities());

  // Verifying that both templates return the same value
  ASSERT_TRUE(mvdot_double.isApprox(DiscardGradient(mvdot_autodiff)));
}

// Time derivatives test
TEST_F(ContactToolkitTest, TimeDerivativesTest) {
  VectorX<double> x_double = x0_;
  VectorX<double> u_double = VectorXd::Random(tree_.get_num_actuators());
  VectorX<double> lambda_double =
      VectorXd::Random(tree_.getNumPositionConstraints() +
                       contact_toolkit_double_->get_num_contacts() * 3);
  VectorX<double> xdot_double = contact_toolkit_double_->CalcTimeDerivatives(
      x_double, u_double, lambda_double);

  VectorX<AutoDiffXd> x_autodiff = initializeAutoDiff(x_double);
  VectorX<AutoDiffXd> u_autodiff = initializeAutoDiff(u_double);
  VectorX<AutoDiffXd> lambda_autodiff = initializeAutoDiff(lambda_double);
  VectorX<AutoDiffXd> xdot_autodiff =
      contact_toolkit_autodiff_->CalcTimeDerivatives(x_autodiff, u_autodiff,
                                                     lambda_autodiff);

  // Verifying the dimensions
  ASSERT_EQ(xdot_double.rows(),
            tree_.get_num_positions() + tree_.get_num_velocities());
  ASSERT_EQ(xdot_autodiff.rows(),
            tree_.get_num_velocities() + tree_.get_num_velocities());

  // Verifying that both templates return the same value
  ASSERT_TRUE(xdot_double.isApprox(DiscardGradient(xdot_autodiff)));
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
