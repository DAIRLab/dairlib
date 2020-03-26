#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include "examples/Cassie/cassie_utils.h"
#include "multibody/contact_toolkit.h"
#include "multibody/multibody_utils.h"
#include "drake/multibody/tree/revolute_joint.h"

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
using dairlib::multibody::ContactToolkit;
using dairlib::multibody::ContactInfo;
using drake::systems::DiagramBuilder;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::systems::Context;
using drake::multibody::ContactResults;

typedef ::testing::Types<double> ScalarTypes;

class ContactToolkitTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setting the random seed
    std::srand((unsigned int)time(0));

    DiagramBuilder<double> builder;

    SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
    scene_graph.set_name("scene_graph");

    plant_ = builder.AddSystem<MultibodyPlant>(1.0);
    // Adding the ground
    multibody::addFlatTerrain(plant_, &scene_graph, .8, .8);
    addCassieMultibody(plant_, &scene_graph, true);
    plant_->Finalize();

    const int num_positions = plant_->num_positions();
    const int num_velocities = plant_->num_velocities();
    const int num_states = num_positions + num_velocities;

    // Setting the initial Cassie joint angles
    auto diagram = builder.Build();
    std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
    Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(*plant_, diagram_context.get());

    plant_->GetJointByName<RevoluteJoint>("hip_roll_left").
        set_angle(&plant_context, .1);
    plant_->GetJointByName<RevoluteJoint>("hip_yaw_left").
        set_angle(&plant_context, .01);
    plant_->GetJointByName<RevoluteJoint>("hip_pitch_left").
        set_angle(&plant_context, -.169);
    plant_->GetJointByName<RevoluteJoint>("knee_left").
        set_angle(&plant_context, -.744);
    plant_->GetJointByName<RevoluteJoint>("ankle_joint_left").
        set_angle(&plant_context, .81);
    plant_->GetJointByName<RevoluteJoint>("toe_left").
        set_angle(&plant_context, -30.0 * M_PI / 180.0);

    plant_->GetJointByName<RevoluteJoint>("hip_roll_right").
        set_angle(&plant_context, -.01);
    plant_->GetJointByName<RevoluteJoint>("hip_yaw_right").
        set_angle(&plant_context, .01);
    plant_->GetJointByName<RevoluteJoint>("hip_pitch_right").
        set_angle(&plant_context, .269);
    plant_->GetJointByName<RevoluteJoint>("knee_right").
        set_angle(&plant_context, -.744);
    plant_->GetJointByName<RevoluteJoint>("ankle_joint_right").
        set_angle(&plant_context, .81);
    plant_->GetJointByName<RevoluteJoint>("toe_right").
        set_angle(&plant_context, -60.0 * M_PI / 180.0);

    Eigen::Isometry3d transform;
    transform.linear() = Eigen::Matrix3d::Identity();;
    transform.translation() = Eigen::Vector3d(0, 0, 1.2);
    plant_->SetFreeBodyPose(&plant_context, plant_->GetBodyByName("pelvis"),
        transform);

    // Colliison detect
    VectorXd phi_total;
    Matrix3Xd normal_total, xA_total, xB_total;
    vector<int> idxA_total, idxB_total;

    // plant_->CalcContactResults(plant_context,
    // const std::vector<PenetrationAsPointPair<T>>& point_pairs,
    // const std::vector<Matrix3<T>>& R_WC_set,
    // ContactResults<T>* contact_results)

    auto contact_results_value =
      plant_->get_contact_results_output_port().Allocate();
    const ContactResults<double>& contact_results =
      contact_results_value->GetValueOrThrow<ContactResults<double>>();
    // Compute the poses for each geometry in the model.
    plant_->get_contact_results_output_port().Calc(
      plant_context, contact_results_value.get());

    // // Extracting information into the four contacts.
    Matrix3Xd normal(3, 4), xA(3, 4), xB(3, 4);
    auto world_ind = plant_->world_body().index();
    auto toe_left_ind = plant_->GetBodyByName("toe_left").index();
    auto toe_right_ind = plant_->GetBodyByName("toe_right").index();
    std::vector<const drake::multibody::Frame<double>*> frames;
    int k = 0;
    for (int i = 0; i < contact_results.num_contacts(); i++) {
      auto info = contact_results.contact_info(i);
      auto ind_a = info.bodyA_index();
      auto ind_b = info.bodyB_index();
      if ((ind_a == world_ind && ind_b == toe_left_ind) ||
          (ind_a == world_ind && ind_b == toe_right_ind)) {
        xA.col(k) = info.point_pair().p_WCb;
        xB.col(k) = info.point_pair().p_WCa;
        frames.push_back(&plant_->get_body(ind_b).body_frame());
        ++k;
      } else if ((ind_a == toe_left_ind && ind_b == world_ind) ||
                 (ind_a == toe_right_ind && ind_b == world_ind)) {
        xA.col(k) = info.point_pair().p_WCa;
        xB.col(k) = info.point_pair().p_WCb;
        frames.push_back(&plant_->get_body(ind_a).body_frame());
        ++k;
      }
    }

    std::cout << contact_results.num_contacts() << std::endl;

    // Creating the contact info
    contact_info_ = {xA, xB, frames};

    // ContactToolkit objects for both templates
    contact_toolkit_ =
        make_unique<ContactToolkit<double>>(*plant_, contact_info_);
  }

  MultibodyPlant<double>* plant_;
  unique_ptr<ContactToolkit<double>> contact_toolkit_;
  ContactInfo<double> contact_info_;
  VectorXd x0_;
};

TYPED_TEST_CASE(ContactToolkitTest, ScalarTypes);


// Tests the contact toolkit constructor initializations, getters and setters
TEST_F(ContactToolkitTest, InitializationTest) {
  // ContactInfo getter
  ContactInfo<double> tmp_info = contact_toolkit_->get_contact_info();
  ASSERT_TRUE(tmp_info.xA.isApprox(contact_info_.xA));
  ASSERT_TRUE(tmp_info.xB.isApprox(contact_info_.xB));
  ASSERT_TRUE(tmp_info.frameA == contact_info_.frameA);

  // num contacts getter
  ASSERT_EQ(contact_toolkit_->get_num_contacts(), 4);

  // Verifying the contact info setter
  Matrix3Xd xA(3, 2), xB(3, 2);
  vector<int> idxA = {0, 0};
  xA << 0.1, 0.2, 0.3, 0.2, -0.3, 2.3;
  xB << 0.1, -0.3, 0.5, 1.7, 5.5, 0.9;
  // tmp_info = {xA, xB, idxA};

  // contact_toolkit_->set_contact_info(tmp_info);
  // ASSERT_TRUE(xA.isApprox(contact_toolkit_->get_contact_info().xA));
  // ASSERT_TRUE(xB.isApprox(contact_toolkit_->get_contact_info().xB));
  // ASSERT_TRUE(idxA == contact_toolkit_->get_contact_info().idxA);
}

// // Contact Jacobian test
// TEST_F(ContactToolkitTest, ContactJacobianTest) {
//   VectorX<double> x_double = x0_;
//   VectorX<AutoDiffXd> x_autodiff = initializeAutoDiff(x0_);
//   MatrixX<double> jac_double =
//       contact_toolkit_->CalcContactJacobian(x_double);
//   MatrixX<AutoDiffXd> jac_autodiff =
//       contact_toolkit_autodiff_->CalcContactJacobian(x_autodiff);

//   // Checking dimensions of the jacobian
//   // Each contact has three directional jacobian componenets - the normal and
//   // two surface tangents.
//   ASSERT_EQ(jac_double.rows(), contact_toolkit_->get_num_contacts() * 3);
//   ASSERT_EQ(jac_double.cols(), plant_->num_positions());

//   ASSERT_EQ(jac_autodiff.rows(),
//             contact_toolkit_autodiff_->get_num_contacts() * 3);
//   ASSERT_EQ(jac_autodiff.cols(), plant_->num_positions());
// }

// // MVDot test
// TEST_F(ContactToolkitTest, MVDotTest) {
//   VectorX<double> x_double = x0_;
//   VectorX<double> u_double = VectorXd::Random(plant_->num_actuators());
//   VectorX<double> lambda_double =
//       VectorXd::Random(tree_.getNumPositionConstraints() +
//                        contact_toolkit_->get_num_contacts() * 3);
//   VectorX<double> mvdot_double =
//       contact_toolkit_->CalcMVDot(x_double, u_double, lambda_double);

//   VectorX<AutoDiffXd> x_autodiff = initializeAutoDiff(x_double);
//   VectorX<AutoDiffXd> u_autodiff = initializeAutoDiff(u_double);
//   VectorX<AutoDiffXd> lambda_autodiff = initializeAutoDiff(lambda_double);
//   VectorX<AutoDiffXd> mvdot_autodiff = contact_toolkit_autodiff_->CalcMVDot(
//       x_autodiff, u_autodiff, lambda_autodiff);

//   // Verifying the dimensions
//   ASSERT_EQ(mvdot_double.rows(), plant_->num_velocities());
//   ASSERT_EQ(mvdot_autodiff.rows(), plant_->num_velocities());

//   // Verifying that both templates return the same value
//   ASSERT_TRUE(mvdot_double.isApprox(DiscardGradient(mvdot_autodiff)));
// }

// // Time derivatives test
// TEST_F(ContactToolkitTest, TimeDerivativesTest) {
//   VectorX<double> x_double = x0_;
//   VectorX<double> u_double = VectorXd::Random(plant_->num_actuators());
//   VectorX<double> lambda_double =
//       VectorXd::Random(tree_.getNumPositionConstraints() +
//                        contact_toolkit_->get_num_contacts() * 3);
//   VectorX<double> xdot_double = contact_toolkit_->CalcTimeDerivatives(
//       x_double, u_double, lambda_double);

//   VectorX<AutoDiffXd> x_autodiff = initializeAutoDiff(x_double);
//   VectorX<AutoDiffXd> u_autodiff = initializeAutoDiff(u_double);
//   VectorX<AutoDiffXd> lambda_autodiff = initializeAutoDiff(lambda_double);
//   VectorX<AutoDiffXd> xdot_autodiff =
//       contact_toolkit_autodiff_->CalcTimeDerivatives(x_autodiff, u_autodiff,
//                                                      lambda_autodiff);

//   // Verifying the dimensions
//   ASSERT_EQ(xdot_double.rows(),
//             plant_->num_positions() + plant_->num_velocities());
//   ASSERT_EQ(xdot_autodiff.rows(),
//             plant_->num_velocities() + plant_->num_velocities());

//   // Verifying that both templates return the same value
//   ASSERT_TRUE(xdot_double.isApprox(DiscardGradient(xdot_autodiff)));
// }

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
