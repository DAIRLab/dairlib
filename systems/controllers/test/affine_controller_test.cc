#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include "systems/controllers/affine_controller.h"

namespace dairlib {
namespace systems {
namespace {

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::make_unique;
using std::unique_ptr;
using drake::systems::System;
using drake::systems::Context;
using drake::systems::SystemOutput;
using drake::systems::BasicVector;

class AffineControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int np = 2;
    int nv = 2;
    ns_ = np + nv;
    ne_ = 2;
    positions_vec_.resize(np);
    velocities_vec_.resize(nv);
    efforts_vec_.resize(ne_);
    K_vec_.resize(ns_ * ne_);
    E_vec_.resize(ne_);
    x_des_vec_.resize(ns_);
    params_vec_.resize(ns_ * ne_ + ne_ + ns_ + 1);
    expected_output_vec_.resize(ne_);
    K_.resize(ne_, ns_);

    timestamp_ = 0.0;

    positions_vec_ << 1.0, -2.5;
    velocities_vec_ << 0.5, -1.5;
    efforts_vec_ << 0.0, 0.0;
    K_vec_ << 1.0, 2.0, 3.0, 4.0, 2.0, 1.0, 2.0, 1.0;
    E_vec_ << -0.5, 3.5;
    x_des_vec_ << 0.0, 1.0, -1.5, 4.0;
    params_vec_ << K_vec_, E_vec_, x_des_vec_, 0.0;
    expected_output_vec_ << 16.0, 19.0;
    K_ << 1.0, 3.0, 2.0, 2.0, 2.0, 4.0, 1.0, 1.0;

    input_port_info_val_ = make_unique<OutputVector<double>>(
        positions_vec_, velocities_vec_, efforts_vec_);
    input_port_info_val_->set_timestamp(timestamp_);
    input_port_params_val_ = make_unique<AffineParams>(ns_, ne_);
    affine_controller_ = make_unique<AffineController>(np, nv, ne_);

    input_port_params_val_->SetFromVector(params_vec_);

    context_ = affine_controller_->CreateDefaultContext();
    output_ = affine_controller_->AllocateOutput();
  }

  int ns_;
  int ne_;
  double timestamp_;
  VectorXd positions_vec_;
  VectorXd velocities_vec_;
  VectorXd efforts_vec_;
  VectorXd K_vec_;
  VectorXd E_vec_;
  VectorXd x_des_vec_;
  VectorXd params_vec_;
  VectorXd expected_output_vec_;
  MatrixXd K_;

  unique_ptr<OutputVector<double>> input_port_info_val_;
  unique_ptr<AffineParams> input_port_params_val_;

  unique_ptr<AffineController> affine_controller_;
  unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

TEST_F(AffineControllerTest, TestAffineParamsAccessors) {
  AffineParams affine_params_test(2, 3);
  MatrixXd K_test(3, 2);
  VectorXd E_test(3);
  VectorXd x_des_test(2);

  K_test << 0.1, 2.3, 3.4, -1.5, 6.7, 0.98;
  E_test << 1, 2, 3.3;
  x_des_test << 2.7, 5.5;

  affine_params_test.set_K(K_test);
  affine_params_test.set_E(E_test);
  affine_params_test.set_desired_state(x_des_test);

  ASSERT_EQ(K_test, affine_params_test.get_K());
  ASSERT_EQ(E_test, affine_params_test.get_E());
  ASSERT_EQ(x_des_test, affine_params_test.get_desired_state());
}

// Tests number of input and output ports.
TEST_F(AffineControllerTest, TestNumberOfPortsAndControllerOutput) {
  /// Checks that the number of input ports in the system and in the context
  // are consistent.
  ASSERT_EQ(context_->num_input_ports(), 2);
  ASSERT_EQ(affine_controller_->num_input_ports(), 2);

  // Hook input of the expected size.
  context_->FixInputPort(affine_controller_->get_input_port_info_index(),
                         *input_port_info_val_);
  context_->FixInputPort(affine_controller_->get_input_port_params_index(),
                         *input_port_params_val_);

  affine_controller_->CalcOutput(*context_, output_.get());

  // Checks that the number of output ports in the system and in the
  // output are consistent.
  ASSERT_EQ(1, output_->num_ports());
  ASSERT_EQ(1, affine_controller_->num_output_ports());

  Eigen::VectorXd output, output_data;
  const BasicVector<double>* output_port_vec = output_->get_vector_data(0);

  // Making sure that the output vector does not point to null
  ASSERT_NE(nullptr, output_port_vec);

  output = output_port_vec->get_value();
  output_data = output.head(output.size() - 1);

  ASSERT_EQ(expected_output_vec_, output_data);
}

TEST_F(AffineControllerTest, TestAffineParams) {
  ASSERT_EQ(K_, input_port_params_val_->get_K());
  ASSERT_EQ(E_vec_, input_port_params_val_->get_E());
  ASSERT_EQ(x_des_vec_, input_port_params_val_->get_desired_state());
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
