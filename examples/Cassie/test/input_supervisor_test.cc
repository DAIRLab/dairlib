#include "examples/Cassie/systems/input_supervisor.h"

#include <Eigen/Dense>
#include <dairlib/lcmt_controller_failure.hpp>
#include <gtest/gtest.h>

#include "dairlib/lcmt_controller_switch.hpp"
#include "examples/Cassie/cassie_utils.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace systems {

using Eigen::VectorXd;

class InputSupervisorTest : public ::testing::Test {
 protected:
  InputSupervisorTest()
      : plant_(drake::multibody::MultibodyPlant<double>(0.0)) {
    AddCassieMultibody(&plant_, nullptr, true /*floating base*/,
                       "examples/Cassie/urdf/cassie_v2.urdf",
                       true /*spring model*/, false /*loop closure*/);
    plant_.Finalize();
    VectorXd input_limit = 20.0 * VectorXd::Ones(10);
    supervisor_ = std::make_unique<InputSupervisor>(
        plant_, "DEFAULT_CONTROL_CHANNEL", 10.0, 0.01, input_limit, min_consecutive_failures);
    context_ = supervisor_->CreateDefaultContext();
    status_output_ = std::make_unique<dairlib::lcmt_input_supervisor_status>();
    cassie_out_ = std::make_unique<dairlib::lcmt_cassie_out>();
    motor_output_ =
        std::make_unique<TimestampedVector<double>>(plant_.num_actuators());
    command_input_ =
        std::make_unique<TimestampedVector<double>>(plant_.num_actuators());
    state_input_ = std::make_unique<OutputVector<double>>(
        plant_.num_positions(), plant_.num_velocities(),
        plant_.num_actuators());

    state_input_port_ = supervisor_->get_input_port_state().get_index();
  }

  int state_input_port_;
  drake::multibody::MultibodyPlant<double> plant_;
  const int min_consecutive_failures = 5;
  std::unique_ptr<InputSupervisor> supervisor_;
  std::unique_ptr<dairlib::lcmt_input_supervisor_status> status_output_;
  std::unique_ptr<dairlib::lcmt_cassie_out> cassie_out_;
  std::unique_ptr<TimestampedVector<double>> motor_output_;
  std::unique_ptr<TimestampedVector<double>> command_input_;
  std::unique_ptr<OutputVector<double>> state_input_;
  std::unique_ptr<drake::systems::Context<double>> context_;
};

TEST_F(InputSupervisorTest, BlendEffortsTest) {
  VectorXd prev_input = VectorXd::Zero(plant_.num_actuators());
  VectorXd desired_input = 10 * VectorXd::Ones(plant_.num_actuators());
  double blend_start_time = 0.5;
  double timestamp = 1.0;
  double blend_duration = 1.0;
  std::unique_ptr<drake::Value<lcmt_controller_switch>> switch_msg =
      std::make_unique<drake::Value<lcmt_controller_switch>>();
  std::unique_ptr<drake::Value<lcmt_controller_failure>> controller_failure =
      std::make_unique<drake::Value<lcmt_controller_failure>>();
  cassie_out_->pelvis.radio.channel[15] = 0;
  switch_msg->get_mutable_value().blend_duration = blend_duration;
  switch_msg->get_mutable_value().utime = blend_start_time * 1e6;
  command_input_->get_mutable_value() = desired_input;
  command_input_->set_timestamp(timestamp);
  supervisor_->get_input_port_state().FixValue(context_.get(), *state_input_);
  supervisor_->get_input_port_cassie().FixValue(context_.get(), *cassie_out_);
  supervisor_->get_input_port_safety_controller().FixValue(context_.get(),
                                                           *command_input_);
  supervisor_->get_input_port_controller_switch().FixValue(
      context_.get(), (drake::AbstractValue&)*switch_msg);
  supervisor_->get_input_port_controller_error().FixValue(
      context_.get(), (drake::AbstractValue&)*controller_failure);
  supervisor_->get_input_port_command().FixValue(context_.get(),
                                                 *command_input_);
  supervisor_->UpdateErrorFlag(*context_,
                               &context_->get_mutable_discrete_state());
  supervisor_->SetMotorTorques(*context_, motor_output_.get());

  VectorXd output_from_supervisor = motor_output_->get_value();
  double alpha = (timestamp - blend_start_time) / blend_duration;

  EXPECT_EQ(output_from_supervisor, alpha * desired_input);
}

}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
