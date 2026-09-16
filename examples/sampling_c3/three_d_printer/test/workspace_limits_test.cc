// Covers the workspace limit (WSL) enforcement added to the 3D printer OSC.
//
// The motivating failure:  teleop drove the end effector into the build plate
// while the printer driver was running.  The driver does clamp, but in printer
// joint (carriage) coordinates and with no knowledge of the end effector
// geometry, so it was never a backstop for the dairlib-side limits.  These
// tests pin the three pieces that make it one:
//   1. GetWorkspaceBox reads the box, and refuses a config it would silently
//      misinterpret.
//   2. ThreeDPrinterCommandSender clamps the published command, including the
//      velocity the driver extrapolates its lookahead waypoints from.
//   3. The teleop target itself is bounded, so it can't wind up past the
//      boundary while the stick is held into it.

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <dairlib/lcmt_radio_out.hpp>
#include <gtest/gtest.h>

#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "systems/controllers/osc/end_effector_position.h"
#include "systems/robot_lcm_systems.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/yaml/yaml_io.h"

namespace dairlib {
namespace {

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr char kConeControllerParams[] =
    "examples/sampling_c3/three_d_printer/cone/parameters/"
    "sampling_c3_controller_params.yaml";

std::vector<VectorXd> MakeAxisAlignedLimits(const Vector3d& lower,
                                            const Vector3d& upper) {
  std::vector<VectorXd> limits;
  for (int i = 0; i < 3; ++i) {
    VectorXd row(5);
    row << Vector3d::Unit(i), lower(i), upper(i);
    limits.push_back(row);
  }
  return limits;
}

// ---------------------------------------------------------------------------
// GetWorkspaceBox
// ---------------------------------------------------------------------------

// The shipped cone limits, which are what the hardware actually runs against.
TEST(WorkspaceLimitsTest, ConeYamlGivesTheExpectedBox) {
  const SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          kConeControllerParams);
  const auto [lower, upper] =
      GetWorkspaceBox(controller_params.sampling_c3_options.workspace_limits);
  EXPECT_TRUE(lower.isApprox(Vector3d(0.0, 0.0, 0.015)));
  EXPECT_TRUE(upper.isApprox(Vector3d(0.35, 0.35, 0.248)));
}

// The margin is what SamplingC3Controller::ClampPlanToWorkspaceLimits applies
// to its published plan.  The OSC deliberately uses the raw box (margin 0), so
// it is strictly looser and never perturbs a valid C3 trajectory.
TEST(WorkspaceLimitsTest, TheOscBoxIsStrictlyLooserThanTheC3PlansBox) {
  const SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          kConeControllerParams);
  const auto& options = controller_params.sampling_c3_options;
  ASSERT_GT(options.workspace_margins, 0.0);
  const auto [osc_lower, osc_upper] = GetWorkspaceBox(options.workspace_limits);
  const auto [c3_lower, c3_upper] =
      GetWorkspaceBox(options.workspace_limits, options.workspace_margins);
  EXPECT_TRUE((osc_lower.array() < c3_lower.array()).all());
  EXPECT_TRUE((osc_upper.array() > c3_upper.array()).all());
}

// workspace_limits is stored as a general half-space, and the C3 solve does
// honor that.  Every clamp that reads it does not.  Fail loudly rather than
// silently clamping against a bound that means something else.
TEST(WorkspaceLimitsTest, NonAxisAlignedLimitsAreRejected) {
  auto limits = MakeAxisAlignedLimits(Vector3d::Zero(), Vector3d::Ones());
  limits[0] << 1, 1, 0, 0.0, 1.0;  // a diagonal half-space, not the x axis
  EXPECT_THROW(GetWorkspaceBox(limits), std::exception);
}

TEST(WorkspaceLimitsTest, WrongNumberOfLimitRowsIsRejected) {
  auto limits = MakeAxisAlignedLimits(Vector3d::Zero(), Vector3d::Ones());
  limits.pop_back();
  EXPECT_THROW(GetWorkspaceBox(limits), std::exception);
}

// ---------------------------------------------------------------------------
// ThreeDPrinterCommandSender -- the hard backstop
// ---------------------------------------------------------------------------

class CommandSenderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    Add3DPrinterToPlant(&plant_);
    plant_.Finalize();
    sender_ = std::make_unique<systems::ThreeDPrinterCommandSender>(plant_);
    sender_->SetPositionLimits(Vector3d(0.0, 0.0, 0.015),
                               Vector3d(0.35, 0.35, 0.248));
    context_ = sender_->CreateDefaultContext();
  }

  // Feeds a constant-velocity trajectory through the sender and returns the
  // published message.
  dairlib::lcmt_robot_output Publish(const Vector3d& position,
                                     const Vector3d& velocity) {
    const PiecewisePolynomial<double> traj =
        PiecewisePolynomial<double>::FirstOrderHold(
            std::vector<double>{0.0, 1.0},
            std::vector<Eigen::MatrixXd>{position, position + velocity});
    const Trajectory<double>& traj_ref = traj;
    sender_->get_input_port(0).FixValue(context_.get(), traj_ref);
    context_->SetTime(0.0);
    return sender_->get_output_port(0).Eval<dairlib::lcmt_robot_output>(
        *context_);
  }

  drake::multibody::MultibodyPlant<double> plant_{0.0};
  std::unique_ptr<systems::ThreeDPrinterCommandSender> sender_;
  std::unique_ptr<drake::systems::Context<double>> context_;
};

TEST_F(CommandSenderTest, AnInBoundsCommandPassesThroughUntouched) {
  const auto msg = Publish(Vector3d(0.1, 0.2, 0.1), Vector3d(0.01, 0.0, -0.01));
  EXPECT_NEAR(msg.position[0], 0.1, 1e-12);
  EXPECT_NEAR(msg.position[1], 0.2, 1e-12);
  EXPECT_NEAR(msg.position[2], 0.1, 1e-12);
  EXPECT_NEAR(msg.velocity[2], -0.01, 1e-12);
}

// The build-plate collision, in miniature.
TEST_F(CommandSenderTest, DrivingBelowTheFloorIsClampedToTheFloor) {
  const auto msg = Publish(Vector3d(0.1, 0.2, -0.05), Vector3d(0.0, 0.0, -0.01));
  EXPECT_NEAR(msg.position[2], 0.015, 1e-12);
}

// The driver extrapolates lookahead sub-waypoints from the commanded velocity,
// so leaving an outward velocity at the wall would carry the head past the
// position clamp.
TEST_F(CommandSenderTest, OutwardVelocityIsZeroedAtTheBoundary) {
  const auto below = Publish(Vector3d(0.1, 0.2, -0.05), Vector3d(0, 0, -0.01));
  EXPECT_NEAR(below.velocity[2], 0.0, 1e-12);

  const auto above = Publish(Vector3d(0.1, 0.2, 0.5), Vector3d(0, 0, 0.01));
  EXPECT_NEAR(above.position[2], 0.248, 1e-12);
  EXPECT_NEAR(above.velocity[2], 0.0, 1e-12);
}

// Clamping the position must not also kill the escape.
TEST_F(CommandSenderTest, InwardVelocityIsPreservedAtTheBoundary) {
  const auto msg = Publish(Vector3d(0.1, 0.2, -0.05), Vector3d(0, 0, 0.01));
  EXPECT_NEAR(msg.position[2], 0.015, 1e-12);
  EXPECT_NEAR(msg.velocity[2], 0.01, 1e-12);
}

TEST_F(CommandSenderTest, NaNIsClampedRatherThanPassedThrough) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto msg = Publish(Vector3d(0.1, 0.2, nan), Vector3d::Zero());
  EXPECT_FALSE(std::isnan(msg.position[2]));
  EXPECT_NEAR(msg.position[2], 0.015, 1e-12);
}

// Every other caller of this system must be unaffected.
TEST_F(CommandSenderTest, LimitsAreUnsetByDefault) {
  systems::ThreeDPrinterCommandSender unbounded(plant_);
  auto unbounded_context = unbounded.CreateDefaultContext();
  const PiecewisePolynomial<double> traj(Vector3d(-100.0, 0.0, 100.0));
  const Trajectory<double>& traj_ref = traj;
  unbounded.get_input_port(0).FixValue(unbounded_context.get(), traj_ref);
  const auto msg =
      unbounded.get_output_port(0).Eval<dairlib::lcmt_robot_output>(
          *unbounded_context);
  EXPECT_NEAR(msg.position[0], -100.0, 1e-12);
  EXPECT_NEAR(msg.position[2], 100.0, 1e-12);
}

// ---------------------------------------------------------------------------
// EndEffectorPositionTrajectoryGenerator -- anti-windup on the teleop target
// ---------------------------------------------------------------------------

class TeleopTargetTest : public ::testing::Test {
 protected:
  static constexpr double kZScale = 0.001;  // matches osc_params.yaml

  void SetUp() override {
    Add3DPrinterToPlant(&plant_);
    plant_.Finalize();
    plant_context_ = plant_.CreateDefaultContext();
    generator_ = std::make_unique<EndEffectorPositionTrajectoryGenerator>(
        plant_, plant_context_.get(), Vector3d(0.1, 0.1, 0.1),
        /*teleop_neutral_pose=*/true, k3dEndEffectorTipName);
    generator_->SetRemoteControlParameters(Vector3d(0.1, 0.1, 0.1), kZScale,
                                           kZScale, kZScale);
    generator_->SetWorkspaceLimits(Vector3d(0.0, 0.0, 0.015),
                                   Vector3d(0.35, 0.35, 0.248));
    context_ = generator_->CreateDefaultContext();

    // Teleop on, sticks centered; the state input is only read on the
    // transition into teleop.
    radio_.channel[14] = 1;
    generator_->get_input_port_radio().FixValue(context_.get(), radio_);
    const PiecewisePolynomial<double> empty_plan(Vector3d::Zero());
    const Trajectory<double>& plan_ref = empty_plan;
    generator_->get_input_port_trajectory().FixValue(context_.get(), plan_ref);
    generator_->get_input_port_state().FixValue(
        context_.get(),
        systems::OutputVector<double>(VectorXd::Zero(plant_.num_positions()),
                                      VectorXd::Zero(plant_.num_velocities()),
                                      VectorXd::Zero(plant_.num_actuators())));
  }

  // Runs one controller tick with the z stick at `z_command` and returns the
  // resulting target position.
  Vector3d Step(double z_command) {
    radio_.channel[2] = z_command;
    generator_->get_input_port_radio().FixValue(context_.get(), radio_);
    const auto& traj =
        generator_->get_output_port(0).Eval<Trajectory<double>>(*context_);
    return traj.value(0.0);
  }

  drake::multibody::MultibodyPlant<double> plant_{0.0};
  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  std::unique_ptr<EndEffectorPositionTrajectoryGenerator> generator_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  dairlib::lcmt_radio_out radio_{};
};

// Without this, the target integrates the stick without limit.  Holding the
// stick into the floor for a few seconds would wind it metres below the plate,
// and the operator would have to drive back for just as long before the head
// moved at all -- even with the downstream clamp in place.
TEST_F(TeleopTargetTest, HoldingTheStickIntoTheFloorDoesNotWindThePastTarget) {
  Vector3d target = Vector3d::Zero();
  for (int i = 0; i < 1000; ++i) {  // ~10 s of holding the stick down
    target = Step(-1.0);
  }
  EXPECT_NEAR(target(2), 0.015, 1e-12);

  // One tick of the opposite command must move the target immediately.
  const Vector3d escaped = Step(1.0);
  EXPECT_NEAR(escaped(2), 0.015 + kZScale, 1e-12);
}

TEST_F(TeleopTargetTest, TheUpperBoundHoldsToo) {
  Vector3d target = Vector3d::Zero();
  for (int i = 0; i < 1000; ++i) {
    target = Step(1.0);
  }
  EXPECT_NEAR(target(2), 0.248, 1e-12);
  EXPECT_NEAR(Step(-1.0)(2), 0.248 - kZScale, 1e-12);
}

// Franka shares this system and never calls SetWorkspaceLimits.
TEST_F(TeleopTargetTest, LimitsAreUnsetByDefault) {
  EndEffectorPositionTrajectoryGenerator unbounded(
      plant_, plant_context_.get(), Vector3d(0.1, 0.1, 0.1),
      /*teleop_neutral_pose=*/true, k3dEndEffectorTipName);
  unbounded.SetRemoteControlParameters(Vector3d(0.1, 0.1, 0.1), kZScale,
                                       kZScale, kZScale);
  auto unbounded_context = unbounded.CreateDefaultContext();
  radio_.channel[2] = -1.0;
  unbounded.get_input_port_radio().FixValue(unbounded_context.get(), radio_);
  const PiecewisePolynomial<double> empty_plan(Vector3d::Zero());
  const Trajectory<double>& plan_ref = empty_plan;
  unbounded.get_input_port_trajectory().FixValue(unbounded_context.get(),
                                                 plan_ref);
  unbounded.get_input_port_state().FixValue(
      unbounded_context.get(),
      systems::OutputVector<double>(VectorXd::Zero(plant_.num_positions()),
                                    VectorXd::Zero(plant_.num_velocities()),
                                    VectorXd::Zero(plant_.num_actuators())));
  Vector3d target = Vector3d::Zero();
  for (int i = 0; i < 1000; ++i) {
    target = unbounded.get_output_port(0)
                 .Eval<Trajectory<double>>(*unbounded_context)
                 .value(0.0);
  }
  EXPECT_LT(target(2), 0.0);  // walked straight through the floor
}

}  // namespace
}  // namespace dairlib
