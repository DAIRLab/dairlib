// HoldEEPlanAbovePressedObject (reposition.h) against the cone demo's
// controller cone, built the way the controller builds its LCS scene.
//
// The printer's z axis is stiff and the finger gives only sideways, so a plan
// that keeps descending into the cone's top squeezes the braced cone (issue
// #13, 2026-10-02 compliant-sim batch).  These pin down the latch: it engages
// only where the plan's start presses into an up-facing face, then holds every
// knot at or above the highest start since engaging, until the start is clear.

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/reposition.h"
#include "examples/sampling_c3/sampling_c3_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/query_object.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {
namespace systems {
namespace {

using drake::geometry::GeometrySet;
using drake::geometry::QueryObject;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;

constexpr double kEERadius = 0.010;
constexpr double kMinNormalZ = 0.5;
constexpr double kReleaseGap = 0.005;
// cone.obj: a hexagonal pyramid, base at the body origin, apex on body +x.
constexpr double kApex = 0.0494062;
// The lateral face whose body normal is (0.406737, 0, 0.913545), and its
// offset from the body origin.
const Vector3d kFaceNormal(0.406737, 0.0, 0.913545);
constexpr double kFaceOffset = 0.0200929;

// The controller's LCS scene, with the cone placed at a chosen pose, and the
// object geometry the controller queries (sampling_based_c3_controller.cc).
class ConeScene {
 public:
  explicit ConeScene(const RigidTransformd& X_WC) : X_WC_(X_WC) {
    const auto params = drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
        "examples/sampling_c3/three_d_printer/cone/parameters/"
        "sampling_c3_controller_params.yaml");
    auto pair = AddMultibodyPlantSceneGraph(&builder_, 0.0);
    plant_ = &pair.plant;
    AddLCSModelsTo3DPrinterPlant(plant_, &pair.scene_graph,
                                 params.object_models);
    plant_->Finalize();
    diagram_ = builder_.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
    context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());
    const auto& cone = plant_->GetBodyByName(params.base_names.at(0));
    plant_->SetFreeBodyPose(context_, cone, X_WC);
    object_ = GeometrySet(plant_->GetCollisionGeometriesForBody(cone)[0]);
  }

  const QueryObject<double>& query_object() const {
    return plant_->get_geometry_query_input_port().Eval<QueryObject<double>>(
        *context_);
  }

  // A world point from a point in the cone's body frame.
  Vector3d World(const Vector3d& p_C) const { return X_WC_ * p_C; }

  // The EE centre that sits @p depth into the lateral face above, at body
  // point @p on_face.
  Vector3d IntoFace(const Vector3d& on_face, double depth) const {
    return World(on_face + (kEERadius - depth) * kFaceNormal);
  }

  EEPressLatchStep Latch(EEPressLatchState* state, MatrixXd* knots,
                         double min_normal_z = kMinNormalZ) const {
    return HoldEEPlanAbovePressedObject(query_object(), object_, kEERadius,
                                        min_normal_z, kReleaseGap, state,
                                        knots);
  }

 private:
  RigidTransformd X_WC_;
  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{};
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagram_context_;
  drake::systems::Context<double>* context_{};
  GeometrySet object_;
};

// Upright at the goal-0 line-up spot: body +x along world +z.
RigidTransformd Upright() {
  return RigidTransformd(
      RotationMatrixd(Quaterniond(M_SQRT1_2, 0.0, -M_SQRT1_2, 0.0)),
      Vector3d(0.28, 0.07, 0.0));
}

// Lying along world +x, the face above with its normal mostly up (z 0.91).
RigidTransformd Lying() {
  return RigidTransformd(RotationMatrixd(), Vector3d(0.28, 0.07, 0.022));
}

// A body point on the lateral face above, at body x = @p x.
Vector3d OnFace(double x) {
  return Vector3d(x, 0.0, (kFaceOffset - kFaceNormal.x() * x) /
                              kFaceNormal.z());
}

// A plan from @p start that descends @p step per knot.
MatrixXd Descending(const Vector3d& start, double step, int num_knots = 5) {
  MatrixXd knots(3, num_knots);
  for (int i = 0; i < num_knots; ++i) {
    knots.col(i) = start - i * Vector3d(0.002, 0.0, step);
  }
  return knots;
}

TEST(PressLatchTest, EngagesOnTheApexAndHoldsTheDescent) {
  const ConeScene scene(Upright());
  const Vector3d start = scene.World(Vector3d(kApex + kEERadius - 0.002, 0, 0));
  MatrixXd knots = Descending(start, 0.001);
  const MatrixXd before = knots;
  EEPressLatchState state;
  const EEPressLatchStep step = scene.Latch(&state, &knots);
  EXPECT_TRUE(step.engaged);
  EXPECT_NEAR(step.gap, -0.002, 1e-4);
  EXPECT_TRUE(state.engaged);
  EXPECT_DOUBLE_EQ(state.floor_z, start.z());
  EXPECT_EQ(step.knots_raised, knots.cols() - 1);
  EXPECT_NEAR(step.max_lift, 0.004, 1e-9);
  for (int i = 0; i < knots.cols(); ++i) {
    EXPECT_DOUBLE_EQ(knots(2, i), start.z());
    // Only z moves: a push sideways is left to the plan.
    EXPECT_EQ(knots.col(i).head<2>(), before.col(i).head<2>());
  }
}

TEST(PressLatchTest, LeavesASidePushAlone) {
  // Into the upright cone's lateral face, whose normal is 0.41 up.
  const ConeScene scene(Upright());
  const Vector3d start = scene.IntoFace(OnFace(0.025), 0.002);
  MatrixXd knots = Descending(start, 0.001);
  const MatrixXd before = knots;
  EEPressLatchState state;
  const EEPressLatchStep step = scene.Latch(&state, &knots);
  EXPECT_NEAR(step.gap, -0.002, 1e-4);
  EXPECT_FALSE(step.engaged);
  EXPECT_FALSE(state.engaged);
  EXPECT_EQ(knots, before);
}

TEST(PressLatchTest, EngagesOnALyingConeOnlyAboveTheNormalThreshold) {
  const ConeScene scene(Lying());
  const Vector3d start = scene.IntoFace(OnFace(0.01), 0.002);
  {
    MatrixXd knots = Descending(start, 0.001);
    EEPressLatchState state;
    EXPECT_TRUE(scene.Latch(&state, &knots).engaged);
  }
  {
    MatrixXd knots = Descending(start, 0.001);
    const MatrixXd before = knots;
    EEPressLatchState state;
    EXPECT_FALSE(scene.Latch(&state, &knots, 0.95).engaged);
    EXPECT_EQ(knots, before);
  }
}

TEST(PressLatchTest, DoesNotEngageOutsideTheObject) {
  // Just above the lying cone's top face, descending onto it.
  const ConeScene scene(Lying());
  MatrixXd knots = Descending(scene.IntoFace(OnFace(0.01), -0.001), 0.001);
  const MatrixXd before = knots;
  EEPressLatchState state;
  const EEPressLatchStep step = scene.Latch(&state, &knots);
  EXPECT_NEAR(step.gap, 0.001, 1e-4);
  EXPECT_FALSE(state.engaged);
  EXPECT_EQ(knots, before);
}

TEST(PressLatchTest, FloorRisesWithTheStartAndReleasesOnlyWhenClear) {
  const ConeScene scene(Lying());
  EEPressLatchState state;
  const Vector3d pressed = scene.IntoFace(OnFace(0.01), 0.002);
  MatrixXd knots = Descending(pressed, 0.001);
  ASSERT_TRUE(scene.Latch(&state, &knots).engaged);

  // The start lifts to 3 mm clear: still latched, and the floor follows it.
  const Vector3d lifted = scene.IntoFace(OnFace(0.01), -0.003);
  knots = Descending(lifted, 0.001);
  EEPressLatchStep step = scene.Latch(&state, &knots);
  EXPECT_FALSE(step.released);
  EXPECT_TRUE(state.engaged);
  EXPECT_DOUBLE_EQ(state.floor_z, lifted.z());
  EXPECT_DOUBLE_EQ(knots.row(2).minCoeff(), lifted.z());

  // Back down into the face: held at the higher floor, not re-anchored lower.
  knots = Descending(pressed, 0.001);
  step = scene.Latch(&state, &knots);
  EXPECT_FALSE(step.engaged);
  EXPECT_DOUBLE_EQ(knots.row(2).minCoeff(), lifted.z());

  // 6 mm clear: released, and the plan is left alone.
  const Vector3d clear = scene.IntoFace(OnFace(0.01), -0.006);
  knots = Descending(clear, 0.001);
  const MatrixXd before = knots;
  step = scene.Latch(&state, &knots);
  EXPECT_TRUE(step.released);
  EXPECT_FALSE(state.engaged);
  EXPECT_EQ(knots, before);
}

TEST(PressLatchTest, NoReadingKeepsTheLatchAsItIs) {
  const ConeScene scene(Lying());
  const GeometrySet nothing;
  EEPressLatchState state;
  MatrixXd knots = Descending(scene.IntoFace(OnFace(0.01), 0.002), 0.001);
  EEPressLatchStep step = HoldEEPlanAbovePressedObject(
      scene.query_object(), nothing, kEERadius, kMinNormalZ, kReleaseGap,
      &state, &knots);
  EXPECT_FALSE(step.engaged);
  EXPECT_TRUE(std::isnan(step.gap));

  // Engaged, a missing reading does not release it.
  state.engaged = true;
  state.floor_z = 0.06;
  step = HoldEEPlanAbovePressedObject(scene.query_object(), nothing, kEERadius,
                                      kMinNormalZ, kReleaseGap, &state,
                                      &knots);
  EXPECT_FALSE(step.released);
  EXPECT_TRUE(state.engaged);
  EXPECT_DOUBLE_EQ(knots.row(2).minCoeff(), 0.06);
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
