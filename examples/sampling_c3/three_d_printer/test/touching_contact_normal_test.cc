// Drake reports a NaN contact normal whenever two geometries are exactly
// touching (|phi| < 1e-14) and neither of them is a sphere -- the fallback that
// covers that case, CalcGradientWhenTouching, only supports sphere-vs-shape and
// box-vs-box.  The LCS factory hands that normal straight to
// RotationMatrix::MakeFromOneVector, which throws, so a single degenerate pose
// used to abort the controller process mid-run.
//
// The cone demo is the scene that can reach it: the cone's C3 collision
// geometry is a mesh and each ramp piece is a convex mesh, so the object-ramp
// group carries 14 mesh-vs-convex pairs.  The tests below find a pose where one
// of those pairs lands exactly on contact and check that the controller's
// contact resolution keeps the group full without ever handing the factory the
// NaN.

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "common/update_context.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "systems/controllers/sampling_based_c3_controller.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {
namespace {

using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::geometry::QueryObject;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::vector;
using systems::SamplingC3Controller;

// The pose the 3D printer demo was in when this crash was first seen: the cone
// tipped against the far end of the ramp, resting on its base corners.
constexpr double kObjectY = 0.0732;
constexpr double kObjectZ = 0.0014;
const Vector4d kObjectQuat{0.6211, -0.3108, -0.6581, -0.2934};
const Vector3d kEEPosition{0.2463, 0.0838, 0.0383};

// The demo's LCS scene, built the way the controller binary builds it.
class ConeRampScene {
 public:
  ConeRampScene() {
    controller_params_ =
        drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
            "examples/sampling_c3/three_d_printer/cone/parameters/"
            "sampling_c3_controller_params.yaml");
    auto pair = AddMultibodyPlantSceneGraph(&builder_, 0.0);
    plant_ = &pair.plant;
    AddLCSModelsTo3DPrinterPlant(plant_, &pair.scene_graph,
                                 controller_params_.object_models);
    plant_->Finalize();
    plant_ad_ = drake::systems::System<double>::ToAutoDiffXd(*plant_);
    diagram_ = builder_.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
    context_ = &diagram_->GetMutableSubsystemContext(*plant_,
                                                     diagram_context_.get());
    context_ad_ = plant_ad_->CreateDefaultContext();
    contact_pairs_ =
        BuildConeContactPairs(*plant_, controller_params_.base_names);
    cone_id_ = plant_->GetCollisionGeometriesForBody(
        plant_->GetBodyByName(controller_params_.base_names.at(0)))[0];
  }

  // Places the cone at `object_x` (the other five pose coordinates are fixed)
  // and leaves the plant context there.
  void SetObjectX(double object_x) {
    VectorXd x_lcs =
        VectorXd::Zero(plant_->num_positions() + plant_->num_velocities());
    x_lcs.head(3) = kEEPosition;
    x_lcs.segment(3, 4) = kObjectQuat.normalized();
    x_lcs.segment(7, 3) = Vector3d(object_x, kObjectY, kObjectZ);
    UpdateContext(plant_->num_positions(), plant_->num_velocities(),
                  plant_->num_actuators(), *plant_, context_, *plant_ad_,
                  context_ad_.get(), x_lcs);
  }

  const QueryObject<double>& query_object() const {
    return plant_->get_geometry_query_input_port()
        .Eval<QueryObject<double>>(*context_);
  }

  const MultibodyPlant<double>& plant() const { return *plant_; }
  const drake::systems::Context<double>& context() const { return *context_; }
  const SamplingC3ControllerParams& controller_params() const {
    return controller_params_;
  }
  const vector<vector<SortedPair<GeometryId>>>& contact_pairs() const {
    return contact_pairs_;
  }
  GeometryId cone_id() const { return cone_id_; }

 private:
  SamplingC3ControllerParams controller_params_;
  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{};
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_ad_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagram_context_;
  drake::systems::Context<double>* context_{};
  std::unique_ptr<drake::systems::Context<drake::AutoDiffXd>> context_ad_;
  vector<vector<SortedPair<GeometryId>>> contact_pairs_;
  GeometryId cone_id_;
};

// Bisects the cone's x position until `pair` sits close enough to exactly
// touching that Drake gives up on the normal.  Returns nullopt if the bracket
// does not straddle contact, so a ramp piece the cone never reaches is skipped
// rather than reported as a failure.
std::optional<double> FindTouchingObjectX(ConeRampScene* scene,
                                          const SortedPair<GeometryId>& pair) {
  auto phi = [&](double x) {
    scene->SetObjectX(x);
    return scene->query_object()
        .ComputeSignedDistancePairClosestPoints(pair.first(), pair.second())
        .distance;
  };
  double lo = 0.2324 - 0.02;  // into the ramp
  double hi = 0.2324 + 0.05;  // clear of the ramp
  if (!(phi(lo) < 0) || !(phi(hi) > 0)) return std::nullopt;
  for (int i = 0; i < 200; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (mid == lo || mid == hi) break;
    scene->SetObjectX(mid);
    const auto result =
        scene->query_object().ComputeSignedDistancePairClosestPoints(
            pair.first(), pair.second());
    if (result.nhat_BA_W.array().isNaN().any()) return mid;
    if (result.distance < 0) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return std::nullopt;
}

// The premise: the object-ramp group really does contain pairs Drake cannot
// orient at contact.  If Drake ever grows a mesh-vs-convex touching gradient,
// this is the test that says the guard below has become unnecessary.
TEST(TouchingContactNormalTest, ConeVsRampNormalIsNaNWhenExactlyTouching) {
  ConeRampScene scene;
  int degenerate_pairs = 0;
  for (const auto& pair : scene.contact_pairs().at(3)) {
    if (pair.first() != scene.cone_id() && pair.second() != scene.cone_id()) {
      continue;
    }
    if (FindTouchingObjectX(&scene, pair).has_value()) ++degenerate_pairs;
  }
  EXPECT_GT(degenerate_pairs, 0);
}

// The fix: at such a pose the controller still resolves a full set of contacts,
// and not one of them carries a NaN normal into the LCS factory.
TEST(TouchingContactNormalTest, ResolvedContactsNeverCarryANaNNormal) {
  ConeRampScene scene;
  const auto& options = scene.controller_params().sampling_c3_options;
  int poses_checked = 0;

  for (const auto& pair : scene.contact_pairs().at(3)) {
    if (pair.first() != scene.cone_id() && pair.second() != scene.cone_id()) {
      continue;
    }
    const std::optional<double> touching_x = FindTouchingObjectX(&scene, pair);
    if (!touching_x.has_value()) continue;
    ++poses_checked;
    scene.SetObjectX(touching_x.value());

    for (bool is_pose_tracking : {false, true}) {
      const auto lcs_options = options.GetLCSFactoryOptions(is_pose_tracking);
      SCOPED_TRACE("pose tracking: " + std::to_string(is_pose_tracking));
      vector<SortedPair<GeometryId>> resolved;
      ASSERT_NO_THROW(
          resolved = SamplingC3Controller::GetResolvedContactPairs(
              scene.plant(), scene.context(), scene.contact_pairs(),
              options.resolve_contacts_to,
              options.max_contacts_per_object_geometry,
              options.contact_dedup_witness_radius.value_or(0.0)));
      EXPECT_EQ(static_cast<int>(resolved.size()), lcs_options.num_contacts);
      for (const auto& resolved_pair : resolved) {
        const auto result =
            scene.query_object().ComputeSignedDistancePairClosestPoints(
                resolved_pair.first(), resolved_pair.second());
        EXPECT_FALSE(result.nhat_BA_W.array().isNaN().any());
      }
    }
  }
  EXPECT_GT(poses_checked, 0);
}

}  // namespace
}  // namespace dairlib
