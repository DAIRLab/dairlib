// Unit tests for the per-goal keep-out regions added for the 3D printer cone
// demo: the generalized SampleAvoidsGeometries() distance/clearance check
// (generate_samples.h) and AddKeepOutModelsToPlant()'s URDF-loading /
// world-welding convention (sampling_c3_utils.h).

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "common/find_resource.h"
#include "examples/sampling_c3/generate_samples.h"
#include "examples/sampling_c3/sampling_c3_utils.h"

#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {
namespace systems {
namespace {

using drake::geometry::Box;
using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::GeometrySet;
using drake::geometry::QueryObject;
using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ModelInstanceIndex;

Eigen::VectorXd EEState(const Eigen::Vector3d& ee) {
  // SampleAvoidsGeometries only reads .head(3); size beyond that is irrelevant.
  Eigen::VectorXd x = Eigen::VectorXd::Zero(19);
  x.head(3) = ee;
  return x;
}

// A bare SceneGraph holding a single anchored box, x in [0.15, 0.25],
// y in [0.02, 0.12], z in [0.0, 0.20] (centre (0.20, 0.07, 0.10)).
struct BoxScene {
  std::unique_ptr<SceneGraph<double>> scene_graph;
  std::unique_ptr<drake::systems::Context<double>> context;
  GeometrySet geometries;

  const QueryObject<double>& query() const {
    return scene_graph->get_query_output_port().Eval<QueryObject<double>>(
        *context);
  }
};

BoxScene MakeBoxScene() {
  BoxScene s;
  s.scene_graph = std::make_unique<SceneGraph<double>>();
  const auto source = s.scene_graph->RegisterSource("test");
  auto box = std::make_unique<GeometryInstance>(
      RigidTransformd(Eigen::Vector3d(0.20, 0.07, 0.10)),
      std::make_unique<Box>(0.10, 0.10, 0.20), "box");
  box->set_proximity_properties(drake::geometry::ProximityProperties{});
  const GeometryId id =
      s.scene_graph->RegisterAnchoredGeometry(source, std::move(box));
  s.geometries = GeometrySet(std::vector<GeometryId>{id});
  s.context = s.scene_graph->CreateDefaultContext();
  return s;
}

TEST(KeepOutRegionsTest, SampleInsideRegionIsRejected) {
  const BoxScene s = MakeBoxScene();
  EXPECT_FALSE(SampleAvoidsGeometries(EEState({0.20, 0.07, 0.10}), s.query(),
                                      s.geometries, /*clearance=*/0.0));
}

TEST(KeepOutRegionsTest, SampleFarFromRegionIsAccepted) {
  const BoxScene s = MakeBoxScene();
  EXPECT_TRUE(SampleAvoidsGeometries(EEState({0.50, 0.50, 0.15}), s.query(),
                                     s.geometries, /*clearance=*/0.02));
}

TEST(KeepOutRegionsTest, ClearanceInflatesTheRegion) {
  const BoxScene s = MakeBoxScene();
  // 0.01 m outside the +x face: accepted with no clearance, rejected once the
  // required clearance exceeds the gap.
  const Eigen::VectorXd just_outside = EEState({0.26, 0.07, 0.10});
  EXPECT_TRUE(SampleAvoidsGeometries(just_outside, s.query(), s.geometries,
                                     /*clearance=*/0.0));
  EXPECT_FALSE(SampleAvoidsGeometries(just_outside, s.query(), s.geometries,
                                      /*clearance=*/0.02));
}

TEST(KeepOutRegionsTest, EmptyGeometrySetAlwaysPasses) {
  const BoxScene s = MakeBoxScene();
  const GeometrySet empty;
  EXPECT_TRUE(SampleAvoidsGeometries(EEState({0.20, 0.07, 0.10}), s.query(),
                                     empty, /*clearance=*/1.0));
}

// AddKeepOutModelsToPlant loads the example URDF, welds keep_out_base to the
// world, and its collision geometry lands at the pose the URDF specifies.
TEST(KeepOutRegionsTest, AddKeepOutModelsToPlantLoadsAndWeldsExampleUrdf) {
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

  const std::vector<ModelInstanceIndex> indices = AddKeepOutModelsToPlant(
      &plant, &scene_graph,
      {"", "examples/sampling_c3/urdf/keep_out_example.urdf"});
  ASSERT_EQ(indices.size(), 2u);
  EXPECT_FALSE(indices[0].is_valid());  // "" -> no model
  ASSERT_TRUE(indices[1].is_valid());

  plant.Finalize();
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  const auto& plant_context = plant.GetMyContextFromRoot(*diagram_context);

  std::vector<GeometryId> ids;
  for (const auto& body_index : plant.GetBodyIndices(indices[1])) {
    for (const GeometryId& id :
         plant.GetCollisionGeometriesForBody(plant.get_body(body_index))) {
      ids.push_back(id);
    }
  }
  ASSERT_FALSE(ids.empty());
  const GeometrySet keep_out(ids);

  const auto& query_object =
      plant.get_geometry_query_input_port().Eval<QueryObject<double>>(
          plant_context);
  // The example region is a box centred at (0.20, 0.07, 0.10).
  EXPECT_FALSE(SampleAvoidsGeometries(EEState({0.20, 0.07, 0.10}), query_object,
                                      keep_out, /*clearance=*/0.0));
  EXPECT_TRUE(SampleAvoidsGeometries(EEState({0.20, 0.30, 0.10}), query_object,
                                     keep_out, /*clearance=*/0.0));
}

// Regression test for the goal-crosstalk bug: a goal step that declares no
// keep-out model must not be filtered against another step's regions.  Mirrors
// how SamplingC3Controller::BuildKeepOutScene builds one GeometrySet per goal
// step and hands only the active step's set to the sampler.
TEST(KeepOutRegionsTest, InactiveGoalStepDoesNotFilterSamples) {
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

  // Step 0 declares no keep-out model; step 1 declares the example box.
  const std::vector<ModelInstanceIndex> indices = AddKeepOutModelsToPlant(
      &plant, &scene_graph,
      {"", "examples/sampling_c3/urdf/keep_out_example.urdf"});
  plant.Finalize();
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  const auto& plant_context = plant.GetMyContextFromRoot(*diagram_context);
  const auto& query_object =
      plant.get_geometry_query_input_port().Eval<QueryObject<double>>(
          plant_context);

  // One GeometrySet per goal step, as the controller builds them.
  std::vector<GeometrySet> per_step_sets;
  std::vector<bool> per_step_has_regions;
  for (const ModelInstanceIndex& model_instance : indices) {
    std::vector<GeometryId> ids;
    if (model_instance.is_valid()) {
      for (const auto& body_index : plant.GetBodyIndices(model_instance)) {
        for (const GeometryId& id :
             plant.GetCollisionGeometriesForBody(plant.get_body(body_index))) {
          ids.push_back(id);
        }
      }
    }
    per_step_has_regions.push_back(!ids.empty());
    per_step_sets.emplace_back(ids);
  }
  ASSERT_EQ(per_step_sets.size(), 2u);
  EXPECT_FALSE(per_step_has_regions[0]);
  EXPECT_TRUE(per_step_has_regions[1]);

  // The controller only builds an active KeepOutQuery for a step that has
  // regions, so step 0's is inactive.
  const KeepOutQuery step_0{
      &query_object, per_step_has_regions[0] ? &per_step_sets[0] : nullptr};
  const KeepOutQuery step_1{
      &query_object, per_step_has_regions[1] ? &per_step_sets[1] : nullptr};
  EXPECT_FALSE(step_0.active());
  EXPECT_TRUE(step_1.active());

  // A point inside step 1's box: rejected on step 1, accepted on step 0.
  const Eigen::Vector3d inside_step_1_region(0.20, 0.07, 0.10);
  EXPECT_FALSE(SampleAvoidsGeometries(EEState(inside_step_1_region),
                                      *step_1.query_object, *step_1.geometries,
                                      /*clearance=*/0.0));
  EXPECT_FALSE(step_0.active());  // -> SampleIsAcceptable skips condition 4
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
