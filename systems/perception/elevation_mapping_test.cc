#include "gtest/gtest.h"

#include "systems/perception/elevation_mapping_system.h"
#include "systems/framework/output_vector.h"

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"

#include "pcl/common/io.h"

namespace dairlib {
namespace perception {

GTEST_TEST(PerceptionTests, ElevationMappingSystemTest){

  elevation_mapping_params params{
      {{
          "oracle",
          "r_situational_awareness_camera_link",
          drake::math::RigidTransformd{},
      }}, // sensors
      {
        drake::systems::TriggerType::kForced,
        0
      },
      "utorso",
      Eigen::Vector3d::Zero()
  };

  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser(&plant).AddModelsFromUrl(
      "package://drake_models/atlas/atlas_convex_hull.urdf");
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  ElevationMappingSystem mapper(plant, plant_context.get(), params);
  auto sensor_preprocessor =
      std::make_unique<elevation_mapping::PerfectSensorProcessor>(
          elevation_mapping::SensorProcessorBase::GeneralParameters{}
          );
  mapper.AddSensorPreProcessor("oracle", std::move(sensor_preprocessor));

  /*
   * <Setup ATLAS>
   */
  DRAKE_DEMAND(plant.num_velocities() == 36);
  DRAKE_DEMAND(plant.num_positions() == 37);
  const drake::multibody::Body<double>& pelvis = plant.GetBodyByName("pelvis");
  DRAKE_DEMAND(pelvis.is_floating());
  DRAKE_DEMAND(pelvis.has_quaternion_dofs());
  DRAKE_DEMAND(pelvis.floating_positions_start() == 0);
  DRAKE_DEMAND(pelvis.floating_velocities_start() == plant.num_positions());
  const Eigen::Translation3d X_WP(0.0, 0.0, 0.95);
  plant.SetFreeBodyPoseInWorldFrame(plant_context.get(), pelvis, X_WP);

  auto robot_output = systems::OutputVector<double>(
      plant.num_positions(), plant.num_velocities(), plant.num_actuators());
  robot_output.SetZero();
  robot_output.SetPositions(plant.GetPositions(*plant_context));

  /*
  * </Setup ATLAS>
  */

  /* get a dummy pointcloud */
  const auto X_WS = plant.EvalBodyPoseInWorld(
      *plant_context,
      plant.GetBodyByName("r_situational_awareness_camera_link")
  );
  const auto X_SW = X_WS.inverse();

  pcl::PointCloud<pcl::PointXYZ> pc;
  // Approx 200 x 200 pointcloud
  for (int i = -101; i < 101; ++i) {
    for (int j = -101; j < 101; ++j) {
      Eigen::Vector3d p(0.01 * i, 0.01 * j, 0.1);
      p = X_SW * p;
      pcl::PointXYZ pt;
      pt.getVector3fMap() = p.cast<float>();
      pc.push_back(pt);
    }
  }
  elevation_mapping::PointCloudType::Ptr pc_input(new elevation_mapping::PointCloudType);
  pcl::copyPointCloud(pc, *pc_input);

  Eigen::MatrixXd base_cov = Eigen::MatrixXd::Identity(6, 6);
  base_cov.resize(36,1);
  auto context = mapper.CreateDefaultContext();
  mapper.get_input_port_state().FixValue(context.get(), robot_output);
  mapper.get_input_port_covariance().FixValue(context.get(), base_cov);
  mapper.get_input_port_pointcloud("oracle").FixValue(context.get(), pc_input);

  mapper.CalcForcedUnrestrictedUpdate(
      *context,
      &context->get_mutable_state()
  );

  auto map = mapper.get_output_port_map().Eval<elevation_mapping::ElevationMap>(*context);
  const auto& elevation = map.getRawGridMap().get("elevation");
  const auto expected_result = Eigen::MatrixXf::Constant(elevation.rows(), elevation.cols(), 0.1);
  bool pass = drake::CompareMatrices(elevation, expected_result, 1e-6);
  if (pass) {
    drake::log()->info("PASSED!");
  } else {
    drake::log()->error("TEST FAILED!");
    std::cout << "expected matrix: \n" << expected_result <<
                 "\nbut received\n" << elevation << std::endl;
  }
  EXPECT_TRUE(pass);
}
}
}

int main(int argc, char** argv) {
  testing::InitGoogleTest();
  return RUN_ALL_TESTS();
}