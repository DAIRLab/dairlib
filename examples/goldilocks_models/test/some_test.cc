#include <memory>

#include <gtest/gtest.h>

#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"

#include "drake/multibody/plant/multibody_plant.h"

using drake::MatrixX;
using drake::VectorX;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::System;
using drake::systems::SystemOutput;
using std::cout;
using std::endl;
using std::make_unique;
using std::string;
using std::unique_ptr;

namespace dairlib {
namespace goldilocks_models {
namespace {

class MultibodyPlantJointOrderTest : public ::testing::Test {
 protected:
  MultibodyPlantJointOrderTest(){};
};

class EigenTest : public ::testing::Test {
 protected:
  EigenTest(){};
};

TEST_F(MultibodyPlantJointOrderTest, FixedSpringCassie) {
  // Because in some of my code I hard-coded the joint index.
  // Putting a test here to give myself a headsup.

  drake::multibody::MultibodyPlant<double> plant(1e-3);
  addCassieMultibody(&plant, nullptr, true /*floating base*/,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf",
                     false /*spring model*/, false /*loop closure*/);
  plant.Finalize();

  std::map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);

  EXPECT_TRUE(pos_map.at("base_qw") == 0);
  EXPECT_TRUE(pos_map.at("base_qx") == 1);
  EXPECT_TRUE(pos_map.at("base_qy") == 2);
  EXPECT_TRUE(pos_map.at("base_qz") == 3);
  EXPECT_TRUE(pos_map.at("base_x") == 4);
  EXPECT_TRUE(pos_map.at("base_y") == 5);
  EXPECT_TRUE(pos_map.at("base_z") == 6);
  EXPECT_TRUE(pos_map.at("hip_roll_left") == 7);
  EXPECT_TRUE(pos_map.at("hip_roll_right") == 8);
  EXPECT_TRUE(pos_map.at("hip_yaw_left") == 9);
  EXPECT_TRUE(pos_map.at("hip_yaw_right") == 10);
  EXPECT_TRUE(pos_map.at("hip_pitch_left") == 11);
  EXPECT_TRUE(pos_map.at("hip_pitch_right") == 12);
  EXPECT_TRUE(pos_map.at("knee_left") == 13);
  EXPECT_TRUE(pos_map.at("knee_right") == 14);
  EXPECT_TRUE(pos_map.at("ankle_joint_left") == 15);
  EXPECT_TRUE(pos_map.at("ankle_joint_right") == 16);
  EXPECT_TRUE(pos_map.at("toe_left") == 17);
  EXPECT_TRUE(pos_map.at("toe_right") == 18);

  std::map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);

  EXPECT_TRUE(vel_map.at("base_wx") == 0);
  EXPECT_TRUE(vel_map.at("base_wy") == 1);
  EXPECT_TRUE(vel_map.at("base_wz") == 2);
  EXPECT_TRUE(vel_map.at("base_vx") == 3);
  EXPECT_TRUE(vel_map.at("base_vy") == 4);
  EXPECT_TRUE(vel_map.at("base_vz") == 5);
  EXPECT_TRUE(vel_map.at("hip_roll_leftdot") == 6);
  EXPECT_TRUE(vel_map.at("hip_roll_rightdot") == 7);
  EXPECT_TRUE(vel_map.at("hip_yaw_leftdot") == 8);
  EXPECT_TRUE(vel_map.at("hip_yaw_rightdot") == 9);
  EXPECT_TRUE(vel_map.at("hip_pitch_leftdot") == 10);
  EXPECT_TRUE(vel_map.at("hip_pitch_rightdot") == 11);
  EXPECT_TRUE(vel_map.at("knee_leftdot") == 12);
  EXPECT_TRUE(vel_map.at("knee_rightdot") == 13);
  EXPECT_TRUE(vel_map.at("ankle_joint_leftdot") == 14);
  EXPECT_TRUE(vel_map.at("ankle_joint_rightdot") == 15);
  EXPECT_TRUE(vel_map.at("toe_leftdot") == 16);
  EXPECT_TRUE(vel_map.at("toe_rightdot") == 17);

  std::map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  EXPECT_TRUE(act_map.at("hip_roll_left_motor") == 0);
  EXPECT_TRUE(act_map.at("hip_roll_right_motor") == 1);
  EXPECT_TRUE(act_map.at("hip_yaw_left_motor") == 2);
  EXPECT_TRUE(act_map.at("hip_yaw_right_motor") == 3);
  EXPECT_TRUE(act_map.at("hip_pitch_left_motor") == 4);
  EXPECT_TRUE(act_map.at("hip_pitch_right_motor") == 5);
  EXPECT_TRUE(act_map.at("knee_left_motor") == 6);
  EXPECT_TRUE(act_map.at("knee_right_motor") == 7);
  EXPECT_TRUE(act_map.at("toe_left_motor") == 8);
  EXPECT_TRUE(act_map.at("toe_right_motor") == 9);
}

TEST_F(EigenTest, MapMatrixXdToVectorXd) {
  // I did a mapping operation in cassie_hybrid_rom_planner_system when
  // is_RL_training == true. Adding a unit test here just in case.

  Eigen::MatrixXd A(2, 2);
  A << 1, 2, 3, 4;
  Eigen::VectorXd A_vec = Eigen::Map<const Eigen::VectorXd>(A.data(), A.size());
  cout << "A = \n" << A << endl;
  cout << "A_vec = \n" << A_vec << endl;

  Eigen::VectorXd A_vec_expected(4);
  A_vec_expected << 1, 3, 2, 4;

  EXPECT_TRUE((A_vec - A_vec_expected).norm() == 0);
}

}  // namespace
}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
