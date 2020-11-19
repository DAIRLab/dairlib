#include "lcm/lcm_trajectory.h"

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/value.h"

namespace dairlib {

using drake::AbstractValue;
using Eigen::Dynamic;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::RowMajor;
using Eigen::VectorXd;
using std::make_unique;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

static const char TEST_FILEPATH[] = "TEST_FILEPATH";
static const char TEST_TRAJ_NAME_1[] = "TEST_TRAJ_NAME_1";
static const char TEST_TRAJ_NAME_2[] = "TEST_TRAJ_NAME_2";
static const char TEST_NAME[] = "TEST_NAME";
static const char TEST_DESCRIPTION[] = "TEST_DESCRIPTION";
static const char DATATYPE_1[] = "DATATYPE_1";
static const char DATATYPE_2[] = "DATATYPE_2";
static const char DATATYPE_3[] = "DATATYPE_3";
static const int NUM_DATAPOINTS = 5;
static const int NUM_DATATYPES = 3;
static const int NUM_TRAJECTORIES = 2;

class LcmTrajectoryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    datatypes_.resize(NUM_DATATYPES);
    datatypes_[0] = DATATYPE_1;
    datatypes_[1] = DATATYPE_2;
    datatypes_[2] = DATATYPE_3;

    traj_1_.traj_name = TEST_TRAJ_NAME_1;
    traj_1_.time_vector = VectorXd::Ones(NUM_DATAPOINTS);
    traj_1_.datapoints = MatrixXd::Identity(NUM_DATATYPES, NUM_DATAPOINTS);
    traj_1_.datatypes = datatypes_;

    traj_2_.traj_name = TEST_TRAJ_NAME_2;
    traj_2_.time_vector = VectorXd::Zero(NUM_DATAPOINTS);
    traj_2_.datapoints = MatrixXd::Random(NUM_DATATYPES, NUM_DATAPOINTS);
    traj_2_.datatypes = datatypes_;

    metadata_.datetime = "";
    metadata_.git_dirty_flag = false;
    metadata_.name = TEST_NAME;
    metadata_.description = TEST_DESCRIPTION;

    trajectory_names_.resize(NUM_TRAJECTORIES);
    trajectory_names_[0] = TEST_TRAJ_NAME_1;
    trajectory_names_[1] = TEST_TRAJ_NAME_2;
    trajectories_.resize(NUM_TRAJECTORIES);
    trajectories_[0] = traj_1_;
    trajectories_[1] = traj_2_;

    lcm_traj_ = LcmTrajectory(trajectories_, trajectory_names_, TEST_NAME,
                              TEST_DESCRIPTION);
  }

  LcmTrajectory::Trajectory traj_1_;
  LcmTrajectory::Trajectory traj_2_;
  vector<string> datatypes_;
  vector<LcmTrajectory::Trajectory> trajectories_;
  vector<string> trajectory_names_;
  lcmt_metadata metadata_;
  LcmTrajectory lcm_traj_;
};

TEST_F(LcmTrajectoryTest, TestConstructorFromLcmTObject) {
  lcm_traj_.WriteToFile(TEST_FILEPATH);

  LcmTrajectory loaded_traj = LcmTrajectory(TEST_FILEPATH);

  // Test the LcmTrajectory fields first
  EXPECT_EQ(loaded_traj.GetTrajectoryNames().size(), NUM_TRAJECTORIES);
  lcmt_metadata metadata = loaded_traj.GetMetadata();
  EXPECT_NE(metadata.datetime, metadata_.datetime);
  EXPECT_EQ(metadata.name, TEST_NAME);
  EXPECT_EQ(metadata.description, TEST_DESCRIPTION);
  EXPECT_EQ(metadata.git_dirty_flag, metadata_.git_dirty_flag);

  // Test the individual LcmTrajectory::Trajectory objects
  EXPECT_TRUE(loaded_traj.GetTrajectory(TEST_TRAJ_NAME_1)
                  .time_vector.isApprox(
                      lcm_traj_.GetTrajectory(TEST_TRAJ_NAME_1).time_vector));
  EXPECT_TRUE(loaded_traj.GetTrajectory(TEST_TRAJ_NAME_1)
                  .datapoints.isApprox(
                      lcm_traj_.GetTrajectory(TEST_TRAJ_NAME_1).datapoints));
  EXPECT_TRUE(loaded_traj.GetTrajectory(TEST_TRAJ_NAME_1).datatypes ==
              lcm_traj_.GetTrajectory(TEST_TRAJ_NAME_1).datatypes);
  EXPECT_TRUE(loaded_traj.GetTrajectory(TEST_TRAJ_NAME_2)
                  .time_vector.isApprox(
                      lcm_traj_.GetTrajectory(TEST_TRAJ_NAME_2).time_vector));
  EXPECT_TRUE(loaded_traj.GetTrajectory(TEST_TRAJ_NAME_2)
                  .datapoints.isApprox(
                      lcm_traj_.GetTrajectory(TEST_TRAJ_NAME_2).datapoints));
  EXPECT_TRUE(loaded_traj.GetTrajectory(TEST_TRAJ_NAME_2).datatypes ==
              lcm_traj_.GetTrajectory(TEST_TRAJ_NAME_2).datatypes);
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
