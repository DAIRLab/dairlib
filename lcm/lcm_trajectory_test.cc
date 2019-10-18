#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <string>
#include <chrono>
#include <unordered_map>
#include <vector>
#include "drake/common/value.h"

#include "lcm/lcm_trajectory.h"

namespace dairlib {
namespace {

using std::unique_ptr;
using std::make_unique;
using drake::AbstractValue;
using std::vector;
using std::string;
using std::unordered_map;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::RowMajor;
using Eigen::Map;

static const char TEST_FILEPATH[] = "TEST_FILEPATH";
static const char TEST_TRAJ_NAME_1[] = "TEST_TRAJ_NAME_1";
static const char TEST_TRAJ_NAME_2[] = "TEST_TRAJ_NAME_2";
static const char TEST_NAME[] = "TEST_NAME";
static const char TEST_DESCRIPTION[] = "TEST_DESCRIPTION";
static const char DATATYPE_1[] = "DATATYPE_1";
static const char DATATYPE_2[] = "DATATYPE_2";
static const int NUM_DATAPOINTS = 5;
static const int NUM_TRAJECTORIES = 2;


void runLcmTrajectoryTest() {
  lcmt_trajectory_block traj_block_;
  lcmt_saved_traj lcmt_traj_;
  LcmTrajectory::Trajectory traj_1_;
  LcmTrajectory::Trajectory traj_2_;
  vector<string> datatypes_;
  vector<LcmTrajectory::Trajectory> trajectories_;
  vector<string> trajectory_names_;
  lcmt_metadata metadata_;
  LcmTrajectory lcm_traj_;

  datatypes_.resize(NUM_DATAPOINTS);
  datatypes_[0] = DATATYPE_1;
  datatypes_[1] = DATATYPE_2;

  traj_1_.traj_name = TEST_TRAJ_NAME_1;
  traj_1_.time_vector = VectorXd::Ones(NUM_DATAPOINTS);
  traj_1_.datapoints = MatrixXd::Identity(NUM_DATAPOINTS, NUM_DATAPOINTS);
  traj_1_.datatypes = datatypes_;

  traj_2_.traj_name = TEST_TRAJ_NAME_2;
  traj_2_.time_vector = VectorXd::Zero(NUM_DATAPOINTS);
  traj_2_.datapoints = MatrixXd::Identity(NUM_DATAPOINTS, NUM_DATAPOINTS);
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

  lcm_traj_ = LcmTrajectory(trajectories_,
                            trajectory_names_,
                            TEST_NAME,
                            TEST_DESCRIPTION);
  lcmt_traj_ = lcmt_saved_traj();


  lcm_traj_.writeToFile(TEST_FILEPATH);

  LcmTrajectory loaded_traj_1 = LcmTrajectory(
                                  LcmTrajectory::loadFromFile(TEST_FILEPATH));

  DRAKE_ASSERT(loaded_traj_1.metadata_.datetime == metadata_.datetime);
  DRAKE_ASSERT(loaded_traj_1.metadata_.git_dirty_flag == false);
  DRAKE_ASSERT(loaded_traj_1.metadata_.name == metadata_.name);
  DRAKE_ASSERT(loaded_traj_1.metadata_.description == metadata_.description);

  DRAKE_ASSERT(
    loaded_traj_1.trajectories_[TEST_TRAJ_NAME_1].time_vector.isApprox(
      lcm_traj_.trajectories_[TEST_TRAJ_NAME_1].time_vector));
  DRAKE_ASSERT(
    loaded_traj_1.trajectories_[TEST_TRAJ_NAME_2].time_vector.isApprox(
      lcm_traj_.trajectories_[TEST_TRAJ_NAME_2].time_vector));

  return;
}


}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) {
  dairlib::runLcmTrajectoryTest();
}
