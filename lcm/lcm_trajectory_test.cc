#include <string>
#include <vector>

#include "lcm_trajectory.h"





namespace dairlib{


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

static const string TEST_FILEPATH = "TEST_FILEPATH";
static const string TEST_TRAJ_NAME = "TEST_TRAJ_NAME";
static const string TEST_TRAJ_DESCRIPTION = "TEST_TRAJ_DESCRIPTION";
static const int NUM_DATAPOINTS = 10;


class LcmTrajectoryTest : public ::testing:Test {
 protected:
 	void SetUp() override {
 		vector<string> trajectory_names;
 		Trajectory traj_1;
 		traj_1.traj_name = TEST_TRAJ_NAME;
 		traj_1.time_vector = VectorXd::Zero(NUM_DATAPOINTS);
 		traj_1.datapoints = MatrixXd::Identity(NUM_DATAPOINTS);

 		vector<string> trajectories = traj_1;
 		

 	}

};

TEST_F(LcmTrajectoryTest, TestSaveTrajectory) {

}

TEST_F(LcmTrajectoryTest, TestLoadTrajectory) {
	
}

TEST_F(LcmTrajectoryTest, TestMetadataConstruction) {
	
}


} // dairlib


int main(int argc, char* argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
