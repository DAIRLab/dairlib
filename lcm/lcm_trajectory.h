#include <vector>
#include <string>
#include <unordered_map>
#include <Eigen/Dense>
#include "drake/systems/lcm/serializer.h"

#include "dairlib/lcmt_saved_traj.hpp"

static const int MAX_FILE_SIZE_IN_BYTES = 10000000;

namespace dairlib{

class LcmTrajectory{
public:
	struct Trajectory{
		std::string traj_name;
		Eigen::VectorXd time_vector;
		Eigen::MatrixXd datapoints;
		std::vector<std::string> datatypes;
	};

	LcmTrajectory(std::vector<Trajectory> trajectories, 
								std::vector<std::string> trajectory_names, 
								std::string name = "DEFAULT_NAME", 
								std::string description = "DEFAULT_DESCRIPTION");

	LcmTrajectory(lcmt_saved_traj trajectory);

	lcmt_saved_traj generateLcmObject() const;

	void writeToFile(std::string filepath);

	LcmTrajectory loadFromFile(std::string filepath);

private:
	lcmt_metadata constructMetadataObject(	std::string name, 
																			std::string description) const;

	drake::systems::lcm::Serializer<lcmt_saved_traj> serializer;
	lcmt_metadata metadata_;
	std::unordered_map<std::string, Trajectory> trajectories_;
	std::vector<std::string> trajectory_names_;
};


} // dairlib