#include <chrono>
#include <vector>
#include <string>
#include <unordered_map>
#include "lcmt_trajectory.h"


using std::vector;
using std::string;
using std::unordered_map;
using Eigen::VectorXd;
using Eigen::MatrixXd;

static const int MAX_FILE_SIZE_IN_BYTES = 1000000000;

class LcmTrajectory{
public:
	struct Trajectory{
		string traj_name;
		VectorXd time_vector;
		MatrixXd datapoints;
		vector<string> datatypes;
	};

	LcmTrajectory(vector<Trajectory> trajectories, 
								vector<string> trajectory_names, 
								string name = "DEFAULT_NAME", 
								string description = "DEFAULT_DESCRIPTION"):
								trajectories_(trajectories),
								trajectory_names_(trajectory_names){
		metadata_ = constructMetadataObject(name, description);
		serializer = Serializer<lcmt_trajectory>();
	}

	LcmTrajectory(lcmt_trajectory trajectory){
		// TODO: Reconstruct LcmTrajectory from lcmt_trajectory
	}

	lcmt_trajectory generateLcmObject() const{
		lcmt_trajectory traj;
		traj.num_trajectories = trajectories_.size();

		int index = 0;
		for (std::pair<string, Trajectory> traj : trajectories_){
			traj.trajectory_names[index] = traj.first;

			// TODO: Construct trajectory_t structs from Trajectory structs
			++index;
		}
		traj.metadata = constructMetadataObject();
		return traj;
	}	

	void writeToFile(string filepath){
		std::vector<uint8_t> bytes;
		serializer.Serialize(generateLcmObject(), bytes);

		std::ofstream fout(filepath);
		fout << bytes;
		fout.close();
	}

	LcmTrajectory loadFromFile(string filepath){
		std::vector<uint8_t> bytes;
		bytes.resize(MAX_FILE_SIZE_IN_BYTES);
		std::ifstream fin(filepath, std::ios::binary);
		lcmt_trajectory traj;

		// TODO: Read file in to bytes
		serializer.Deserialize(bytes, bytes.size(), &traj);
		return LcmTrajectory(traj);
	}

private:
	metadata_t constructMetadataObject(string name, string description) const{
		metadata_t metadata;
		
		metadata.name = name;
		metadata.description = description;
		// TODO: autofill git details
		return metadata;
	}

	drake::systems::lcm::Serializer<lcmt_trajectory> serializer;
	metadata_t metadata_;
	unordered_map<string, Trajectory> trajectories_;
	vector<string> trajectory_names_;
};