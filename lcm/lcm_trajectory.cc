#include <chrono>
#include <cstring>

#include "lcm_trajectory.h"


using std::vector;
using std::string;
using std::unordered_map;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Map;
using drake::systems::lcm::Serializer;

namespace dairlib {

LcmTrajectory::LcmTrajectory(vector<Trajectory> trajectories, 
				vector<string> trajectory_names, 
				string name, 
				string description):
				trajectory_names_(trajectory_names){
	metadata_ = constructMetadataObject(name, description);
	serializer = Serializer<lcmt_saved_traj>();
}

LcmTrajectory::Trajectory(string traj_name, const lcmt_saved_traj& traj_block){
	int num_points = traj_block.num_points;
	int num_datatypes = traj_block.num_datatypes;
	this->traj_name = traj_name;
	this->datatypes = vector<string>(traj_block.traj_names.data());
	this->time_vector = Map<Vector<double, Dynamic>>(traj_block.time_vec.data());
	this->datapoints = Map<Matrix<double, Dynamic, Dynamic>>(traj_block.datapoints.data(), num_points, num_datatypes);
}

LcmTrajectory::LcmTrajectory(const lcmt_saved_traj& traj){
	metadata_ = traj.metadata;
	trajectories_ = unordered_map<string, Trajectory>();
	trajectory_names_ = vector<std::string>();
	for (int i = 0; i < traj.num_trajectories; ++i) {
		string traj_name = traj.trajectory_names[i];
		trajectory_names_.push_back(traj_name);
		trajectories_[traj_name] = Trajectory(traj_name, traj.trajectories[i]);
	}
	serializer = Serializer<lcmt_saved_traj>();
}

lcmt_saved_traj LcmTrajectory::generateLcmObject() const{
	lcmt_saved_traj traj;
	traj.num_trajectories = trajectories_.size();
	traj.metadata = metadata_;

	int index = 0;
	for (std::pair<string, Trajectory> traj_el : trajectories_){
		lcmt_trajectory_block traj_block;
		Trajectory* cpp_traj = &traj_el.second;

		traj_block.num_points = cpp_traj->time_vector.size();
		traj_block.num_datatypes = cpp_traj->datatypes.size();
		memcpy(&traj_block.time_vec, cpp_traj->time_vector.data(), sizeof(traj_block.time_vec));
		memcpy(&traj_block.datatypes, cpp_traj->datatypes.data(), sizeof(traj_block.datatypes));
		memcpy(&traj_block.data_points, cpp_traj->datapoints.data(), sizeof(traj_block.data_points));
	
		traj.trajectories[index] = traj_block;
		traj.trajectory_names[index] = traj_el.first;
		++index;
	}
	return traj;
} 

void LcmTrajectory::writeToFile(string filepath){
	std::vector<uint8_t> bytes;
	serializer.Serialize(generateLcmObject(), &bytes);

	std::ofstream fout(filepath);
	fout << bytes;
	fout.close();
}

LcmTrajectory LcmTrajectory::loadFromFile(string filepath){
	std::ifstream infile(filepath, std::ios_base::binary);
	std::vector<char> bytes( std::istreambuf_iterator<char>(infile),
														std::istreambuf_iterator<char>());

	lcmt_saved_traj traj;
	serializer.Deserialize(bytes, bytes.size(), &traj);
	return LcmTrajectory(traj);
}

lcmt_metadata LcmTrajectory::constructMetadataObject(string name, string description) const{
	lcmt_metadata metadata;
	
	metadata.name = name;
	metadata.description = description;
	// TODO: autofill git details
	return metadata;
}

} // dairlib