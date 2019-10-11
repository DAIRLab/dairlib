#include <chrono>
#include <cstring>
#include <fstream>

#include "drake/common/value.h"
#include "lcm_trajectory.h"

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
using drake::systems::lcm::Serializer;

namespace dairlib {

LcmTrajectory::LcmTrajectory(const vector<Trajectory>& trajectories, 
				const vector<string>& trajectory_names, 
				const string& name, 
				const string& description):
				trajectory_names_(trajectory_names){
	metadata_ = constructMetadataObject(name, description);
	// serializer = Serializer<lcmt_saved_traj>();
}

LcmTrajectory::Trajectory::Trajectory(string traj_name, const lcmt_trajectory_block& traj_block){
	int num_points = traj_block.num_points;
	int num_datatypes = traj_block.num_datatypes;
	this->traj_name = traj_name;
	this->datatypes = vector<string>(traj_block.datatypes);
	this->time_vector = VectorXd::Map(traj_block.time_vec.data(),
																		num_points);
	// this->datapoints = MatrixXd(num_points, num_datatypes);
	for (int i = 0; i < num_points; ++i){
		this->datapoints.row(i) = VectorXd::Map(&traj_block.datapoints[i][0], 
																						num_datatypes);
	}
	// this->datapoints = Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>(traj_block.datapoints.data(), num_points, num_datatypes);
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
	// serializer = Serializer<lcmt_saved_traj>();
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
		memcpy(&traj_block.datapoints, cpp_traj->datapoints.data(), sizeof(traj_block.datapoints));
	
		traj.trajectories[index] = traj_block;
		traj.trajectory_names[index] = traj_el.first;
		++index;
	}
	return traj;
} 

void LcmTrajectory::writeToFile(string filepath){
	std::vector<uint8_t> bytes;
	serializer.Serialize(*AbstractValue::Make(generateLcmObject()), &bytes);

	std::ofstream fout(filepath);
	fout << bytes.data();
	fout.close();
}

lcmt_saved_traj LcmTrajectory::loadFromFile(string filepath){
	// std::vector<uint8_t> bytes( std::istreambuf_iterator<char>(infile),
														// std::istreambuf_iterator<char>());

	std::ifstream inFile(filepath, std::ios_base::binary);

	inFile.seekg(0, std::ios_base::end);
	size_t length = inFile.tellg();
	inFile.seekg(0, std::ios_base::beg);

	std::vector<uint8_t> bytes;
	bytes.reserve(length);
	std::copy( std::istreambuf_iterator<char>(inFile),
	           std::istreambuf_iterator<char>(),
	           std::back_inserter(bytes) );

	lcmt_saved_traj traj;
	std::unique_ptr<AbstractValue> traj_value = AbstractValue::Make(traj);
	serializer.Deserialize((void *)bytes.data(), (int)bytes.size(), traj_value.release());
	return traj_value->get_value<lcmt_saved_traj>();
}

lcmt_metadata LcmTrajectory::constructMetadataObject(string name, string description) const{
	lcmt_metadata metadata;
	
	metadata.name = name;
	metadata.description = description;
	// TODO: autofill git details
	return metadata;
}

} // dairlib