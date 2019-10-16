#include <vector>
#include <string>
#include <unordered_map>
#include <Eigen/Dense>
#include "drake/systems/lcm/serializer.h"

#include "dairlib/lcmt_saved_traj.hpp"

namespace dairlib {

class LcmTrajectory {
 public:
  struct Trajectory {
    Trajectory();
    Trajectory(std::string traj_name, const lcmt_trajectory_block& traj_block);
    std::string traj_name;
    Eigen::VectorXd time_vector;
    Eigen::MatrixXd datapoints;
    std::vector<std::string> datatypes;
  };

  LcmTrajectory(const std::vector<Trajectory>& trajectories,
                const std::vector<std::string>& trajectory_names,
                const std::string& name = "DEFAULT_NAME",
                const std::string& description = "DEFAULT_DESCRIPTION");

  LcmTrajectory(const lcmt_saved_traj& traj);

  LcmTrajectory(const std::string filepath);

  lcmt_saved_traj generateLcmObject() const;

  void writeToFile(std::string filepath);

  lcmt_metadata metadata_;
  std::unordered_map<std::string, Trajectory> trajectories_;
  std::vector<std::string> trajectory_names_;
 private:
  lcmt_metadata constructMetadataObject(	std::string name,
                                          std::string description) const;
  lcmt_saved_traj loadFromFile(const std::string filepath);
};


} // dairlib