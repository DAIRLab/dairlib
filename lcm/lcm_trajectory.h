#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
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
  LcmTrajectory();
  LcmTrajectory(const std::vector<Trajectory>& trajectories,
                const std::vector<std::string>& trajectory_names,
                const std::string& name = "DEFAULT_NAME",
                const std::string& description = "DEFAULT_DESCRIPTION");

  explicit LcmTrajectory(const lcmt_saved_traj& traj);

  // explicit LcmTrajectory(const std::string& filepath);

  lcmt_saved_traj generateLcmObject() const;

  void writeToFile(std::string filepath);

  lcmt_metadata metadata_;
  std::unordered_map<std::string, Trajectory> trajectories_;
  std::vector<std::string> trajectory_names_;

  static lcmt_saved_traj loadFromFile(const std::string filepath);

 private:
  lcmt_metadata constructMetadataObject(std::string name,
                                        std::string description) const;
};


}  // namespace dairlib
