#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::shared_ptr;
using std::string;
using std::to_string;
using std::vector;

namespace dairlib {
namespace goldilocks_models {

// Task class
class Task {
 public:
  Task(vector<string> names) : task_dim_(names.size()), names_(names) {
    // Create index map
    for (unsigned int i = 0; i < names.size(); i++) {
      name_to_index_map_[names[i]] = i;
    }
  }

  // Getters and setters
  double get(const string& name) const {
    return task_.at(name_to_index_map_.at(name));
  }
  int dim() {return task_dim_};
  const std::vector<double>& get() const { return task_; }
  void set(const std::vector<double>& values) {
    DRAKE_DEMAND(values.size() == (unsigned)task_dim_);
    task_ = values;
  }

 private:
  int task_dim_;
  std::vector<string> names_;
  std::vector<double> task_;
  std::unordered_map<string, int> name_to_index_map_;
};

class GridTasksGenerator {
 public:
  GridTasksGenerator(int task_dim, std::vector<string> names,
                     std::vector<int> N_sample_vec, std::vector<double> task_0,
                     std::vector<double> task_delta, bool is_stochastic);

  // Default constructor
  GridTasksGenerator(){};

  // Getters
  vector<string> names() { return names_; }
  int dim() { return task_dim_; }
  int dim_nondeg() { return task_dim_nondeg_; }
  vector<int> sample_numbers() { return N_sample_vec_; }
  int total_sample_number() { return N_sample_; }
  double task_min(const string& name) {
    return task_min_range_[name_to_index_map_.at(name)];
  }
  double task_max(const string& name) {
    return task_max_range_[name_to_index_map_.at(name)];
  }
  vector<double> task_min_range() { return task_min_range_; }
  vector<double> task_max_range() { return task_max_range_; }
  const std::map<int, std::vector<int>>& get_forward_map() {
    return forward_task_idx_map_;
  };
  const std::map<std::vector<int>, int>& get_inverse_map() {
    return inverse_task_idx_map_;
  };

  // Generator
  vector<double> NewTask(int sample_idx, bool disable_stochastic);

 private:
  static void RunThroughIndex(
      const std::vector<int>& N_sample, int i_layer, vector<int> index_tuple,
      int* sample_idx, std::map<int, std::vector<int>>* forward_task_idx_map,
      std::map<std::vector<int>, int>* inverse_task_idx_map);

  int task_dim_;
  int task_dim_nondeg_;
  vector<string> names_;
  vector<int> N_sample_vec_;
  vector<double> task_0_;
  vector<double> task_delta_;
  vector<double> task_min_range_;
  vector<double> task_max_range_;
  vector<vector<double>> task_grid_;
  vector<std::uniform_real_distribution<>> distribution_;
  int N_sample_;
  bool is_stochastic_;

  std::map<int, vector<int>> forward_task_idx_map_;
  std::map<vector<int>, int> inverse_task_idx_map_;

  std::unordered_map<string, int> name_to_index_map_;

  std::default_random_engine random_eng_;
};

class UniformTasksGenerator {
 public:
  UniformTasksGenerator(int task_dim, std::vector<string> names,
                        std::vector<int> N_sample_vec,
                        std::vector<double> task_min,
                        std::vector<double> task_max);

  // Default constructor
  UniformTasksGenerator(){};

  // Getters
  vector<string> names() { return names_; }
  int dim() { return task_dim_; }
  int dim_nondeg() { return task_dim_nondeg_; }
  vector<int> sample_numbers() { return N_sample_vec_; }
  int total_sample_number() { return N_sample_; }
  double task_min(const string& name) {
    return task_min_range_[name_to_index_map_.at(name)];
  }
  double task_max(const string& name) {
    return task_max_range_[name_to_index_map_.at(name)];
  }

  // Generator
  vector<double> NewTask(int sample_idx);

 private:
  int task_dim_;
  int task_dim_nondeg_;
  vector<string> names_;
  vector<int> N_sample_vec_;
  vector<double> task_min_range_;
  vector<double> task_max_range_;
  vector<std::uniform_real_distribution<>> distribution_;
  int N_sample_;

  std::unordered_map<string, int> name_to_index_map_;

  std::default_random_engine random_eng_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
