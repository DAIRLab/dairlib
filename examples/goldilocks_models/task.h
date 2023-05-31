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

  void Print() const {
    for (int i = 0; i < names_.size(); i++) {
      cout << names_.at(i) << ": " << task_.at(i) << endl;
    }
  }

  // Getters and setters for task values
  double get(const string& name) const {
    return task_.at(name_to_index_map_.at(name));
  }
  const std::vector<double>& get() const { return task_; }
  void set(const std::vector<double>& values) {
    DRAKE_DEMAND(values.size() == (unsigned)task_dim_);
    task_ = values;

    // Hacks -- forbid "non-zero turning rate + non-zero ground incline"
    {
      double duration = values.at(name_to_index_map_.at("duration"));
      double ground_incline =
          values.at(name_to_index_map_.at("ground_incline"));
      double turning_rate = values.at(name_to_index_map_.at("turning_rate"));

      double turning_angle = turning_rate * duration;
      if (ground_incline != 0 && turning_angle != 0) {
        if (ground_incline > turning_angle) {
          task_.at(name_to_index_map_.at("turning_rate")) = 0;
        } else {
          task_.at(name_to_index_map_.at("ground_incline")) = 0;
        }
      }
    }
  }
  // Other getters
  const std::unordered_map<string, int>& name_to_index_map() {
    return name_to_index_map_;
  };

 private:
  int task_dim_;
  std::vector<string> names_;
  std::vector<double> task_;
  std::unordered_map<string, int> name_to_index_map_;
};

class TasksGenerator {
 public:
  TasksGenerator(int task_dim, std::vector<string> names,
                 std::vector<int> N_sample_vec);

  // Default constructor
  TasksGenerator(){};

  // Getters
  vector<string> names() const { return names_; }
  int dim() const { return task_dim_; }
  int dim_nondeg() const { return task_dim_nondeg_; }
  int total_sample_number() const { return N_sample_; }
  vector<double> task_min_range() const { return task_min_range_; }
  vector<double> task_max_range() const { return task_max_range_; }
  double task_min(const string& name) const {
    return task_min_range_[name_to_index_map_.at(name)];
  }
  double task_max(const string& name) const {
    return task_max_range_[name_to_index_map_.at(name)];
  }
  virtual const vector<int>& sample_numbers() const { return N_sample_vec_; }
  virtual Eigen::MatrixXd sample_numbers_in_matrix_form() const {
    Eigen::MatrixXd n_samples(1, N_sample_vec_.size());
    for (int i = 0; i < n_samples.cols(); i++) {
      n_samples(0, i) = N_sample_vec_.at(i);
    }
    return n_samples;
  }

  // Generator
  virtual vector<double> NewTask(int sample_idx) = 0;

  // Printing message
  virtual void PrintInfo() const {};

 protected:
  int task_dim_{};
  int task_dim_nondeg_{};
  vector<string> names_;
  vector<double> task_min_range_;
  vector<double> task_max_range_;
  vector<std::uniform_real_distribution<>> distribution_;
  int N_sample_{};

  std::unordered_map<string, int> name_to_index_map_;

  std::default_random_engine random_eng_;

 private:
  vector<int> N_sample_vec_;
};

class GridTasksGenerator : public TasksGenerator {
 public:
  GridTasksGenerator(int task_dim, std::vector<string> names,
                     std::vector<int> N_sample_vec, std::vector<double> task_0,
                     std::vector<double> task_delta,
                     std::vector<bool> is_stochastic);

  // Default constructor
  GridTasksGenerator() = default;

  // Getters
  const std::map<int, std::vector<int>>& get_forward_map() const {
    return forward_task_idx_map_;
  };
  const std::map<std::vector<int>, int>& get_inverse_map() const {
    return inverse_task_idx_map_;
  };

  // Generator
  vector<double> NewNominalTask(int sample_idx);
  vector<double> NewTask(int sample_idx) final;

  // Specialized method -- create adjacent matrix
  // n_node_vec is used for extra check on number of nodes for each sample
  Eigen::MatrixXi CreateSampleIdxAdjacentMatrix(
      std::vector<int> n_node_vec = {}) const;

  // Printing message
  void PrintInfo() const override;

 private:
  static void RunThroughIndex(
      const std::vector<int>& N_sample, int i_layer, vector<int> index_tuple,
      int* sample_idx, std::map<int, std::vector<int>>* forward_task_idx_map,
      std::map<std::vector<int>, int>* inverse_task_idx_map);

  vector<double> task_0_;
  vector<double> task_delta_;
  vector<vector<double>> task_grid_;
  std::vector<bool> is_stochastic_{};

  std::map<int, vector<int>> forward_task_idx_map_;
  std::map<vector<int>, int> inverse_task_idx_map_;
};

// TODO(yminchen): Currently it only takes two grid. We can generalize this
//  class to more then 2 grids.
class MultiGridTasksGenerator : public TasksGenerator {
 public:
  MultiGridTasksGenerator(GridTasksGenerator task_gen1,
                          GridTasksGenerator task_gen2);

  // Default constructor
  MultiGridTasksGenerator() = default;

  const vector<int>& sample_numbers() const override {
    throw "please use sample_numbers(int) instead of sample_numbers()";
    return vector<int>();
  }
  const vector<int>& sample_numbers(int i) const {
    if (i == 0) {
      return task_gen1_.sample_numbers();
    } else {
      return task_gen2_.sample_numbers();
    }
  }
  Eigen::MatrixXd sample_numbers_in_matrix_form() const override {
    Eigen::MatrixXd n_samples(2, task_gen1_.sample_numbers().size());
    for (int i = 0; i < n_samples.cols(); i++) {
      n_samples(0, i) = task_gen1_.sample_numbers().at(i);
      n_samples(1, i) = task_gen2_.sample_numbers().at(i);
    }
    return n_samples;
  }

  // Generator
  vector<double> NewNominalTask(int sample_idx);
  vector<double> NewTask(int sample_idx) final;

  // Specialized method -- create adjacent matrix
  // n_node_vec is used for extra check on number of nodes for each sample
  Eigen::MatrixXi CreateSampleIdxAdjacentMatrix(
      std::vector<int> n_node_vec = {}) const;

  // Printing message
  void PrintInfo() const override;

 private:
  GridTasksGenerator task_gen1_;
  GridTasksGenerator task_gen2_;
  std::vector<int> N_sample_of_each_grid_;
};

class UniformTasksGenerator : public TasksGenerator {
 public:
  UniformTasksGenerator(int task_dim, std::vector<string> names,
                        std::vector<int> N_sample_vec,
                        const std::vector<double>& task_min,
                        const std::vector<double>& task_max);

  // Default constructor
  UniformTasksGenerator() = default;

  // Generator
  vector<double> NewTask(int sample_idx) final;

  // Printing message
  void PrintInfo() const override;
};

}  // namespace goldilocks_models
}  // namespace dairlib
