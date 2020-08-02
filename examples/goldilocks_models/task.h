#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>

#include "examples/goldilocks_models/goldilocks_utils.h"
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

  // Getters and setters for task values
  double get(const string& name) const {
    return task_.at(name_to_index_map_.at(name));
  }
  const std::vector<double>& get() const { return task_; }
  void set(const std::vector<double>& values) {
    DRAKE_DEMAND(values.size() == (unsigned)task_dim_);
    task_ = values;
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
  vector<int> sample_numbers() const { return N_sample_vec_; }
  int total_sample_number() const { return N_sample_; }
  vector<double> task_min_range() const { return task_min_range_; }
  vector<double> task_max_range() const { return task_max_range_; }
  double task_min(const string& name) const {
    return task_min_range_[name_to_index_map_.at(name)];
  }
  double task_max(const string& name) const {
    return task_max_range_[name_to_index_map_.at(name)];
  }
  bool currently_extend_task_space() const {return currently_extend_task_space_;}
  int iter_start_optimization() const {return iter_start_optimization_;}
  bool start_finding_mediate_sample() const {return start_finding_mediate_sample_;}
  int iter_start_finding_mediate_sample() const {return iter_start_finding_mediate_sample_;}

  // Setters
  void set_start_finding_mediate_sample(bool if_start){
    start_finding_mediate_sample_ = if_start;
  }
  void set_iter_start_finding_mediate_sample(int iter_start){
    iter_start_finding_mediate_sample_ = iter_start;
  }
  void set_is_first_time_to_try_mediate_sample(bool is_first_time){
    is_first_time_to_try_mediate_sample_ = is_first_time;
  }

  // Generator
  virtual vector<double> NewTask(string dir,int iter,int sample_idx) = 0;

  // Printing message
  virtual void PrintInfo() const {};

  //get scale for tasks
  Eigen::VectorXd GetGammaScale() const;

 protected:
  int task_dim_{};
  int task_dim_nondeg_{};
  vector<string> names_;
  vector<int> N_sample_vec_;
  vector<double> task_min_range_;
  vector<double> task_max_range_;
  vector<std::uniform_real_distribution<>> distribution_;
  int N_sample_{};
  bool currently_extend_task_space_;
  int iter_start_optimization_;
  bool start_finding_mediate_sample_;
  int iter_start_finding_mediate_sample_;
  bool is_first_time_to_try_mediate_sample_;
  int sample_index_to_help_;

  std::unordered_map<string, int> name_to_index_map_;

  std::default_random_engine random_eng_;
};

class GridTasksGenerator : public TasksGenerator {
 public:
  GridTasksGenerator(int task_dim, std::vector<string> names,
                     std::vector<int> N_sample_vec, std::vector<double> task_0,
                     std::vector<double> task_delta, bool is_stochastic);

  // Default constructor
  GridTasksGenerator()= default;

  // Getters
  const std::map<int, std::vector<int>>& get_forward_map() {
    return forward_task_idx_map_;
  };
  const std::map<std::vector<int>, int>& get_inverse_map() {
    return inverse_task_idx_map_;
  };

  // Generator
  vector<double> NewNominalTask(int sample_idx);
  vector<double> NewTask(string dir,int iter,int sample_idx) final;

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
  bool is_stochastic_{};

  std::map<int, vector<int>> forward_task_idx_map_;
  std::map<vector<int>, int> inverse_task_idx_map_;
};

class UniformTasksGenerator : public TasksGenerator {
 public:
  UniformTasksGenerator(int task_dim, std::vector<string> names,
                        std::vector<int> N_sample_vec,
                        const std::vector<double>& task_min,
                        const std::vector<double>& task_max,
                        int iter_start_optimization);

  // Default constructor
  UniformTasksGenerator()= default;

  // Generator
//  vector<double> NewTask(int sample_idx);
  vector<double> NewTask(string dir,int iter,int sample_idx) final;

  // Printing message
  void PrintInfo() const override;
};

//functions related to initial guess
//move it here to avoid cycle dependency  graph
double GammaDistanceCalculation(const VectorXd& past_gamma,
                                const VectorXd& current_gamma,
                                const VectorXd& gamma_scale);
std::string CompareTwoTasks(const string& dir, string prefix1,string prefix2,
                            const VectorXd& current_gamma,
                            const VectorXd& gamma_scale);

}  // namespace goldilocks_models
}  // namespace dairlib
