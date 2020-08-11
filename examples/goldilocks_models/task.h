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


  // Generator
  virtual vector<double> NewTask(string dir,int sample_idx) = 0;

  // Printing message
  virtual void PrintInfo() const {};

  //get scale for tasks
  Eigen::VectorXd GetTaskScale() const;

 protected:
  int task_dim_{};
  int task_dim_nondeg_{};
  vector<string> names_;
  vector<int> N_sample_vec_;
  vector<double> task_min_range_;
  vector<double> task_max_range_;
  vector<std::uniform_real_distribution<>> distribution_;
  int N_sample_{};

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
  vector<double> NewTask(string dir,int sample_idx) final;

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
                        const std::vector<double>& task_max);

  // Default constructor
  UniformTasksGenerator()= default;

  // Generator
  vector<double> NewTask(string dir,int sample_idx) final;

  // Printing message
  void PrintInfo() const override;
};


class ExpansionTasksGenerator{
 public:
  ExpansionTasksGenerator(int max_num,bool extend){
    currently_extend_task_space_ = extend;
    num_extending_task_space_ = 0;
    max_num_extending_task_space_ = max_num;
  };
  ExpansionTasksGenerator(){};

  vector<double> NewTask(string dir,int sample_idx,const TasksGenerator* task_gen);

  bool currently_extend_task_space() const {return currently_extend_task_space_;}
  int num_extending_task_space() const {return num_extending_task_space_;}
  int max_num_extending_task_space() const {return max_num_extending_task_space_;}

  //Setter
  void set_num_extending_task_space(int num) {num_extending_task_space_ = num;}
  void set_currently_extend_task_space(bool extend)
  {currently_extend_task_space_ = extend;}

 private:
  bool currently_extend_task_space_;
  int num_extending_task_space_;
  int max_num_extending_task_space_;

  std::default_random_engine random_eng_;
};

class MediateTasksGenerator{
 public:
  MediateTasksGenerator(int N_sample,int task_dim);
  MediateTasksGenerator(){};

  vector<double> NewTask(string dir,int iter,int sample_idx);
  void set_mediate_samples(string dir,int iter,const TasksGenerator* task_gen);
  // Getters
  int total_sample_number() const { return N_sample_; }
  bool start_finding_mediate_sample() const {return start_finding_mediate_sample_;}
  bool currently_find_mediate_sample() const {return currently_find_mediate_sample_;}
  bool choose_sample_from_iter_to_help() const {return choose_sample_from_iter_to_help_;}
  int sample_index_to_help() const {return sample_index_to_help_;}

  // Setters
  void set_start_finding_mediate_sample(bool if_start){
    start_finding_mediate_sample_ = if_start;
  }
  void set_currently_find_mediate_sample(bool currently_find){
    currently_find_mediate_sample_ = currently_find;
  }
  void set_choose_sample_from_iter_to_help(bool if_choose){
    choose_sample_from_iter_to_help_ = if_choose;
  }
  void set_sample_index_to_help(int index){
    sample_index_to_help_ = index;
  };

 private:
  int N_sample_;
  int task_dim_;
  bool start_finding_mediate_sample_;
  bool currently_find_mediate_sample_;
  bool choose_sample_from_iter_to_help_;
  int sample_index_to_help_;
};

//functions related to initial guess
//move it here to avoid cycle dependency graph
double  TaskDistanceCalculation(const VectorXd& past_task,
                                const VectorXd& current_task,
                                const VectorXd& task_scale);
std::string CompareTwoTasks(const string& dir, string prefix1,string prefix2,
                            const VectorXd& current_task,
                            const VectorXd& task_scale);

}  // namespace goldilocks_models
}  // namespace dairlib
