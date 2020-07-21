#include "examples/goldilocks_models/task.h"

#include <iostream>

namespace dairlib {
namespace goldilocks_models {

TasksGenerator::TasksGenerator(int task_dim, std::vector<string> names,
                               std::vector<int> N_sample_vec)
    : task_dim_(task_dim), names_(names), N_sample_vec_(N_sample_vec) {
  DRAKE_DEMAND(task_dim > 0);
  DRAKE_DEMAND(names.size() == (unsigned)task_dim);
  DRAKE_DEMAND(N_sample_vec.size() == (unsigned)task_dim);
  for (auto n_sample : N_sample_vec) {
    DRAKE_DEMAND(n_sample > 0);
  }

  // Random number generator
  std::random_device randgen;
  random_eng_ = std::default_random_engine(randgen());

  // Non-degenerate task dimension and total sample numbers
  task_dim_nondeg_ = 0;
  N_sample_ = 1;
  for (auto n_sample : N_sample_vec) {
    task_dim_nondeg_ += int(n_sample > 1);
    N_sample_ *= n_sample;
  }

  // Create index map
  for (int i = 0; i < task_dim; i++) {
    name_to_index_map_[names[i]] = i;
  }
}

GridTasksGenerator::GridTasksGenerator(int task_dim, std::vector<string> names,
                                       std::vector<int> N_sample_vec,
                                       std::vector<double> task_0,
                                       std::vector<double> task_delta,
                                       bool is_stochastic)
    : TasksGenerator(task_dim, names, N_sample_vec),
      task_0_(task_0),
      task_delta_(task_delta),
      is_stochastic_(is_stochastic) {
  DRAKE_DEMAND(task_0.size() == (unsigned)task_dim);
  DRAKE_DEMAND(task_delta.size() == (unsigned)task_dim);

  // we don't implement the feature of extending task space for grid method
  currently_extend_task_space_ = false;
  // Construct forward and backward index map
  int i_layer = 0;
  int sample_idx = 0;
  vector<int> index_tuple(task_dim, 0);
  RunThroughIndex(N_sample_vec, i_layer, index_tuple, &sample_idx,
                  &forward_task_idx_map_, &inverse_task_idx_map_);

  // Tasks setup
  for (int i = 0; i < task_dim; i++) {
    // Distribution
    distribution_.emplace_back(-task_delta[i] / 2, task_delta[i] / 2);
    // Task grid
    vector<double> sub_task_grid;
    for (int j = 0 - N_sample_vec[i] / 2;
         j < N_sample_vec[i] - N_sample_vec[i] / 2; j++)
      sub_task_grid.push_back(j * task_delta[i]);
    task_grid_.push_back(sub_task_grid);
    // Min
    task_min_range_.push_back((is_stochastic && (N_sample_vec[i] > 0))
                                  ? task_0[i] + sub_task_grid.front() -
                                        task_delta[i] * 0.5
                                  : task_0[i] + sub_task_grid.front());
    // Max
    task_max_range_.push_back((is_stochastic && (N_sample_vec[i] > 0))
                                  ? task_0[i] + sub_task_grid.back() +
                                        task_delta[i] * 0.5
                                  : task_0[i] + sub_task_grid.back());
  }
}

void GridTasksGenerator::PrintInfo() const {
  for (int i = 0; i < dim(); i++) {
    cout << names()[i] << ": \n";
    cout << "  # of samples = " << sample_numbers()[i] << endl;
    cout << "  center = " << task_0_[i] << endl;
    cout << "  spacing = " << task_delta_[i] << endl;
    cout << "  min = " << task_min_range_[i] << endl;
    cout << "  max = " << task_max_range_[i] << endl;
  }
}

vector<double> GridTasksGenerator::NewNominalTask(int sample_idx) {
  auto index_tuple = forward_task_idx_map_.at(sample_idx);
  /*cout << sample_idx << ", (";
  for (auto mem : index_tuple) {
    cout << mem << ", ";
  } cout << ")\n";*/

  vector<double> ret(task_dim_, 0);
  for (int i = 0; i < task_dim_; i++) {
    ret[i] = task_0_[i] + task_grid_[i][index_tuple[i]];
  }
  return ret;
}

vector<double> GridTasksGenerator::NewTask(int iter,int sample_idx) {
  auto index_tuple = forward_task_idx_map_.at(sample_idx);
  /*cout << sample_idx << ", (";
  for (auto mem : index_tuple) {
    cout << mem << ", ";
  } cout << ")\n";*/

  vector<double> ret(task_dim_, 0);
  for (int i = 0; i < task_dim_; i++) {
    ret[i] = task_0_[i] + task_grid_[i][index_tuple[i]];
    if (is_stochastic_) {
      if (N_sample_vec_[i] > 0) {
        ret[i] += distribution_[i](random_eng_);
      }
    }
  }
  return ret;
}

void GridTasksGenerator::RunThroughIndex(
    const std::vector<int>& N_sample, int i_layer, vector<int> index_tuple,
    int* sample_idx, std::map<int, std::vector<int>>* forward_task_idx_map,
    std::map<std::vector<int>, int>* inverse_task_idx_map) {
  if ((unsigned)i_layer == N_sample.size()) {
    (*forward_task_idx_map)[(*sample_idx)] = index_tuple;
    (*inverse_task_idx_map)[index_tuple] = (*sample_idx);
    (*sample_idx) += 1;
  } else {
    while (index_tuple[i_layer] < N_sample[i_layer]) {
      RunThroughIndex(N_sample, i_layer + 1, index_tuple, sample_idx,
                      forward_task_idx_map, inverse_task_idx_map);
      index_tuple[i_layer] += 1;
    }
  }
}

// Tasks are randomly generated from the whole optimization space
UniformTasksGenerator::UniformTasksGenerator(
    int task_dim, std::vector<string> names, std::vector<int> N_sample_vec,
    const std::vector<double>& task_min, const std::vector<double>& task_max)
    : TasksGenerator(task_dim, names, N_sample_vec) {
  DRAKE_DEMAND(task_min.size() == (unsigned)task_dim);
  DRAKE_DEMAND(task_max.size() == (unsigned)task_dim);
  // initially we extend the task space
  currently_extend_task_space_ = true;
  task_min_range_ = task_min;
  task_max_range_ = task_max;

  // Tasks distribution
  for (int i = 0; i < task_dim; i++) {
    // Distribution
    distribution_.emplace_back(task_min_range_[i], task_max_range_[i]);
  }

  //set the flag of extend the task space as true
  currently_extend_task_space_ = true;
}

void UniformTasksGenerator::PrintInfo() const {
  for (int i = 0; i < dim(); i++) {
    cout << names()[i] << ": \n";
    cout << "  # of samples = " << sample_numbers()[i] << endl;
    cout << "  min = " << task_min_range_[i] << endl;
    cout << "  max = " << task_max_range_[i] << endl;
  }
}

vector<double> UniformTasksGenerator::NewTask(int iter,int sample_idx) {
  //the task space is gradually pushed until reach the final optimization range
  vector<double> ret(task_dim_, 0);
  if(iter<100){
    //extend the task space
    currently_extend_task_space_ = true;
    // decide which direction to extend
    // 1 corresponds to the direction of increasing
    // 0 corresponds t0 the direction of decreasing
    std::uniform_int_distribution<int> int_distribution(0,1);
    int direction_flag = int_distribution(random_eng_);
    //decide the range of optimization by the number of iteration
    double central;
    double interval;
    double new_task_max_range;
    double new_task_min_range;
    for (int i = 0; i < task_dim_; i++) {
      central = (task_max_range_[i]+task_min_range_[i])/2;
      interval = (task_max_range_[i]-task_min_range_[i])/2;
      if(direction_flag==1)
      {
        new_task_max_range = central+(iter+1)*interval/100;
        new_task_min_range = central+iter*interval/100;
      }
      else{
        new_task_max_range = central-iter*interval/100;
        new_task_min_range = central-(iter+1)*interval/100;
      }
      // Distribution
      std::uniform_real_distribution<double> new_distribution (new_task_min_range,
                                                               new_task_max_range);
      ret[i] = new_distribution(random_eng_);
    }
  }
  else{
    //fix the task space range
    currently_extend_task_space_ = false;
    for (int i = 0; i < task_dim_; i++) {
      ret[i] = distribution_[i](random_eng_);
    }
  }
  return ret;
}

}  // namespace goldilocks_models
}  // namespace dairlib
