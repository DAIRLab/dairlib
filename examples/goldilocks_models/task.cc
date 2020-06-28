#include "examples/goldilocks_models/task.h"

#include <iostream>

namespace dairlib {
namespace goldilocks_models {

GridTasksGenerator::GridTasksGenerator(int task_dim, std::vector<string> names,
                                       std::vector<int> N_sample_vec,
                                       std::vector<double> task_0,
                                       std::vector<double> task_delta,
                                       bool is_stochastic)
    : task_dim_(task_dim),
      names_(names),
      N_sample_vec_(N_sample_vec),
      task_0_(task_0),
      task_delta_(task_delta),
      is_stochastic_(is_stochastic) {
  DRAKE_DEMAND(task_dim > 0);
  DRAKE_DEMAND(names.size() == (unsigned)task_dim);
  DRAKE_DEMAND(N_sample_vec.size() == (unsigned)task_dim);
  DRAKE_DEMAND(task_0.size() == (unsigned)task_dim);
  DRAKE_DEMAND(task_delta.size() == (unsigned)task_dim);
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

  // Print
  for (int i = 0; i < task_dim; i++) {
    cout << names[i] << ": \n";
    cout << "  # of samples = " << N_sample_vec[i] << endl;
    cout << "  center = " << task_0[i] << endl;
    cout << "  spacing = " << task_delta[i] << endl;
    cout << "  min = " << task_min_range_[i] << endl;
    cout << "  max = " << task_max_range_[i] << endl;
  }
}

vector<double> GridTasksGenerator::NewTask(int sample_idx,
                                            bool disable_stochastic) {
  auto index_tuple = forward_task_idx_map_.at(sample_idx);
  /*cout << sample_idx << ", (";
  for (auto mem : index_tuple) {
    cout << mem << ", ";
  } cout << ")\n";*/

  vector<double> ret(task_dim_, 0);
  for (int i = 0; i < task_dim_; i++) {
    ret[i] = task_0_[i] + task_grid_[i][index_tuple[i]];
    if (!disable_stochastic && is_stochastic_) {
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

//Tasks are randomly generated from the whole optimization space
UniformTasksGenerator::UniformTasksGenerator(int task_dim, std::vector<string> names,
                                       std::vector<int> N_sample_vec,
                                       std::vector<double> task_0,
                                       std::vector<double> task_delta,
                                       bool is_stochastic)
    : task_dim_(task_dim),
      names_(names),
      N_sample_vec_(N_sample_vec),
      task_0_(task_0),
      task_delta_(task_delta),
      is_stochastic_(is_stochastic) {
  DRAKE_DEMAND(task_dim > 0);
  DRAKE_DEMAND(names.size() == (unsigned)task_dim);
  DRAKE_DEMAND(N_sample_vec.size() == (unsigned)task_dim);
  DRAKE_DEMAND(task_0.size() == (unsigned)task_dim);
  DRAKE_DEMAND(task_delta.size() == (unsigned)task_dim);
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


  // Tasks setup
  for (int i = 0; i < task_dim; i++) {
    // Task grid
    vector<double> sub_task_grid;
    for (int j = 0 - N_sample_vec[i] / 2;
         j < N_sample_vec[i] - N_sample_vec[i] / 2; j++)
      sub_task_grid.push_back(j * task_delta[i]);
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
    // Distribution
    distribution_.emplace_back(task_min_range_.back(), task_max_range_.back());
  }

  //restricted number of sample
  for (int dim=0;dim<task_dim_;dim++)
  {
    if(N_sample_vec_[dim]>1)
    {
      N_sample_vec_[dim]=pow(N_sample_vec_[dim],0.5)+1;
    }
  }

  // Print
  for (int i = 0; i < task_dim; i++) {
    cout << names[i] << ": \n";
    cout << "  # of samples = " << N_sample_vec[i] << endl;
    cout << "  center = " << task_0[i] << endl;
    cout << "  spacing = " << task_delta[i] << endl;
    cout << "  min = " << task_min_range_[i] << endl;
    cout << "  max = " << task_max_range_[i] << endl;
  }
}

vector<double> UniformTasksGenerator::NewTask(int sample_idx,
                                           bool disable_stochastic) {
  DRAKE_DEMAND(disable_stochastic==false);

  vector<double> ret(task_dim_, 0);
  for (int i = 0; i < task_dim_; i++) {
    ret[i] = distribution_[i](random_eng_);
  }
  return ret;
}

}  // namespace goldilocks_models
}  // namespace dairlib
