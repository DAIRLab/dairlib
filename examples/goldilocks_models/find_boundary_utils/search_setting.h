#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include "drake/common/drake_assert.h"

using std::vector;
using std::string;

namespace dairlib {
namespace goldilocks_models {

class SearchSetting {
 public:
  // Default constructor
  SearchSetting(){};
  SearchSetting(int task_dim, vector<string> names,
      vector<double> task_0,vector<double> task_delta,
      vector<vector<double>> elements);

  //getters
  int task_dim() const {return task_dim_;}
  const vector<string>& names() const {return names_;}
  const vector<double>& task_0() const {return task_0_;}
  const vector<double>& task_delta() const {return task_delta_;}
  int index(string task_name) const {return name_to_index_map_.at(task_name);}
  int get_n_element(int task_index) const {return elements_[task_index].size();}
  double get_element(int task_index,int element_index) const {
    return elements_[task_index][element_index];
  }

 private:
  int task_dim_;
  vector<string> names_;
  vector<double> task_0_;
  vector<double> task_delta_;
  std::unordered_map<string, int> name_to_index_map_;
  vector<vector<double>> elements_;
};

}
}
