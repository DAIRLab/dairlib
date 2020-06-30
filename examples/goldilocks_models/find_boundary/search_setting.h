//
// Created by jianshu on 6/29/20.
//
#include <string>
#include "drake/common/drake_assert.h"

using std::vector

namespace dairlib {
namespace goldilocks_models {

class SearchSetting {
 public:
  // Default constructor
  SearchSetting(){};
  SearchSetting(int search_dim, vector<string> names,
      vector<double> task_0,vector<double> task_delta);

  //getters
  int search_dim(){return search_dim_};
  vector<string> names(){return names_};
  vector<double> task_0(){return task_0_};
  vector<double> task_delta(){return task_delta_};
  int index(string task_name){return name_to_index_map_[task_name]};

  //setters
  void SetExtendComponents(string task_name,int n_element,
      vector<double> element);
}

 private:
  int search_dim_;
  vector<string> names_;
  vector<double> task_0_;
  vector<double> task_delta_;
  std::unordered_map<string, int> name_to_index_map_;
  vector<int> n_elements_;
  vector<vector<double>> elements_;


}
}
