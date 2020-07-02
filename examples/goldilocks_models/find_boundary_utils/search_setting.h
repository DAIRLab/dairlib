//
// Created by jianshu on 6/29/20.
//
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
      vector<double> task_0,vector<double> task_delta);

  //getters
  int task_dim(){return task_dim_;}
  vector<string> names(){return names_;}
  vector<double> task_0(){return task_0_;}
  const vector<double> task_delta(){return task_delta_;}
  int index(string task_name){return name_to_index_map_[task_name];}
  int get_n_element(int task_dim){return n_elements_[task_dim];}
  double get_element(int task_index,int element_index){
    return elements_[task_index][element_index];
  }

  //setters
  void SetExtendComponents(const string task_name,int n_element,
      vector<double> element);

 private:
  int task_dim_;
  vector<string> names_;
  vector<double> task_0_;
  vector<double> task_delta_;
  std::unordered_map<string, int> name_to_index_map_;
  vector<int> n_elements_;
  vector<vector<double>> elements_;
};

}
}
