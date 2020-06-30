//
// Created by jianshu on 6/29/20.
//
#include <string>

namespace dairlib {
namespace goldilocks_models {

class SearchSetting {
 public:
  // Default constructor
  SearchSetting(){};
  SearchSetting(int search_dim, std::vector<string> names,
      std::vector<double> task_0,std::vector<double> task_delta);

  //getters
  int search_dim(){return search_dim_};
  std::vector<string> names(){return names_};
  std::vector<double> task_0{return task_0_};
  std::vector<double> task_delta{return task_delta_};

  //setters
  void SetExtendComponents(){}

 private:
  int search_dim_;
  std::vector<string> names_;
  std::vector<double> task_0_;
  std:vector<double> task_delta_;


}
}
