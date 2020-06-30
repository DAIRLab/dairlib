//
// Created by jianshu on 6/29/20.
//
#include "examples/goldilocks_models/find_boundary/search_setting.h"


namespace dairlib {
namespace goldilocks_models {

SearchSetting::SearchSetting(int search_dim,
                             std::vector <string> names,
                             std::vector<double> task_0,
                             std::vector<double> task_delta)
    : search_dim_(search_dim),
      names_(names),
      task_0_(task_0),
      task_delta_(task_delta) {

  for (unsigned int i = 0; i < names.size(); i++) {
    name_to_index_map_[names[i]] = i;
  }
}

void SearchSetting::SetExtendComponents(string task_name,int n_element,
                         vector<double> element){
  DRAKE_DEMAND(n_element==element.size());
  n_elements_[name_to_index_map_[task_name]] = n_element;
  elements_[name_to_index_map_[task_name]] = element;
}

}
}