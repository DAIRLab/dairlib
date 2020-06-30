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

}


}
}