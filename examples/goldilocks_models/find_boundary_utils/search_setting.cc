#include "examples/goldilocks_models/find_boundary_utils/search_setting.h"


namespace dairlib {
namespace goldilocks_models {

SearchSetting::SearchSetting(int task_dim,
                             std::vector <string> names,
                             std::vector<double> task_0,
                             std::vector<double> task_delta,
                             std::vector<std::vector<double>> elements)
    : task_dim_(task_dim),
      names_(names),
      task_0_(task_0),
      task_delta_(task_delta),
      elements_(elements){

  for (unsigned int i = 0; i < names.size(); i++) {
    name_to_index_map_[names[i]] = i;
  }
}

}
}