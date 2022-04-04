#pragma once
#include <vector>

namespace dairlib::learning {

template<typename T, typename... Args>
class ParametrizedSystem: public T {
 public:
  ParametrizedSystem(std::vector<double> default_params, Args... args);

};

}

