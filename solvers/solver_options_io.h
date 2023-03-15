#pragma once

#include "drake/solvers/solver_options.h"

/// Struct containing solver options loaded from a yaml file

namespace dairlib::solvers {

using drake::solvers::SolverOptions;
using drake::solvers::CommonSolverOption;

struct SolverOptionsFromYaml {
  // Common solver options
  int print_to_console;
  std::string log_file_name;
  std::map<std::string, int> int_options;
  std::map<std::string, double> double_options;
  std::map<std::string, std::string> string_options;

  template<typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(print_to_console));
    a->Visit(DRAKE_NVP(log_file_name));
    a->Visit(DRAKE_NVP(int_options));
    a->Visit(DRAKE_NVP(double_options));
    a->Visit(DRAKE_NVP(string_options));
  }

  SolverOptions GetAsSolverOptions(const drake::solvers::SolverId& id) {
    SolverOptions options;
    options.SetOption(CommonSolverOption::kPrintToConsole, print_to_console);
    options.SetOption(CommonSolverOption::kPrintFileName, log_file_name);
    for (const auto& [key, val] : int_options) {
      options.SetOption(id, key, val);
    }
    for (const auto& [key, val] : double_options) {
      options.SetOption(id, key, val);
    }
    for (const auto& [key, val] : string_options) {
      options.SetOption(id, key, val);
    }
    return options;
  }
};

}