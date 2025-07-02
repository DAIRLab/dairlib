#pragma once

#include "drake/solvers/solver_options.h"
#include "drake/solvers/osqp_solver.h"

namespace dairlib::solvers {

using drake::solvers::SolverOptions;
using drake::solvers::CommonSolverOption;

/*
 * Struct containing solver options loaded from a YAML.
 * Two common options are print_to_console (1 or 0) and log_file_name
 * (if not applicable, use "").
 *
 * If you aren't supplying options for a given
 * type, provide an empty mapping container, i.e. string_options: {}
 *
 * Loading occurs in two steps in order to easily use the same serialization for
 * any solver. First load this struct from a YAML, then get a
 * drake::solvers::SolverOptions object by calling GetAsSolverOptions and
 * supplying the id of your solver. For example:
 *
 * auto solver_options_from_yaml =
 *    drake::yaml::LoadYamlFile<SolverOptionsFromYaml>(filename);
 * auto osqp_solver_options = solver_options_from_yaml.GetAsSolverOptions(
 *    drake::solvers::OsqpSolver::id());
 *
 */
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
      //std::cout << key << ", " << val << ", " << typeid(val).name() << std::endl;
      if (key == "linsys_solver") {
        //options.SetOption(id, "osqp_linsys_solver_type", val);
      } else {
        options.SetOption(id, key, val);
      }
    }
    for (const auto& [key, val] : double_options) {
      //std::cout << key << ", " << val << ", " << typeid(val).name() << std::endl;
      if (key == "adaptive_rho_interval") {
        int val_int = static_cast<int>(val);
        options.SetOption(id, key, val_int);
      } else {
        options.SetOption(id, key, val);
      }
    }
    for (const auto& [key, val] : string_options) {
      //std::cout << key << ", " << val << ", " << typeid(val).name() << std::endl;
      options.SetOption(id, key, val);
    }
    return options;
  }
};

}