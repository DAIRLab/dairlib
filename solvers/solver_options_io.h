#pragma once

#include "drake/solvers/solver_options.h"
#include "drake/solvers/osqp_solver.h"  // TODO @bibit may not need
#include "solvers/fast_osqp_solver.h"
#include <iostream>


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
  std::map<std::string, int> enum_options;

  template<typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(print_to_console));
    a->Visit(DRAKE_NVP(log_file_name));
    a->Visit(DRAKE_NVP(int_options));
    a->Visit(DRAKE_NVP(double_options));
    a->Visit(DRAKE_NVP(string_options));
    a->Visit(DRAKE_NVP(enum_options));
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
    for (const auto& [key, val] : enum_options) {
      if (key == "linsys_solver") {
        std::cout << "Trying to set linsys solver..." << std::endl;
        if (val == 0) {
          options.SetOption(id, "linsys_solver", 0); //QDLDL_SOLVER);
        }
        else if (val == 1) {
          options.SetOption(id, "linsys_solver", 1); //MKL_PARDISO_SOLVER);
        } else {
          std::cerr << ("Unknown osqp_linsys_solver_type: " + val) << std::endl;
        }
        // std::cout << "Skipping setting osqp solver" << std::endl;
      }
      else {std::cerr << ("Unknown OSQP enum: " + key) << std::endl;}
    }
    return options;
  }
};

}