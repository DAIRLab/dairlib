#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "lcm/rom_planner_saved_trajectory.h"
#include "lcm/hybrid_rom_planner_saved_trajectory.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

PYBIND11_MODULE(lcm_trajectory, m) {
  m.doc() = "Binding functions for saving/loading trajectories";

  py::class_<lcmt_metadata>(m, "lcmt_metadata")
      .def_readwrite("datetime", &lcmt_metadata::datetime)
      .def_readwrite("name", &lcmt_metadata::name)
      .def_readwrite("description", &lcmt_metadata::description)
      .def_readwrite("git_commit_hash", &lcmt_metadata::git_commit_hash);

  py::class_<LcmTrajectory::Trajectory>(m, "Trajectory")
      .def(py::init<>())
      .def_readwrite("traj_name", &LcmTrajectory::Trajectory::traj_name)
      .def_readwrite("time_vector", &LcmTrajectory::Trajectory::time_vector)
      .def_readwrite("datapoints", &LcmTrajectory::Trajectory::datapoints)
      .def_readwrite("datatypes", &LcmTrajectory::Trajectory::datatypes);

  py::class_<lcmt_saved_traj>(m, "lcmt_saved_traj")
      .def(py::init<>())
      .def_readwrite("metadata", &lcmt_saved_traj::metadata)
      .def_readwrite("num_trajectories", &lcmt_saved_traj::num_trajectories)
      .def_readwrite("trajectories", &lcmt_saved_traj::trajectories)
      .def_readwrite("trajectory_names", &lcmt_saved_traj::trajectory_names);

  py::class_<LcmTrajectory>(m, "LcmTrajectory")
      .def(py::init<>())
      .def(py::init<const std::string&>())
      .def(py::init<const lcmt_saved_traj&>())
      .def("LoadFromFile", &LcmTrajectory::LoadFromFile,
           py::arg("trajectory_name"))
      .def("WriteToFile", &LcmTrajectory::WriteToFile,
           py::arg("trajectory_name"))
      .def("GetTrajectoryNames", &LcmTrajectory::GetTrajectoryNames)
      .def("GetMetadata", &LcmTrajectory::GetMetadata)
      .def("AddTrajectory", &DirconTrajectory::AddTrajectory)
      .def("GenerateLcmObject", &LcmTrajectory::GenerateLcmObject)
      .def("GetTrajectory", &LcmTrajectory::GetTrajectory,
           py::arg("trajectory_name"));
  py::class_<DirconTrajectory>(m, "DirconTrajectory")
      .def(py::init<const std::string&>())
      .def("WriteToFile", &LcmTrajectory::WriteToFile,
           py::arg("trajectory_name"))
      .def("GetMetadata", &LcmTrajectory::GetMetadata)
      .def("GetTrajectoryNames", &LcmTrajectory::GetTrajectoryNames)
      .def("AddTrajectory", &DirconTrajectory::AddTrajectory)
      .def("GetTrajectory", &LcmTrajectory::GetTrajectory,
           py::arg("trajectory_name"))
      .def("GetStateSamples", &DirconTrajectory::GetStateSamples)
      .def("GetStateDerivativeSamples",
           &DirconTrajectory::GetStateDerivativeSamples)
      .def("GetStateBreaks", &DirconTrajectory::GetStateBreaks)
      .def("GetInputSamples", &DirconTrajectory::GetInputSamples)
      .def("GetBreaks", &DirconTrajectory::GetBreaks)
      .def("GetForceSamples", &DirconTrajectory::GetForceSamples)
      .def("GetForceBreaks", &DirconTrajectory::GetForceBreaks)
      .def("GetCollocationForceSamples",
           &DirconTrajectory::GetCollocationForceSamples)
      .def("GetCollocationForceBreaks",
           &DirconTrajectory::GetCollocationForceBreaks)
      .def("GetDecisionVariables", &DirconTrajectory::GetDecisionVariables)
      .def("GetNumModes", &DirconTrajectory::GetNumModes)
      .def("ReconstructStateTrajectory",
           &DirconTrajectory::ReconstructStateTrajectory)
      .def("ReconstructInputTrajectory",
           &DirconTrajectory::ReconstructInputTrajectory)
      .def("ReconstructLambdaTrajectory",
           &DirconTrajectory::ReconstructLambdaTrajectory)
      .def("ReconstructLambdaCTrajectory",
           &DirconTrajectory::ReconstructLambdaCTrajectory)
      .def("ReconstructGammaCTrajectory",
           &DirconTrajectory::ReconstructGammaCTrajectory);
  py::class_<RomPlannerTrajectory>(m, "RomPlannerTrajectory")
      .def(py::init<const std::string&>())
      .def("GetMetadata", &LcmTrajectory::GetMetadata)
      .def("GetTrajectoryNames", &LcmTrajectory::GetTrajectoryNames)
      .def("GetTrajectory", &LcmTrajectory::GetTrajectory,
           py::arg("trajectory_name"))
      .def("GetStateSamples", &RomPlannerTrajectory::GetStateSamples)
      .def("GetStateDerivativeSamples",
           &RomPlannerTrajectory::GetStateDerivativeSamples)
      .def("GetStateBreaks", &RomPlannerTrajectory::GetStateBreaks)
      .def("GetInputSamples", &RomPlannerTrajectory::GetInputSamples)
      .def("GetBreaks", &RomPlannerTrajectory::GetBreaks)
      .def("GetDecisionVariables", &RomPlannerTrajectory::GetDecisionVariables)
      .def("GetNumModes", &RomPlannerTrajectory::GetNumModes)
      .def("ReconstructStateTrajectory",
           &RomPlannerTrajectory::ReconstructStateTrajectory)
      .def("ReconstructInputTrajectory",
           &RomPlannerTrajectory::ReconstructInputTrajectory)
      .def("get_x0",
           &RomPlannerTrajectory::get_x0)
      .def("get_x0_time",
           &RomPlannerTrajectory::get_x0_time)
      .def("get_xf",
           &RomPlannerTrajectory::get_xf)
      .def("get_xf_time",
           &RomPlannerTrajectory::get_xf_time)
      .def("get_stance_foot",
           &RomPlannerTrajectory::get_stance_foot);
  py::class_<HybridRomPlannerTrajectory>(m, "HybridRomPlannerTrajectory")
      .def(py::init<const std::string&>())
      .def(py::init<const std::string&, bool>())
      .def("GetMetadata", &LcmTrajectory::GetMetadata)
      .def("GetTrajectoryNames", &LcmTrajectory::GetTrajectoryNames)
      .def("GetTrajectory", &LcmTrajectory::GetTrajectory,
           py::arg("trajectory_name"))
      .def("GetStateSamples", &HybridRomPlannerTrajectory::GetStateSamples)
      .def("GetStateDerivativeSamples",
           &HybridRomPlannerTrajectory::GetStateDerivativeSamples)
      .def("GetStateBreaks", &HybridRomPlannerTrajectory::GetStateBreaks)
      .def("GetInputSamples", &HybridRomPlannerTrajectory::GetInputSamples)
      .def("GetBreaks", &HybridRomPlannerTrajectory::GetBreaks)
      .def("GetDecisionVariables", &HybridRomPlannerTrajectory::GetDecisionVariables)
      .def("GetNumModes", &HybridRomPlannerTrajectory::GetNumModes)
      .def("ReconstructStateTrajectory",
           &HybridRomPlannerTrajectory::ReconstructStateTrajectory)
      .def("ReconstructInputTrajectory",
           &HybridRomPlannerTrajectory::ReconstructInputTrajectory)
      .def("get_global_feet_pos",
           &HybridRomPlannerTrajectory::get_global_feet_pos)
      .def("get_global_feet_pos_time",
           &HybridRomPlannerTrajectory::get_global_feet_pos_time)
      .def("get_global_com_pos",
           &HybridRomPlannerTrajectory::get_global_com_pos)
      .def("get_global_com_pos_time",
           &HybridRomPlannerTrajectory::get_global_com_pos_time)
      .def("get_current_quat_xyz_shift",
           &HybridRomPlannerTrajectory::get_current_quat_xyz_shift)
      .def("get_stance_foot",
           &HybridRomPlannerTrajectory::get_stance_foot);
}

}  // namespace pydairlib
}  // namespace dairlib
