#include <utility>

#include "meshcat_mpc_debug_visualizer.h"
#include "systems/framework/output_vector.h"
#include "drake/geometry/rgba.h"


namespace dairlib::perceptive_locomotion {
using geometry::ConvexFoothold;
using geometry::ConvexFootholdSet;
using Eigen::Matrix3d;
using Eigen::Matrix3Xd;
using systems::OutputVector;

MeshcatMPCDebugVisualizer::MeshcatMPCDebugVisualizer(
    std::shared_ptr<drake::geometry::Meshcat> meshcat,
    const drake::multibody::MultibodyPlant<double>& plant) :
    meshcat_(std::move(meshcat)) {

  mpc_debug_input_port_ = DeclareAbstractInputPort(
      "mpc_debug", drake::Value<lcmt_mpc_debug>()
    ).get_index();

  state_input_port_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(
          plant.num_positions(), plant.num_velocities(), plant.num_actuators())
      ).get_index();

  n_footholds_idx_ = DeclareDiscreteState(1);

  DeclarePerStepUnrestrictedUpdateEvent(
      &MeshcatMPCDebugVisualizer::UnrestrictedUpdate);
}

void MeshcatMPCDebugVisualizer::DrawComTrajSolution(
    const std::string &path,
    const dairlib::lcmt_mpc_solution &com_traj_solution,
    const Matrix3d& R_yaw, const double z_com) const {

  std::vector<drake::geometry::Rgba> rgb = {
      drake::geometry::Rgba(0.003921569, 0.121568627, 0.356862745, 0.6),
      drake::geometry::Rgba(.6, 0, 0, 0.5)
  };
  for (int n = 0; n < com_traj_solution.nm; n++) {
    Matrix3Xd segment = Matrix3Xd::Zero(3, com_traj_solution.nk);

    Eigen::Vector3d pn = Eigen::Vector3d::Map(com_traj_solution.pp.at(n).data());
    Eigen::Vector3d pnp1 = (n == (com_traj_solution.nm - 1)) ?
        pn : Eigen::Vector3d::Map(com_traj_solution.pp.at(n+1).data());
    for (int k = 0; k < com_traj_solution.nk; k++) {
      Eigen::Vector3d com_k = Eigen::Vector3d::Zero();
      com_k.head<2>() = Eigen::Vector4d::Map(com_traj_solution.xx.at(n).at(k).data()).head<2>() + pn.head<2>();
      com_k(2) = z_com + pn(2) + (pnp1(2) - pn(2)) * (static_cast<double>(k) / (com_traj_solution.nk_minus_one));
      segment.col(k) = R_yaw * com_k;
    }
    std::string label = path + "_" + std::to_string(n);
    meshcat_->SetLine(label, segment, 0.5, rgb.at(n % 2));
  }
}

void MeshcatMPCDebugVisualizer::DrawFootholds(ConvexFootholdSet& foothold_set,
                                              int n_prev) const {
  std::vector<drake::geometry::Rgba> rgb = {
      drake::geometry::Rgba(1, 0, 0, 0.5),
      drake::geometry::Rgba(0, 1, 0, 0.5),
      drake::geometry::Rgba(0, 0, 1, 0.5)
  };
  for (int i = 0; i < foothold_set.size(); i++) {
    auto foothold = foothold_set.footholds().at(i);
    const auto [verts, faces] = foothold.GetSurfaceMesh();
    auto faces_reversed = faces;
    faces_reversed.row(0).swap(faces_reversed.row(2));
    meshcat_->SetTriangleMesh(make_path(i) + "top", verts, faces, rgb.at(i % 3));
    meshcat_->SetTriangleMesh(make_path(i) + "bottom", verts, faces_reversed, rgb.at(i % 3));
  }
  for (int i = foothold_set.size(); i < n_prev; i++) {
    meshcat_->Delete(make_path(i) + "top");
    meshcat_->Delete(make_path(i) + "bottom");
  }
}

Eigen::Matrix3d MeshcatMPCDebugVisualizer::R_WB(
    const Eigen::Vector4d& wxyz) {
  if (wxyz.norm() < 0.98 or wxyz.norm() > 1.02) {
    return Matrix3d::Identity();
  }
  Eigen::Vector4d xyzw = Eigen::Vector4d::Zero();
  xyzw.head<3>() = wxyz.tail<3>();
  xyzw(3) = wxyz(0);
  Eigen::Vector3d base_x = Eigen::Quaterniond(xyzw).toRotationMatrix().col(0);
  return drake::math::RotationMatrixd::MakeZRotation(
      atan2(base_x(1), base_x(0))
  ).matrix();
}

drake::systems::EventStatus MeshcatMPCDebugVisualizer::UnrestrictedUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::State<double> *state) const {

  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  Eigen::Vector4d quat = robot_output->GetState().head<4>();
  const Eigen::Matrix3d R_yaw = R_WB(quat);

  const auto& mpc_debug = EvalAbstractInput(
      context, mpc_debug_input_port_)->get_value<lcmt_mpc_debug>();
  auto foothold_set = ConvexFootholdSet::CopyFromLcm(mpc_debug.footholds);

  DrawComTrajSolution("com_sol", mpc_debug.solution, R_yaw, 0.83);

  int n_prev = state->get_discrete_state(n_footholds_idx_).get_value()(0);
  DrawFootholds(foothold_set, n_prev);
  state->get_mutable_discrete_state(n_footholds_idx_).set_value(
      Eigen::VectorXd::Constant(1, foothold_set.size()));


  return drake::systems::EventStatus::Succeeded();
}

}