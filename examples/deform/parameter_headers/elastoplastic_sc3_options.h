#pragma once

#include <drake/common/yaml/yaml_io.h>

#include "c3/multibody/elastoplastic_lcs_factory.h"
#include "c3/multibody/elastoplastic_lcs_factory_options.h"
#include "common/file_utils.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"

#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_read_archive.h"

using c3::ElastoPlasticContactPairConfig;
using c3::ElastoPlasticLCSFactoryOptions;
using c3::multibody::DeformationModel;
using c3::multibody::GetDeformationModelMapToString;

struct ElastoPlasticSC3Options : SamplingC3Options {
  DeformationModel deformation_model;

  // Only applicable for internal contacts.
  int num_internal_contacts_index;
  int num_internal_contacts_index_for_cost;
  std::vector<std::vector<double>> g_internal_slack_list;
  std::vector<std::vector<double>> g_internal_sigma_list;
  std::vector<std::vector<double>> u_internal_slack_list;
  std::vector<std::vector<double>> u_internal_sigma_list;
  std::vector<std::vector<double>> g_eta_internal_slack_list;
  std::vector<std::vector<double>> g_eta_internal_sigma_list;
  std::vector<std::vector<double>> u_eta_internal_slack_list;
  std::vector<std::vector<double>> u_eta_internal_sigma_list;

  int num_internal_contacts;  // resolved from num_internal_contacts_index and
                              // g_internal_slack_list.size()
  ElastoPlasticLCSFactoryOptions ep_lcs_factory_options;

  // Serialize.
  template <typename Archive>
  void Serialize(Archive* a) {
    SamplingC3Options::Serialize(a);
    ENUM_DESERIALIZE(a, deformation_model);
    a->Visit(DRAKE_NVP(num_internal_contacts_index));
    a->Visit(DRAKE_NVP(num_internal_contacts_index_for_cost));
    a->Visit(DRAKE_NVP(g_internal_slack_list));
    a->Visit(DRAKE_NVP(g_internal_sigma_list));
    a->Visit(DRAKE_NVP(u_internal_slack_list));
    a->Visit(DRAKE_NVP(u_internal_sigma_list));
    a->Visit(DRAKE_NVP(g_eta_internal_slack_list));
    a->Visit(DRAKE_NVP(g_eta_internal_sigma_list));
    a->Visit(DRAKE_NVP(u_eta_internal_slack_list));
    a->Visit(DRAKE_NVP(u_eta_internal_sigma_list));

    num_internal_contacts =
        g_internal_slack_list[num_internal_contacts_index].size();

    SetCommonOptions(&c3_options_pose, &lcs_factory_options_pose);
    SetElastoPlasticLCSFactoryOptions(c3_options_pose, lcs_factory_options_pose,
                                      &ep_lcs_factory_options);
  }

  /**
   * NOTES:
   *
   * For elastoplastic systems, there is no difference between position
   * and pose tracking mode since the system state is represented by 3D node
   * positions.  Thus, C3Options and LCSFactory options will always use the
   * pose parameters, and the position parameters are unused.
   *
   * There are getters that take in an unused boolean input to mimic the parent
   * class's inputs, but they will always return the pose tracking options and
   * will give a warning if position tracking options are requested.  There are
   * also getters without any inputs, and these are the preferred getters to
   * avoid confusion.
   */
  C3Options GetC3Options(const bool& is_pose_tracking) const override {
    if (!is_pose_tracking) {
      drake::log()->warn(
          "Position-tracking C3 options requested for elastoplastic system, but"
          " only pose-tracking options are defined.  Returning pose-tracking "
          "C3 options.");
    }
    return c3_options_pose;
  }
  C3Options GetC3Options() const { return c3_options_pose; }
  LCSFactoryOptions GetLCSFactoryOptions(
      const bool& is_pose_tracking) const override {
    if (!is_pose_tracking) {
      drake::log()->warn(
          "Position-tracking C3 options requested for elastoplastic system, but"
          " only pose-tracking options are defined.  Returning pose-tracking "
          "C3 options.");
    }
    return lcs_factory_options_pose;
  }
  LCSFactoryOptions GetLCSFactoryOptions() const {
    return lcs_factory_options_pose;
  }
  ElastoPlasticLCSFactoryOptions GetElastoPlasticLCSFactoryOptions() const {
    return ep_lcs_factory_options;
  }

 private:
  void PopulateCostMatricesFromVectors(C3Options* options) const override {
    std::vector<double> g_vector = std::vector<double>();
    g_vector.insert(g_vector.end(), options->g_x.begin(), options->g_x.end());
    if (contact_model == "stewart_and_trinkle") {
      g_vector.insert(g_vector.end(), options->g_gamma.begin(),
                      options->g_gamma.end());
      g_vector.insert(g_vector.end(), options->g_lambda_n.begin(),
                      options->g_lambda_n.end());
      g_vector.insert(g_vector.end(), options->g_lambda_t.begin(),
                      options->g_lambda_t.end());
    } else {
      g_vector.insert(g_vector.end(), options->g_lambda.begin(),
                      options->g_lambda.end());
    }
    // vv Internal contact terms added after external contact terms vv //
    g_vector.insert(g_vector.end(), options->g_internal_slack->begin(),
                    options->g_internal_slack->end());
    g_vector.insert(g_vector.end(), options->g_internal_sigma->begin(),
                    options->g_internal_sigma->end());
    // ^^ Internal contact terms added after external contact terms ^^ //
    g_vector.insert(g_vector.end(), options->g_u.begin(), options->g_u.end());

    if (projection_type == "C3+") {
      if (contact_model == "stewart_and_trinkle") {
        g_vector.insert(g_vector.end(), options->g_eta_slack->begin(),
                        options->g_eta_slack->end());
        g_vector.insert(g_vector.end(), options->g_eta_n->begin(),
                        options->g_eta_n->end());
        g_vector.insert(g_vector.end(), options->g_eta_t->begin(),
                        options->g_eta_t->end());
      } else {
        g_vector.insert(g_vector.end(), options->g_eta->begin(),
                        options->g_eta->end());
      }
      // vv Internal contact terms added after external contact terms vv //
      g_vector.insert(g_vector.end(), options->g_eta_internal_slack->begin(),
                      options->g_eta_internal_slack->end());
      g_vector.insert(g_vector.end(), options->g_eta_internal_sigma->begin(),
                      options->g_eta_internal_sigma->end());
      // ^^ Internal contact terms added after external contact terms ^^ //
    }

    std::vector<double> u_vector = std::vector<double>();
    u_vector.insert(u_vector.end(), options->u_x.begin(), options->u_x.end());
    if (contact_model == "stewart_and_trinkle") {
      u_vector.insert(u_vector.end(), options->u_gamma.begin(),
                      options->u_gamma.end());
      u_vector.insert(u_vector.end(), options->u_lambda_n.begin(),
                      options->u_lambda_n.end());
      u_vector.insert(u_vector.end(), options->u_lambda_t.begin(),
                      options->u_lambda_t.end());
    } else {
      u_vector.insert(u_vector.end(), options->u_lambda.begin(),
                      options->u_lambda.end());
    }
    // vv Internal contact terms added after external contact terms vv //
    u_vector.insert(u_vector.end(), options->u_internal_slack->begin(),
                    options->u_internal_slack->end());
    u_vector.insert(u_vector.end(), options->u_internal_sigma->begin(),
                    options->u_internal_sigma->end());
    // ^^ Internal contact terms added after external contact terms ^^ //
    u_vector.insert(u_vector.end(), options->u_u.begin(), options->u_u.end());

    if (projection_type == "C3+") {
      if (contact_model == "stewart_and_trinkle") {
        u_vector.insert(u_vector.end(), options->u_eta_slack->begin(),
                        options->u_eta_slack->end());
        u_vector.insert(u_vector.end(), options->u_eta_n->begin(),
                        options->u_eta_n->end());
        u_vector.insert(u_vector.end(), options->u_eta_t->begin(),
                        options->u_eta_t->end());
      } else {
        u_vector.insert(u_vector.end(), options->u_eta->begin(),
                        options->u_eta->end());
      }
      // vv Internal contact terms added after external contact terms vv //
      u_vector.insert(u_vector.end(), options->u_eta_internal_slack->begin(),
                      options->u_eta_internal_slack->end());
      u_vector.insert(u_vector.end(), options->u_eta_internal_sigma->begin(),
                      options->u_eta_internal_sigma->end());
      // ^^ Internal contact terms added after external contact terms ^^ //
    }

    Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(
        options->q_vector.data(), options->q_vector.size());
    Eigen::VectorXd r = Eigen::Map<const Eigen::VectorXd>(
        options->r_vector.data(), options->r_vector.size());
    Eigen::VectorXd g =
        Eigen::Map<const Eigen::VectorXd>(g_vector.data(), g_vector.size());
    Eigen::VectorXd u =
        Eigen::Map<const Eigen::VectorXd>(u_vector.data(), u_vector.size());

    options->Q = options->w_Q * q.asDiagonal();
    options->R = options->w_R * r.asDiagonal();
    options->G = options->w_G * g.asDiagonal();
    options->U = options->w_U * u.asDiagonal();
  }

  void SetCommonOptions(C3Options* c3_options,
                        LCSFactoryOptions* lcs_factory_options) const override {
    // Set the common options.
    SamplingC3Options::SetCommonOptions(c3_options, lcs_factory_options);

    // Set some elastoplastic-specific options.
    c3_options->g_internal_slack =
        g_internal_slack_list[num_internal_contacts_index];
    c3_options->g_internal_sigma =
        g_internal_sigma_list[num_internal_contacts_index];
    c3_options->u_internal_slack =
        u_internal_slack_list[num_internal_contacts_index];
    c3_options->u_internal_sigma =
        u_internal_sigma_list[num_internal_contacts_index];
    if (projection_type == "C3+") {
      c3_options->g_eta_internal_slack =
          g_eta_internal_slack_list[num_internal_contacts_index];
      c3_options->g_eta_internal_sigma =
          g_eta_internal_sigma_list[num_internal_contacts_index];
      c3_options->u_eta_internal_slack =
          u_eta_internal_slack_list[num_internal_contacts_index];
      c3_options->u_eta_internal_sigma =
          u_eta_internal_sigma_list[num_internal_contacts_index];
    }

    // Set the pose-specific options.
    SamplingC3Options::SetPoseTrackingOptions(c3_options, lcs_factory_options);
  }

  void SetElastoPlasticLCSFactoryOptions(
      const C3Options& c3_options, const LCSFactoryOptions& lcs_factory_options,
      ElastoPlasticLCSFactoryOptions* ep_lcs_factory_options) const {
    // Set the options inherited from LCSFactoryOptions.
    ep_lcs_factory_options->SetLCSFactoryOptionsFromBase(lcs_factory_options);

    // Set elastoplastic-specific options.
    ep_lcs_factory_options->deformation_model =
        GetDeformationModelMapToString().at(deformation_model);
    ep_lcs_factory_options->num_internal_contacts =
        c3_options.g_internal_slack->size();
  }
};
