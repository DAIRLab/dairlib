#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>

#include "drake/common/yaml/yaml_read_archive.h"

namespace dairlib {

/// Read a CSV formatted file as an Eigen Matrix
Eigen::MatrixXd readCSV(const std::string & path);

/// Write an Eigen Matrix into a CSV formatted file
void writeCSV(const std::string& path, const Eigen::MatrixXd& M);

/// Deserialize a YAML integer into a custom enum type.
template <typename T, typename Archive>
void DeserializeEnum(const char* value_str, Archive* a, T* enum_out) {
    int raw_value = static_cast<int>(*enum_out);
    a->Visit(drake::MakeNameValue(value_str, &raw_value));
    *enum_out = static_cast<T>(raw_value);
}

}  // namespace dairlib

/// Deserializes a YAML integer by the name of the provided e_out variable into
/// the custom enum type of e_out.
#define ENUM_DESERIALIZE(a, e_out) dairlib::DeserializeEnum(#e_out, a, &e_out)
