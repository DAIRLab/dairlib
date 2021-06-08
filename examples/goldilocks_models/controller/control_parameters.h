#pragma once

namespace dairlib {
namespace goldilocks_models {

// Filepath containing gains
const std::string GAINS_FILENAME =
    "examples/goldilocks_models/rom_walking_gains.yaml";

const int LEFT_STANCE = 0;
const int RIGHT_STANCE = 1;
const int DOUBLE_STANCE = 2;
const int POST_LEFT_DOUBLE_STANCE = 3;
const int POST_RIGHT_DOUBLE_STANCE = 4;

}  // namespace goldilocks_models
}  // namespace dairlib
