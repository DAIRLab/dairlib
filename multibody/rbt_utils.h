#include "drake/multibody/rigid_body_tree.h"

using std::map;
using std::string;

namespace dairlib {
namespace multibody {
namespace utils {

map<string, int> makeNameToPositionsMap(const RigidBodyTree<double>& tree);
map<string, int> makeNameToVelocitiesMap(const RigidBodyTree<double>& tree);
map<string, int> makeNameToActuatorsMap(const RigidBodyTree<double>& tree);

}
}
}