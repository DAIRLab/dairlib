#include "systems/system_utils.h"

#include <fstream>
#include <regex>

namespace dairlib {

void DrawAndSaveDiagramGraph(const drake::systems::Diagram<double>& diagram,
                             std::string path) {
  // Default path
  if (path.empty()) path = "../" + diagram.get_name();

  // Save Graphviz string to a file
  std::ofstream out(path);
  out << diagram.GetGraphvizString();
  out.close();

  // Use dot command to draw the diagram and save it to a file
  std::regex r(" ");
  path = std::regex_replace(path, r, "\\ ");
  std::string cmd = "dot -Tps " + path + " -o " + path + ".ps";
  (void) std::system(cmd.c_str());
}

}  // namespace dairlib
