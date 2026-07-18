#include "systems/system_utils.h"

#include <fstream>
#include <regex>

namespace dairlib {

void DrawAndSaveDiagramGraph(const drake::systems::Diagram<double>& diagram,
                             std::string path) {
  // Default path
  if (path.empty()) {
    path = "../diagrams/" + diagram.get_name();
  }

  // Create the directory if it does not exist
  std::system("mkdir -p ../diagrams");

  // Save Graphviz string to a file
  std::ofstream out(path);
  out << diagram.GetGraphvizString();
  out.close();

  // Use dot command to convert Graphviz string to a image file
  // The command is `dot -Tps input_file -o output_file`
  std::regex r(" ");
  path = std::regex_replace(path, r, "\\ ");
  std::string cmd = "dot -Tsvg " + path + " -o " + path + ".svg";
  (void)std::system(cmd.c_str());

  // Remove Graphviz string file
  cmd = "rm " + path;
  (void)std::system(cmd.c_str());
}

}  // namespace dairlib
