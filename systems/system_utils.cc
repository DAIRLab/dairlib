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

  // Use dot command to convert Graphviz string to a pdf
  // The command is `dot -Tpdf input_file -o output_file`
  std::regex r(" ");
  path = std::regex_replace(path, r, "\\ ");
  std::string cmd = "dot -Tpdf " + path + " -o " + path + ".pdf";
  (void) std::system(cmd.c_str());

  // Remove Graphviz string file
  cmd = "rm " + path;
  (void) std::system(cmd.c_str());
}

}  // namespace dairlib
