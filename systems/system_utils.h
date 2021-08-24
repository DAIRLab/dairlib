#pragma once

#include <string>
#include "drake/systems/framework/diagram.h"

namespace dairlib {

/// DrawAndSaveDiagramGraph generates a graph for Drake's diagram, and saves it
/// into a .ps file (an image file) which can be opened by Ubuntu's default
/// image viewer.
/// Note that this function requires `dot` program to be installed. To change
/// the visualization setting, visit: http://www.graphviz.org/
/// @param diagram A Drake Diagram
/// @param path The path where the graph is saved. Default location is one
/// layer outside the current directory, and the file name is the diagram's
/// name.
void DrawAndSaveDiagramGraph(const drake::systems::Diagram<double>& diagram,
                             std::string path = "");

}  // namespace dairlib
