The files in this directory are adapted from the Open Dynamic Robot Initiative
repository at https://github.com/open-dynamic-robot-initiative/trifinger_simulation. 
Specific changes are:
* The .stl files have been converted to .obj files for use in Drake, with URDFs changed appropriately to use the .obj files.
* Actuators have been added to some of the URDF files.
* The trifinger_minimal_collision.urdf file has been added where only the finger tips have collision objects (spheres), rather than meshes for all finger links.
