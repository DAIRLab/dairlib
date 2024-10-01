#include "plane_seg_system.h"


int main(int argc, char* argv[]) {
  // Just testing instantiation of the object to make sure planeseg library
  // links correctly
  dairlib::perception::PlaneSegSystem plane_seg_system("elevation");
  return 0;
}