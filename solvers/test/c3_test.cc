#include "solvers/c3_miqp.h"

namespace dairlib {
namespace solvers {

int DoMain(int argc, char* argv[]) {
	/// This just moves your test code into the dairlib::solvers namespace
	/// Makes life a bit easier
	
	// You can use this function as a place to test out the different methods
	// in C3
	// Make sure you build a C3MIQP (C3 is virtual, so you can't actually build it)
	return 0;
}

}
}

int main(int argc, char* argv[]) {
	return dairlib::solvers::DoMain(argc, argv);
}