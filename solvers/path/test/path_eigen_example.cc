#include "solvers/path/path_eigen.h"
#include <iostream>
#include <chrono>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std::chrono;

int main () {
	srand(time(0));

	// Run N different random (M PSD) examples of dimension n
	auto start = high_resolution_clock::now();
	int N = 100;
	int n = 50;
	
	for (int i = 0; i < N; i++) {
		MatrixXd M = MatrixXd::Random(n, n);
		M = M*M.transpose();

		VectorXd q = VectorXd::Random(n);

		VectorXd z = VectorXd::Zero(n);

		int result = SolveLCP(&M, &q, &z);
		
		if (result != 1)
			std::cout << result << std::endl;
	}
	auto stop = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(stop - start);
  

	std::cout << "For LCP dimension " << n << " average time of ";
	std::cout << duration.count()/N << " us." << std::endl;

	return 0;
}	