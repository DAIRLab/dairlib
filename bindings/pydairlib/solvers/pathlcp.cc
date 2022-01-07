#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "solvers/path/path_eigen.h"

namespace py = pybind11;

PYBIND11_MODULE(pathlcp, m)
{
	  m.def("SolveLCP",
	  		[](Eigen::MatrixXd* M, Eigen::VectorXd* q, Eigen::VectorXd* z) {
	  			int status = SolveLCP(M, q, z);

	  			// pybind will convert z to a (n,) numpy array, but we want (n,1)
	  			Eigen::MatrixXd z_mat = *z;
	  			Eigen::Map<Eigen::MatrixXd> z_map(z->data(), q->size(), 1);
	  			return py::make_tuple(status, z_mat);
	  		}
	  		);
	  m.def("SolveLCP",
	  		[](Eigen::MatrixXd* M, Eigen::VectorXd* q) {
	  			Eigen::VectorXd z = Eigen::VectorXd::Zero(q->size());
	  			int status = SolveLCP(M, q, &z);

	  			// pybind will convert z to a (n,) numpy array, but we want (n,1)
	  			Eigen::MatrixXd z_mat = z;
	  			return py::make_tuple(status, z_mat);
	  		}
	  		);
}
