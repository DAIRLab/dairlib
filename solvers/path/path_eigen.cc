#include "path_eigen.h"

#include <iostream>
#include <signal.h>

#include "cWrapper_Path.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Map;

MatrixXd* M_ = nullptr;
VectorXd* q_ = nullptr;
bool jac_init_ = false;
VectorXd f = VectorXd::Zero(0);
VectorXd lb = VectorXd::Zero(0);
VectorXd ub = VectorXd::Zero(0);

int SolveLCP(MatrixXd* M, VectorXd* q, VectorXd* z) {
	M_ = M;
	q_ = q;
	jac_init_ = false;

	int n = q->size();
	int nnz = n*n;
	int status;

	// Allocate space, if it has changed since previous run
	if (n != f.size()) {
	  f = VectorXd::Zero(n);
	  lb = VectorXd::Zero(n);
		ub = VectorXd::Constant(n, 1e20);
	}

	pathMain(n, nnz, &status, z->data(), f.data(), lb.data(), ub.data());

	if (status == 6) {
		// User interrupt
		raise(SIGINT);
	}

	return status;
}


///
/// Evaluate the function f = M*z + q
///
int funcEval(int n, double *z, double *f) {
	Map<VectorXd> z_map(z, n);
	Map<VectorXd> f_map(f, n);
	f_map = *M_ * z_map + *q_;

	int count = 0;
	for (int i = 0; i < n; i++) {
		if (z[i] < 0)	count++;
	}

	return count;
}
int jacEval(int n, int nnz, double *z, int *col_start, int *col_len,
            int *row, double *data) {


	if (!jac_init_) {
		// This really only needs to be done once
		int l = 0;
		for (int i = 0; i < n; i++) {
			col_start[i] = 1 + i*n;
			col_len[i] = n;
			for (int j = 0; j < n; j++) {
				row[l] = j + 1;
				l++;
			}
		}
		jac_init_ = true;

		// Copy the data
		Map<MatrixXd> M_map(data, n, n);
		M_map = *M_;
	}

	int count = 0;
	for (int i = 0; i < n; i++) {
		if (z[i] < 0)	count++;
	}
	return count;	
}
