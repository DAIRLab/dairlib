#include <cmath>
#include <algorithm>
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

namespace {

size_t calc_path_length_if_larger_than_lower_bound(
    const std::vector<double>& a, const std::vector<double>& b,
    const std::pair<int,int>& starting_point, double tol,
    size_t lower_bound) {

  // matrix has a down the side and b across the top
  size_t n = a.size();
  size_t m = b.size();
  size_t i = starting_point.first;
  size_t j = starting_point.second;

  size_t path_len = 0;
  double target_val = a[i] - b[j];

  while (i < n and j < m) {

    double diff = target_val - (a[i] - b[j]);
    if (abs(diff) < tol) {
      ++path_len;
      ++i;
      ++j;
    }
    else if (diff > 0) {
      ++i;
    } else {
      ++j;
    }

    if (path_len + n - i < lower_bound and path_len + m - j < lower_bound) {
      // impossible for the total path length to be larger than the existing
      // lower bound, just return what we have
      return path_len;
    }
  }
  return path_len;
}

}

PYBIND11_MODULE(file_utils, m) {
  m.doc() = "Binding data analysis utilities for improved"
        "speed (and, honestly, for fun)";

  m.def("line_up_timestamps",
        [](std::vector<double> a, std::vector<double> b, double tol) {
    std::sort(a.begin(), a.end());
    std::sort(b.begin(), b.end());

    size_t n = a.size();
    size_t m = b.size();

    size_t longest_path_len = 0;
    int start_i = 0;
    int start_j = 0;

    size_t max_path_len = std::min(n, m);


    std::pair<int, int> test_idx = {0, 0};
    for (int l = 0; l < max_path_len and max_path_len - l > longest_path_len; ++l) {

      for (int j = l; j < m and m - j > longest_path_len; ++j) {
        test_idx.first = l;
        test_idx.second = j;
        size_t path_len = calc_path_length_if_larger_than_lower_bound(
            a, b, test_idx, tol, longest_path_len
        );
        if (path_len > longest_path_len) {
          longest_path_len = path_len;
          start_j = j;
          start_i = l;
        }
      }

      for (int i = l + 1; i < n and n - i > longest_path_len; ++i) {
        test_idx.first = i;
        test_idx.second = l;
        size_t path_len = calc_path_length_if_larger_than_lower_bound(
            a, b, test_idx, tol, longest_path_len
        );
        if (path_len > longest_path_len) {
          longest_path_len = path_len;
          start_i = i;
          start_j = l;
        }
      }
    }

    std::vector<double> final_a;
    std::vector<double> final_b;
    final_a.reserve(longest_path_len);
    final_b.reserve(longest_path_len);

    size_t i = start_i;
    size_t j = start_j;
    double offset = a[i] - b[j];
    while (i < n and j < m) {
      if (abs(offset - (a[i] - b[j])) < tol){
        final_a.push_back(a[i]);
        final_b.push_back(b[j]);
        ++i;
        ++j;
      }
      else if (a[i] - b[j] < offset) {
        ++i;
      } else {
        ++j;
      }
    }
    return py::make_tuple(final_a, final_b);
  }, py::arg("times_a"), py::arg("time_b"), py::arg("tol")=1.0);
}


}  // namespace pydairlib
}  // namespace dairlib