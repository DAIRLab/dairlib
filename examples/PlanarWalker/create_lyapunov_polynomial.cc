#include "examples/PlanarWalker/create_lyapunov_polynomial.h"

#include <Eigen/SparseCore>

#include "common/find_resource.h"

// Polynomial is represented as
// p(i) = sum_k M(i, k) prod_j x(j)^p(k, j)

LoadLyapunovPolynomial::LoadLyapunovPolynomial(std::string file_M_name,
                                               std::string file_p_name) {
  file_M_.open(dairlib::FindResourceOrThrow(file_M_name),
              std::ios::binary);
  file_p_.open(dairlib::FindResourceOrThrow(file_p_name),
              std::ios::binary);

}

void LoadLyapunovPolynomial::load(std::vector<Polynomiald>& x, std::vector<Polynomiald>& V) {
  int i_max = INT_MIN;
  int k_max = INT_MIN;
  int j_max = INT_MIN;

  std::string value, line;
  int i, j, k;
  double M_ik;
  while(file_M_.good()) {
    getline(file_M_, line);
    if(line == "") break;

    std::stringstream s(line);

    getline(s, value, ',');
    i = std::stoi(value);
    if(i > i_max) i_max = i;
    getline(s, value, ',');
    k = std::stoi(value);
    if(k > k_max) k_max = k;

    getline(s, value, '\n');
    M_ik = std::stod(value);
    M_[i-1][k-1] = M_ik;
  }
  file_M_.close();

  double p_kj;
  while(file_p_.good()) {
    getline(file_p_, line);
    if(line == "") break;

    std::stringstream s(line);

    getline(s, value, ',');
    k = std::stoi(value);
    if(k > k_max) k_max = k;
    getline(s, value, ',');
    j = std::stod(value);
    if(j > j_max) j_max = j;

    getline(s, value, '\n');
    p_kj = std::stoi(value);
    p_[k-1][j-1] = p_kj;
  }
  file_p_.close();

  for(int i=0; i<x.size(); i++) {
    x[i] = Polynomiald("x", i+1);
  }

  Polynomiald temp = 1;

  for(int i=0; i<i_max; i++) {
    for(auto& M_i : M_[i]) {
      k = M_i.first;
      temp = 1;
      for(auto& p_k : p_[k]) {
        temp *= pow(x[p_k.first], p_k.second);
      }
      V[i] += M_i.second * temp;
    }
  }
}

// int main() {
//   std::map<int, std::map<int, double>> M;
//   std::map<int, std::map<int, int>> p;

//   std::ifstream file_M_(
//       dairlib::FindResourceOrThrow("examples/PlanarWalker/csv/V_M.csv"),
//       std::ios::binary);
//   std::ifstream file_p_(
//       dairlib::FindResourceOrThrow("examples/PlanarWalker/csv/V_p.csv"),
//       std::ios::binary);

//   int i_max = INT_MIN;
//   int k_max = INT_MIN;
//   int j_max = INT_MIN;

//   std::string value, line;
//   int i, j, k;
//   double M_ik;
//   while(file_M_.good()) {
//     getline(file_M_, line);
//     if(line == "") break;

//     std::stringstream s(line);

//     getline(s, value, ',');
//     i = std::stoi(value);
//     if(i > i_max) i_max = i;
//     getline(s, value, ',');
//     k = std::stoi(value);
//     if(k > k_max) k_max = k;

//     getline(s, value, '\n');
//     M_ik = std::stod(value);
//     M[i-1][k-1] = M_ik;
//   }
//   std::cout << i_max << " " << k_max << std::endl;
//   file_M_.close();

//   double p_kj;
//   while(file_p_.good()) {
//     getline(file_p_, line);
//     if(line == "") break;

//     std::stringstream s(line);

//     getline(s, value, ',');
//     k = std::stoi(value);
//     if(k > k_max) k_max = k;
//     getline(s, value, ',');
//     j = std::stod(value);
//     if(j > j_max) j_max = j;

//     getline(s, value, '\n');
//     p_kj = std::stoi(value);
//     p[k-1][j-1] = p_kj;
//   }
//   std::cout << k_max << " " << j_max << std::endl;
//   file_p_.close();

//   Polynomiald x1("x", 1);
//   Polynomiald x2("x", 2);
//   Polynomiald x3("x", 3);
//   std::vector<Polynomiald> x = {x1, x2, x3};

//   Polynomiald V0 = 0;
//   Polynomiald V1 = 0;
//   Polynomiald temp = 1;

//   for(auto& M_i : M[0]) {
//     k = M_i.first;
//     temp = 1;
//     for(auto& p_k : p[k]) {
//       temp *= pow(x[p_k.first], p_k.second);
//     }
//     V0 += M_i.second * temp;
//   }
//   std::cout << V0 << std::endl;

//   for(auto& M_i : M[1]) {
//     k = M_i.first;
//     temp = 1;
//     for(auto& p_k : p[k]) {
//       temp *= pow(x[p_k.first], p_k.second);
//     }
//     V1 += M_i.second * temp;
//   }
//   std::cout << V1 << std::endl;

//   // TODO: Use this to replicate the work done in MATLAB
// }
