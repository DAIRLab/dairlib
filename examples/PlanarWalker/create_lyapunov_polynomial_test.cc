#include <iostream>
#include "examples/PlanarWalker/create_lyapunov_polynomial.h"

int main() {
  LoadLyapunovPolynomial lyap_poly_loader("examples/PlanarWalker/csv/V_M.csv",
                                          "examples/PlanarWalker/csv/V_p.csv");

  std::vector<Polynomiald> x(3, 0);
  std::vector<Polynomiald> V(9, 0);
  lyap_poly_loader.load(x, V);

  for(int i=0; i<3; i++) std::cout << x[i] << std::endl;
  for(int i=0; i<9; i++) std::cout << V[i] << std::endl;
}
