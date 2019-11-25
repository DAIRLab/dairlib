#include <fstream>
#include <string>

#include "drake/common/polynomial.h"

class LoadLyapunovPolynomial {

  public:
    LoadLyapunovPolynomial(std::string file_M_name, std::string file_p_name);

    void load(std::vector<Polynomiald>& x, std::vector<Polynomiald>& V);

  private:
    std::ifstream file_M_;
    std::ifstream file_p_;

    std::map<int, std::map<int, double>> M_;
    std::map<int, std::map<int, int>> p_;

};
