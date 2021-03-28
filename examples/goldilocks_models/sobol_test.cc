# include <cmath>
# include <cstdlib>
# include <iomanip>
# include <iostream>

using namespace std;

# include "examples/goldilocks_models/find_models/sobol.hpp"

int main(){
  int dim = 3;
  int num_samples = 10;
  double *sobol_data = i8_sobol_generate(dim, num_samples, 10);
  int num = 0;
  for(int i=0;i<num_samples;i++)
  {
    for (int j=0;j<dim;j++)
    {
      cout<<sobol_data[num+j]<<" ";
    }
    cout<<endl;
    num = num+dim;
  }
}


