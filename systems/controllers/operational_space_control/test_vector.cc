#include <iostream>
#include <vector>
#include <memory>


int main() {

  int a = 1;
  int b = 2;
  int c = 3;

  // std::vector<int*> initial = {};
  // std::vector<int*>* int_vec = &initial;

  auto int_vec = std::make_unique<std::vector<int*>>();

  int_vec->push_back(&a);
  int_vec->push_back(&b);
  int_vec->push_back(&c);

  std::cout << int_vec->at(0) << std::endl;

  // for (auto member : *int_vec) {
  //   std::cout << *member << std::endl;
  // }

  // for (int i = 0; i < int_vec->size(); i++) {
  //   int* member = int_vec->at(i);
  //   std::cout << *member << std::endl;
  // }

  // std::cout << int_vec->at(0) << std::endl;
  // std::cout << a << std::endl;
  // std::cout << a << std::endl;

  return 0;
}

