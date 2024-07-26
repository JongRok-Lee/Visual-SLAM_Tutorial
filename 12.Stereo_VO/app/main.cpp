#include <iostream>
#include <gflags/gflags.h>

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::cout << "Hello, World!" << std::endl;

  return 0;
}