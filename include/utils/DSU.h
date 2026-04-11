#pragma once

#include <vector>

namespace utils {

class DSU {
private:
  std::vector<int> parent;
  std::vector<int> size;

public:
  DSU() {}

  void init(int n);
  int find(int x);
  bool unite(int a, int b);
};

} // namespace utils
