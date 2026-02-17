#pragma once

#include <vector>

namespace voronoi {

class DSU {
private:
  std::vector<int> parent;
  std::vector<int> size;

public:
  DSU() {}

  void init(int n);
  int find(int x);
  bool unite(int a, int b);
  bool same(int a, int b);
  int componentSize(int x);
};

} // namespace voronoi
