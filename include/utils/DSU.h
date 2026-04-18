#pragma once

#include <cstddef>
#include <vector>

namespace utils {

class DSU {
private:
  std::vector<size_t> parent;
  std::vector<size_t> size;

public:
  DSU() {}

  void init(size_t n);
  size_t find(size_t x);
  bool unite(size_t a, size_t b);
};

} // namespace utils
