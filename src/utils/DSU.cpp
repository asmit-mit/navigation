#include "utils/DSU.h"

namespace utils {

void DSU::init(size_t n) {
  parent.resize(n);
  size.assign(n, 1);
  for (size_t i = 0; i < n; ++i)
    parent[i] = i;
}

size_t DSU::find(size_t x) {
  size_t root = x;

  while (root != parent[root])
    root = parent[root];

  while (x != root) {
    int next = parent[x];
    parent[x] = root;
    x = next;
  }

  return root;
}

bool DSU::unite(size_t a, size_t b) {
  size_t ra = find(a);
  size_t rb = find(b);

  if (ra == rb)
    return false;

  if (size[ra] < size[rb])
    std::swap(ra, rb);

  parent[rb] = ra;
  size[ra] += size[rb];

  return true;
}

} // namespace utils
