#include "utils/DSU.h"

namespace utils {

void DSU::init(int n) {
  parent.resize(n);
  size.assign(n, 1);
  for (int i = 0; i < n; ++i)
    parent[i] = i;
}

int DSU::find(int x) {
  int root = x;

  while (root != parent[root])
    root = parent[root];

  while (x != root) {
    int next = parent[x];
    parent[x] = root;
    x = next;
  }

  return root;
}

bool DSU::unite(int a, int b) {
  int ra = find(a);
  int rb = find(b);

  if (ra == rb)
    return false;

  if (size[ra] < size[rb])
    std::swap(ra, rb);

  parent[rb] = ra;
  size[ra] += size[rb];

  return true;
}

} // namespace utils
