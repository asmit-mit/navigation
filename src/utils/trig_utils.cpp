#include "utils/trig_utils.h"
#include "utils/math_utils.h"

namespace utils {

TrigTable::TrigTable(int precision)
    : size(precision), step(TWO_PI / precision), inv_step(precision / TWO_PI) {
  sin_table.resize(size);
  cos_table.resize(size);

  for (int i = 0; i < size; ++i) {
    double angle = i * step;
    sin_table[i] = std::sin(angle);
    cos_table[i] = std::cos(angle);
  }
}

inline double TrigTable::lerp(double a, double b, double t) const {
  return a + (b - a) * t;
}

double TrigTable::sin(double theta) const {
  theta = M2pi(theta);

  double x = theta * inv_step;
  int i0 = static_cast<int>(x);
  int i1 = (i0 + 1) % size;

  double t = x - i0;

  return lerp(sin_table[i0], sin_table[i1], t);
}

double TrigTable::cos(double theta) const {
  theta = M2pi(theta);

  double x = theta * inv_step;
  int i0 = static_cast<int>(x);
  int i1 = (i0 + 1) % size;

  double t = x - i0;

  return lerp(cos_table[i0], cos_table[i1], t);
}

} // namespace utils
