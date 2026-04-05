#pragma once

#include <vector>

namespace utils {

template <typename T> class GenVector {
public:
  GenVector() = default;

  GenVector(std::size_t size, T default_value)
      : default_value_(default_value), gen_(0) {
    data_.resize(size);
    gens_.resize(size, -1);
  }

  void resize(std::size_t size, T default_value) {
    default_value_ = default_value;
    data_.resize(size);
    gens_.resize(size, -1);
  }

  void reset() {
    ++gen_;
    if (gen_ == 0) {
      std::fill(gens_.begin(), gens_.end(), 0);
      gen_ = 1;
    }
  }

  void set(int idx, const T &value) {
    data_[idx] = value;
    gens_[idx] = gen_;
  }

  T get(int idx) const {
    return (gens_[idx] == gen_) ? data_[idx] : default_value_;
  }

  bool isSet(int idx) const { return gens_[idx] == gen_; }

  void clear(int idx) { gens_[idx] = gen_ - 1; }

  std::size_t size() const { return data_.size(); }

private:
  std::vector<T> data_;
  std::vector<int> gens_;
  T default_value_;
  int gen_ = 1;
};

}; // namespace utils
