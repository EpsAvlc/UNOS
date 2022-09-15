#ifndef UNOS_PARAMETER_BLOCK_H__
#define UNOS_PARAMETER_BLOCK_H__

#include <memory>
#include <unos/manifold/sub_manifold.hh>

namespace unos {
class ParameterBlock {
 public:
  using Ptr = std::shared_ptr<ParameterBlock>;
  ParameterBlock(double* data, int size, SubManifold* sub = nullptr) {
    data_ = data;
    size_ = size;
  }
  static Ptr create(double* data, int size, SubManifold* sub = nullptr) {
    return std::make_shared<ParameterBlock>(data, size, sub);
  };

  int size() { return size_; }

  double* mutableData() { return data_; }

  void getState(double* state) { memcpy(state, data_, size_ * sizeof(data_)); }

  double const* state() const { return data_; }

 private:
  double* data_;
  int     size_;
};

};      // namespace unos
#endif  // PARAMETER_BLOCKS_HH
