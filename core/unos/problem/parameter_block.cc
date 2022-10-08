#include "unos/problem/parameter_block.hh"
#include <memory.h>

namespace unos {

ParameterBlock::ParameterBlock(double* data, int size, ManifoldBase* manifold) {
  data_     = data;
  size_     = size;
  manifold_ = manifold;
}

ParameterBlock::Ptr ParameterBlock::create(double* data, int size,
                                           ManifoldBase* manifold) {
  return std::make_shared<ParameterBlock>(data, size, manifold);
};

double* ParameterBlock::mutableData() { return data_; }

void ParameterBlock::getState(double* state) {
  memcpy(state, data_, size_ * sizeof(data_));
}

void ParameterBlock::setState(double const* state) {
  memcpy(data_, state, size_ * sizeof(data_));
}

double const* ParameterBlock::state() const { return data_; }

int ParameterBlock::jacobianOffset() const { return jacobian_offset_; }

void ParameterBlock::setJacobianOffset(const int jacobian_offset) {
  jacobian_offset_ = jacobian_offset;
}
}  // namespace unos
