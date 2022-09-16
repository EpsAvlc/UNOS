#include "unos/problem/parameter_block.hh"
#include <memory.h>

namespace unos {

ParameterBlock::ParameterBlock(double* data, int size /*, SubManifold* sub*/) {
  data_ = data;
  size_ = size;
}

ParameterBlock::Ptr ParameterBlock::create(double* data,
                                           int     size /*, SubManifold sub*/) {
  return std::make_shared<ParameterBlock>(data, size /*, sub*/);
};

int ParameterBlock::size() { return size_; }

double* ParameterBlock::mutableData() { return data_; }

void ParameterBlock::getState(double* state) {
  memcpy(state, data_, size_ * sizeof(data_));
}

double const* ParameterBlock::state() const { return data_; }

int ParameterBlock::jacobianOffset() const { return jacobian_offset_; }

void ParameterBlock::setJacobianOffset(const int jacobian_offset) {
  jacobian_offset_ = jacobian_offset;
}
}  // namespace unos
