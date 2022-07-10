#ifndef UNOS_RESIDUAL_BLOCK_H__
#define UNOS_RESIDUAL_BLOCK_H__

#include <memory>
#include "unos/cost_function/cost_function.hh"
#include "unos/loss_function/loss_function.hh"

namespace unos {
class ResidualBlock {
 public:
  using Ptr = std::shared_ptr<ResidualBlock>;
  ResidualBlock(const CostFunction* cost_function,
                const LossFunction* loss_function, double* parameters) {
    residual_size_ = cost_function->getResidualSize();
  }
  static Ptr create(const CostFunction* cost_function,
                    const LossFunction* loss_function, double* parameters) {
    return std::make_shared<ResidualBlock>(cost_function, loss_function,
                                           parameters);
  }

 private:
  double* data;
  int residual_size_;
};
};  // namespace unos

#endif  // RESIDUAL_BLOCK_HH
