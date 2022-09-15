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
                const LossFunction* loss_function, int param_block_id)
      : cost_function_(cost_function), param_block_id_(param_block_id) {
    residual_size_ = cost_function->getResidualSize();
  }

  void evaluate(const double* parameters, double* residual, double** jacobian) {
    cost_function_->evaluate(parameters, residual, jacobian);
  }

  int residualSize() { return residual_size_; }

  int parameterBlockId() { return param_block_id_; }

  static Ptr create(const CostFunction* cost_function,
                    const LossFunction* loss_function, int param_block_id) {
    return std::make_shared<ResidualBlock>(cost_function, loss_function,
                                           param_block_id);
  }

 private:
  int param_block_id_;
  int residual_size_;
  const CostFunction* cost_function_;
};
};  // namespace unos

#endif  // RESIDUAL_BLOCK_HH
