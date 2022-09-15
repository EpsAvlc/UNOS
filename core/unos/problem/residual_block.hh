#ifndef UNOS_RESIDUAL_BLOCK_H__
#define UNOS_RESIDUAL_BLOCK_H__

#include <memory>
#include "unos/cost_function/cost_function.hh"
#include "unos/loss_function/loss_function.hh"
#include "unos/problem/parameter_block.hh"

namespace unos {
class ResidualBlock {
 public:
  using Ptr = std::shared_ptr<ResidualBlock>;
  ResidualBlock(const CostFunction*              cost_function,
                const LossFunction*              loss_function,
                std::vector<ParameterBlock::Ptr> param_blocks)
      : cost_function_(cost_function), parameter_blocks_(param_blocks) {
    residual_size_ = cost_function->getResidualSize();
  }

  void evaluate(double* residual, double** jacobian) {
    // fix paramter block size up to 10. Since it is rare to have move than 10
    // blocks;
    std::array<double const*, 10> parameters;
    for (size_t pi = 0; pi < parameter_blocks_.size(); ++pi) {
      parameters[pi] = parameter_blocks_[pi]->state();
    }
    cost_function_->evaluate(parameters.data(), residual, jacobian);
  }

  int residualSize() { return residual_size_; }

  std::vector<ParameterBlock::Ptr> parameterBlocks() {
    return parameter_blocks_;
  }

  int numParameterBlocks() { return parameter_blocks_.size(); }

  static Ptr create(const CostFunction* cost_function,
                    const LossFunction* loss_function,
                    std::vector<int>    param_block_ids) {
    return std::make_shared<ResidualBlock>(cost_function, loss_function,
                                           param_block_ids);
  }

 private:
  std::vector<ParameterBlock::Ptr> parameter_blocks_;
  int                              residual_size_;
  const CostFunction*              cost_function_;
};

};  // namespace unos

#endif  // RESIDUAL_BLOCK_HH
