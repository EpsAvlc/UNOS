#ifndef UNOS_PROBLEM_RESIDUAL_BLOCK_HH
#define UNOS_PROBLEM_RESIDUAL_BLOCK_HH

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
                std::vector<ParameterBlock::Ptr> param_blocks);

  void evaluate(double* residual, double** jacobian);

  int size();

  const std::vector<ParameterBlock::Ptr>& parameterBlocks() const;

  int numParameterBlocks();

  static ResidualBlock::Ptr create(
      const CostFunction* cost_function, const LossFunction* loss_function,
      std::vector<ParameterBlock::Ptr> param_blocks);

  int jacobianOffset() const;

  void setJacobianOffset(const int jacobian_offset);

 private:
  std::vector<ParameterBlock::Ptr> parameter_blocks_;
  int                              size_;
  const CostFunction*              cost_function_;
  int                              jacobian_offset_;
};

};  // namespace unos

#endif  // UNOS_PROBLEM_RESIDUAL_BLOCK_HH
