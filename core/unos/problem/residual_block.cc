#include "residual_block.hh"

namespace unos {

ResidualBlock::ResidualBlock(const CostFunction*              cost_function,
                             const LossFunction*              loss_function,
                             std::vector<ParameterBlock::Ptr> param_blocks)
    : cost_function_(cost_function), parameter_blocks_(param_blocks) {
  size_ = cost_function->getResidualSize();
}

void ResidualBlock::evaluate(double* residual, double** jacobian) {
  // fix paramter block size up to 10. Since it is rare to have move than 10
  // blocks;
  std::array<double const*, 10> parameters;
  for (size_t pi = 0; pi < parameter_blocks_.size(); ++pi) {
    parameters[pi] = parameter_blocks_[pi]->state();
  }
  cost_function_->evaluate(parameters.data(), residual, jacobian);
}

int ResidualBlock::size() { return size_; }

const std::vector<ParameterBlock::Ptr>& ResidualBlock::parameterBlocks() const {
  return parameter_blocks_;
}

int ResidualBlock::numParameterBlocks() { return parameter_blocks_.size(); }

ResidualBlock::Ptr ResidualBlock::create(
    const CostFunction* cost_function, const LossFunction* loss_function,
    std::vector<ParameterBlock::Ptr> param_blocks) {
  return std::make_shared<ResidualBlock>(cost_function, loss_function,
                                         param_blocks);
}

int ResidualBlock::jacobianOffset() const { return jacobian_offset_; }

void ResidualBlock::setJacobianOffset(const int jacobian_offset) {
  jacobian_offset_ = jacobian_offset;
}

}  // namespace unos
