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
  for (size_t pi = 0; pi < parameter_blocks_.size(); ++pi) {
    if (parameter_blocks_[pi]->getManifold()) {
      Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>
          jaco_wrt_ambient(jacobian[pi], cost_function_->getResidualSize(),
                           parameter_blocks_[pi]->dim());

      Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>
          jaco_wrt_tangent(jacobian[pi], cost_function_->getResidualSize(),
                           parameter_blocks_[pi]->dof());

      Eigen::Matrix<double, -1, -1, Eigen::RowMajor> oplus_jacobian(
          parameter_blocks_[pi]->getManifold()->dim(),
          parameter_blocks_[pi]->getManifold()->dof());

      parameter_blocks_[pi]->getManifold()->oplusJacobian(
          parameter_blocks_[pi]->mutableData(), oplus_jacobian.data());

      jaco_wrt_tangent = (jaco_wrt_ambient * oplus_jacobian).eval();
    }
  }
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
