#include "unos/problem/program.hh"

namespace unos {
Program::Program()
    : max_jacobian_size_(0),
      num_residuals_(0),
      num_parameters_dim_(0),
      num_parameters_dof_(0) {}

void Program::addParameterBlock(double* parameters, int size,
                                ManifoldBase* manifold) {
  // TODO(caoming) : duplicate parameters check;
  ParameterBlock::Ptr new_parameter_block =
      ParameterBlock::create(parameters, size /*, nullptr*/);
  new_parameter_block->setJacobianOffset(num_parameters_dof_);
  parameter_blocks_.push_back(new_parameter_block);
  parameter_block_id_map_.insert(
      std::make_pair(parameters, parameter_block_id_map_.size()));
  if (manifold) {
    num_parameters_dof_ += manifold->dof();
  } else {
    num_parameters_dof_ += size;
  }

  num_parameters_dim_ += size;
}

void Program::addResidualBlock(const CostFunction*  cost_function,
                               const LossFunction*  loss_function,
                               double* const* const parameters,
                               int                  num_parameter_blocks) {
  std::vector<ParameterBlock::Ptr> parameter_blocks(num_parameter_blocks);
  const std::vector<int>&          parameter_sizes =
      cost_function->parameterBlockSizes();
  for (int pi = 0; pi < num_parameter_blocks; ++pi) {
    if (parameter_block_id_map_.count(parameters[pi]) == 0) {
      addParameterBlock(parameters[pi], parameter_sizes[pi]);
    }
    int parameter_block_id = parameter_block_id_map_[parameters[pi]];
    parameter_blocks[pi]   = parameter_blocks_[parameter_block_id];
    std::max(max_jacobian_size_, parameter_blocks_[parameter_block_id]->dim() *
                                     cost_function->getResidualSize());
  }

  ResidualBlock::Ptr new_residual_block =
      ResidualBlock::create(cost_function, loss_function, parameter_blocks);
  new_residual_block->setJacobianOffset(num_residuals_);
  residual_blocks_.push_back(new_residual_block);
  num_residuals_ += cost_function->getResidualSize();
}

const std::vector<typename ParameterBlock::Ptr>& Program::parameterBlocks()
    const {
  return parameter_blocks_;
}

const std::vector<typename ResidualBlock::Ptr>& Program::residualBlocks()
    const {
  return residual_blocks_;
}

int Program::numResiduals() const { return num_residuals_; }

int Program::numParametersDIM() const { return num_parameters_dim_; }

int Program::numParametersDOF() const { return num_parameters_dof_; }

int Program::numParameterBlocks() const { return parameter_blocks_.size(); }

int Program::maxJacobianSize() const { return max_jacobian_size_; }

void Program::setParameterBlockConstant(double* parameters) {
  int parameter_block_id = parameter_block_id_map_[parameters];
  parameter_blocks_[parameter_block_id]->setConst(true);
}

void Program::setManifold(double* parameters, ManifoldBase* manifold) {
  if (0 == parameter_block_id_map_.count(parameters)) {
    throw(std::invalid_argument("parameters has not been added."));
  }
  int parameter_block_id = parameter_block_id_map_[parameters];
  parameter_blocks_[parameter_block_id]->setManifold(manifold);
  manifolds_to_release_.insert(manifold);
}

void Program::parameterBlocksToStateVector(double* state_vector) const {
  for (auto& parameter_block_ptr : parameter_blocks_) {
    if (parameter_block_ptr->isConst()) {
      continue;
    }
    parameter_block_ptr->getState(state_vector);
    state_vector += parameter_block_ptr->dim();
  }
}

void Program::stateVectorToParameterBlocks(double const* state_vector) const {
  for (auto& parameter_block_ptr : parameter_blocks_) {
    if (parameter_block_ptr->isConst()) {
      continue;
    }
    parameter_block_ptr->setState(state_vector);
    state_vector += parameter_block_ptr->dim();
  }
}

void Program::rearrangeParameterBlocks() {
  // num_parameters_dim_ = 0;
  // num_parameters_dof_ = 0;
  // for (auto& parameter_block_ptr : parameter_blocks_) {
  //   if (parameter_block_ptr->isConst()) {
  //     continue;
  //   }
  // }
}

Program::~Program() {
  for (ManifoldBase* manifold : manifolds_to_release_) {
    delete manifold;
  }
}

}  // namespace unos
