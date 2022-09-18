#include "unos/evaluator/evaluator.hh"

namespace unos {
Evaluator::Evaluator(const std::shared_ptr<Program>& program_ptr)
    : program_ptr_(program_ptr) {
  jacobian_writer_ =
      JacobianWriter::create(program_ptr_, JacobianWriter::Type::DENSE);
}

SparseMatrix::UniquePtr Evaluator::createJacobian() {
  return jacobian_writer_->createJacobian();
}

bool Evaluator::evaluate(const double* state, double* residuals,
                         SparseMatrix* jacobians) {
  program_ptr_->stateVectorToParameterBlocks(state);

  int num_residuals  = program_ptr_->numResiduals();
  int num_parameters = program_ptr_->numParameters();


  auto& residual_blocks     = program_ptr_->residualBlocks();
  auto& parameter_blocks    = program_ptr_->parameterBlocks();
  int   residual_block_ind  = 0;
  int   parameter_block_ind = 0;

  Eigen::Map<Eigen::VectorXd> res(residuals, num_residuals);

  // (TODO:caoming) not consider multiple thread.
  double** jacobian_data = prepareJacobianSpace();

  for (size_t ri = 0; ri < residual_blocks.size(); ++ri) {
    std::vector<ParameterBlock::Ptr> res_param_block_ids =
        residual_blocks[ri]->parameterBlocks();
    int res_block_size = residual_blocks[ri]->size();

    Eigen::VectorXd residual_part(res_block_size);
    residual_blocks[ri]->evaluate(residual_part.data(), jacobian_data);
    res.block(residual_block_ind, 0, res_block_size, 1) = residual_part;

    residual_block_ind += residual_blocks[ri]->size();
    jacobian_writer_->write(residual_blocks[ri], jacobian_data,
                            program_ptr_->mutableJacobian().get());
  }

  releaseJacobianSpace(jacobian_data);
  return true;
}

double** Evaluator::prepareJacobianSpace() {
  int num_residuals        = program_ptr_->numResiduals();
  int max_jacobian_size    = program_ptr_->maxJacobianSize();
  int num_parameter_blocks = program_ptr_->numParameterBlocks();

  double** ret = new double*[num_parameter_blocks];
  for (int i = 0; i < num_parameter_blocks; ++i) {
    ret[i] = new double[max_jacobian_size];
  }
  return ret;
}

void Evaluator::releaseJacobianSpace(double** jacobian) {
  int num_parameter_blocks = program_ptr_->numParameterBlocks();
  for (int i = 0; i < num_parameter_blocks; ++i) {
    delete[] jacobian[i];
  }
  delete[] jacobian;
}

}  // namespace unos
