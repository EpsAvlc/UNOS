#include "unos/evaluator/evaluator.hh"

namespace unos {
Evaluator::Evaluator(const std::shared_ptr<Problem>& problem_ptr)
    : problem_ptr_(problem_ptr) {}

bool Evaluator::evaluate(const double* state, double* residuals,
                         double** jacobians) {
  int total_residual_num  = problem_ptr_->totalResidualNum();
  int total_parameter_num = problem_ptr_->totalParameterNum();

  Eigen::Map<Eigen::VectorXd> res(residuals, total_residual_num);

  auto& residual_blocks     = problem_ptr_->residualBlocks();
  auto& parameter_blocks    = problem_ptr_->parameterBlocks();
  int   residual_block_ind  = 0;
  int   parameter_block_ind = 0;
  for (size_t ri = 0; ri < parameter_blocks.size(); ++ri) {
    int res_param_block_id = residual_blocks[ri]->parameterBlockId();
    int res_block_size     = residual_blocks[ri]->residualSize();
    int param_block_size =
        parameter_blocks[res_param_block_id]->parameterSize();
    Eigen::Map<Eigen::MatrixXd> jaco(jacobians[ri], res_block_size,
                                     param_block_size);

    Eigen::VectorXd residual_part(res_block_size);
    double*         jacobian_array[] = {jacobians[ri]};
    residual_blocks[ri]->evaluate(
        parameter_blocks[res_param_block_id]->mutableData(),
        residual_part.data(), jacobian_array);
    res.block(residual_block_ind, 0, res_block_size, 1) = residual_part;

    residual_block_ind += residual_blocks[ri]->residualSize();
    parameter_block_ind += param_block_size;
  }
  return true;
}
}  // namespace unos
