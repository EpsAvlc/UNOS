#ifndef UNOS_OPTIMIZER_PROBLEM_HH__
#define UNOS_OPTIMIZER_PROBLEM_HH__

#include <unordered_map>

#include "unos/cost_function/cost_function.hh"
#include "unos/loss_function/loss_function.hh"
#include "unos/manifold/manifold.hh"
#include "unos/optimizer/optimizer.hh"
#include "unos/problem/parameter_block.hh"
#include "unos/problem/residual_block.hh"
#include "unos/utils/log_utils.hh"

namespace unos {
class Problem {
 public:
  Problem() : iter_num_(1), total_parameter_num_(0), total_residual_num_(0) {}

  void addParameterBlock(double* parameters, int size,
                         SubManifold* mandifold = nullptr) {
    // TODO(caoming) : duplicate parameters check;
    ParameterBlock::Ptr new_parameter_block =
        ParameterBlock::create(parameters, size, nullptr);
    parameter_blocks_.push_back(new_parameter_block);
    parameter_block_id_.insert(
        std::make_pair(parameters, parameter_block_id_.size()));
    parameter_block_ind_.push_back(total_parameter_num_);
    total_parameter_num_ += size;
  }

  void addResidualBlock(const CostFunction* cost_function,
                        const LossFunction* loss_function, double* parameters) {
    if (parameter_block_id_.count(parameters) == 0) {
      throw(std::invalid_argument(__STR_FUNCTION__ +
                                  " parameter has not been added."));
    }
    int param_block_id = parameter_block_id_[parameters];
    ResidualBlock::Ptr new_residual_block =
        ResidualBlock::create(cost_function, loss_function, param_block_id);
    residual_blocks_.push_back(new_residual_block);
    total_residual_num_ += cost_function->getResidualSize();
  }

  void optimize() {
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd residual;
    makeJacobian(&jacobian, &residual);
    std::cout << "-----------Jacobian:----------" << std::endl;
    std::cout << jacobian << std::endl;
    std::cout << "-----------Residual:----------" << std::endl;
    std::cout << residual << std::endl;
  }

 private:
  void makeJacobian(Eigen::MatrixXd* jaco, Eigen::VectorXd* res) {
    jaco->resize(total_residual_num_, total_parameter_num_);
    res->resize(total_residual_num_);
    int residual_block_ind = 0;
    for (size_t ri = 0; ri < residual_blocks_.size(); ++ri) {
      int res_param_block_id = residual_blocks_[ri]->parameterBlockId();
      int res_param_block_ind = parameter_block_ind_[res_param_block_id];
      int res_size = residual_blocks_[ri]->residualSize();
      int param_size = parameter_blocks_[res_param_block_id]->parameterSize();

      // Eigen::VectorXd residual_part =
      Eigen::VectorXd residual_part(res_size);
      Eigen::MatrixXd jacobian_part(res_size, param_size);
      // Eigen::MatrixXd jacobian_part =
      double* jacobian_array[] = {jacobian_part.data()};
      residual_blocks_[ri]->evaluate(
          parameter_blocks_[res_param_block_id]->mutableData(),
          residual_part.data(), jacobian_array);
      res->block(residual_block_ind, 0, res_size, 1) = residual_part;
      jaco->block(residual_block_ind, res_param_block_ind, res_size,
                  param_size) = jacobian_part;
      residual_block_ind += residual_blocks_[ri]->residualSize();
    }
  }

  std::vector<typename ParameterBlock::Ptr> parameter_blocks_;
  std::vector<typename ResidualBlock::Ptr> residual_blocks_;
  uint16_t iter_num_;
  std::unordered_map<double*, int> parameter_block_id_;
  std::vector<int> parameter_block_ind_;
  uint16_t total_residual_num_;
  uint16_t total_parameter_num_;
};
};  // namespace unos

#endif  // UNOS_OPTIMIZER_OPTIMIZER_HH__
