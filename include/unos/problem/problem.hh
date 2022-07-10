#ifndef UNOS_OPTIMIZER_PROBLEM_HH__
#define UNOS_OPTIMIZER_PROBLEM_HH__

#include <unordered_map>

#include "unos/cost_function/cost_function.hh"
#include "unos/manifold/manifold.hh"
#include "unos/optimizer/optimizer.hh"
#include "unos/problem/parameter_block.hh"
#include "unos/problem/residual_block.hh"
#include "unos/loss_function/loss_function.hh"
#include "unos/utils/log_utils.hh"


namespace unos {
class Problem {
 public:
  Problem() : iter_num_(1) {}

  void addParameterBlock(double* parameters, int size,
                         SubManifold* mandifold = nullptr) {
    // TODO(caoming) : duplicate parameters check;
    ParameterBlock::Ptr new_parameter_block =
        ParameterBlock::create(parameters, size, nullptr);
    parameter_blocks_.push_back(new_parameter_block);
    parameter_block_id.insert(
        std::make_pair(parameters, parameter_block_id.size()));
  }

  void addResidualBlock(const CostFunction* cost_function, const LossFunction* loss_function, double* parameters) {
    if (parameter_block_id.count(parameters) == 0) {
      throw(std::invalid_argument(__STR_FUNCTION__ + " parameter has not been added."));
    }
    ResidualBlock::Ptr new_residual_block = ResidualBlock::create(cost_function, loss_function, parameters);
    residual_blocks_.push_back(new_residual_block);
  }

  void setOptimizer(const typename Optimizer::Ptr& optimizer) {
    // optimizer_ = optimizer;
  }

  void optimize() {
    // for (int ii = 0; ii < iter_num_; ++ii) {
    //   optimizer_->init(*manifold_, cost_functions_);
    //   *manifold_ = optimizer_->optimize();
    // }
  }

 private:
  std::vector<typename ParameterBlock::Ptr> parameter_blocks_;
  std::vector<typename ResidualBlock::Ptr> residual_blocks_;
  uint16_t iter_num_;
  std::unordered_map<double*, int> parameter_block_id;
};
};  // namespace unos

#endif  // UNOS_OPTIMIZER_OPTIMIZER_HH__
