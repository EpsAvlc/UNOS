#ifndef UNOS_PROBLEM_PROBLEM_HH
#define UNOS_PROBLEM_PROBLEM_HH

#include <memory>
#include <unordered_map>

#include "unos/cost_function/cost_function.hh"
#include "unos/loss_function/loss_function.hh"
#include "unos/manifold/manifold.hh"
#include "unos/optimizer/optimizer.hh"
#include "unos/problem/parameter_block.hh"
#include "unos/problem/residual_block.hh"
#include "unos/utils/log_utils.hh"

namespace unos {
class Problem : public std::enable_shared_from_this<Problem> {
 public:
  struct Config {
    int max_iteration_num = 50;
  };

  Problem(const Config& config);

  void addParameterBlock(double* parameters, int size,
                         SubManifold* mandifold = nullptr);

  void addResidualBlock(const CostFunction* cost_function,
                        const LossFunction* loss_function, double* parameters);

  const std::vector<typename ParameterBlock::Ptr>& parameterBlocks() const;

  const std::vector<typename ResidualBlock::Ptr>& residualBlocks() const;

  uint16_t totalResidualNum() const;

  uint16_t totalParameterNum() const;

  void optimize();

 private:
  void makeJacobian(Eigen::MatrixXd* jaco, Eigen::VectorXd* res);

  std::vector<typename ParameterBlock::Ptr> parameter_blocks_;
  std::vector<typename ResidualBlock::Ptr> residual_blocks_;
  uint16_t iter_num_;
  std::unordered_map<double*, int> parameter_block_id_;
  std::vector<int> parameter_block_ind_;
  uint16_t total_residual_num_;
  uint16_t total_parameter_num_;
};
};  // namespace unos

#endif  // UNOS_PROBLEM_PROBLEM_HH
