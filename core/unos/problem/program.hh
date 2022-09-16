#ifndef UNOS_PROBLEM_PROGRAM_HH
#define UNOS_PROBLME_PROGRAM_HH

#include <memory>
#include <unordered_map>
#include "unos/cost_function/cost_function.hh"
#include "unos/loss_function/loss_function.hh"
#include "unos/problem/parameter_block.hh"
#include "unos/problem/residual_block.hh"
#include "unos/utils/log_utils.hh"

namespace unos {
class Program {
 public:
  using Ptr = std::shared_ptr<Program>;

  Program();

  void addParameterBlock(double* parameters,
                         int     size /*, SubManifold* mandifold = nullptr*/);

  void addResidualBlock(const CostFunction*  cost_function,
                        const LossFunction*  loss_function,
                        double* const* const parameters,
                        int                  num_parameter_blocks);

  const std::vector<typename ParameterBlock::Ptr>& parameterBlocks() const;

  const std::vector<typename ResidualBlock::Ptr>& residualBlocks() const;

  int numResiduals() const;

  int numParameters() const;

  int numParameterBlocks() const;

  int maxJacobianSize() const;

  void parameterBlocksToStateVector(double* state_vector) const;

 private:
  std::vector<typename ParameterBlock::Ptr> parameter_blocks_;
  std::vector<typename ResidualBlock::Ptr>  residual_blocks_;
  int                                       iter_num_;
  std::unordered_map<double*, int>          parameter_block_id_map_;
  int                                       num_residuals_;
  int                                       num_parameters_;
  int                                       max_jacobian_size;
};
};  // namespace unos

#endif  // PROGRAM_HH
