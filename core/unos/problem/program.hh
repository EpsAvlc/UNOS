#ifndef UNOS_PROBLEM_PROGRAM_HH
#define UNOS_PROBLME_PROGRAM_HH

#include <memory>
#include <set>
#include <unordered_map>
#include "unos/cost_function/cost_function.hh"
#include "unos/loss_function/loss_function.hh"
#include "unos/manifold/manifold_base.hh"
#include "unos/problem/parameter_block.hh"
#include "unos/problem/residual_block.hh"
#include "unos/sparse_matrix/sparse_matrix.hh"
#include "unos/utils/log_utils.hh"

namespace unos {
class Program {
 public:
  using Ptr = std::shared_ptr<Program>;

  Program();

  void addParameterBlock(double* parameters, int size,
                         ManifoldBase* manifold = nullptr);

  void addResidualBlock(const CostFunction*  cost_function,
                        const LossFunction*  loss_function,
                        double* const* const parameters,
                        int                  num_parameter_blocks);

  const std::vector<typename ParameterBlock::Ptr>& parameterBlocks() const;

  const std::vector<typename ResidualBlock::Ptr>& residualBlocks() const;

  int numResiduals() const;

  int numParametersDIM() const;

  int numParametersDOF() const;

  int numParameterBlocks() const;

  int maxJacobianSize() const;

  void setParameterBlockConstant(double* parameters);

  void setManifold(double* parameters, ManifoldBase* manifold);

  void parameterBlocksToStateVector(double* state_vector) const;

  void stateVectorToParameterBlocks(double const* state_vector) const;

  void rearrangeParameterBlocks();

  SparseMatrix::UniquePtr& mutableJacobian() { return jacobian_; }

  SparseMatrix& Jacobian() const { return *jacobian_; }

  ~Program();

 private:
  int                                       iter_num_;
  int                                       num_residuals_;
  int                                       num_parameters_dof_;
  int                                       num_parameters_dim_;
  int                                       max_jacobian_size_;
  SparseMatrix::UniquePtr                   jacobian_;
  std::unordered_map<double*, int>          parameter_block_id_map_;
  std::vector<typename ParameterBlock::Ptr> parameter_blocks_;
  std::vector<typename ResidualBlock::Ptr>  residual_blocks_;
  std::set<ManifoldBase*>                   manifolds_to_release_;
};
};  // namespace unos

#endif  // PROGRAM_HH
