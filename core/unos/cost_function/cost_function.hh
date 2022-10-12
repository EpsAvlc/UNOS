#ifndef UNOS_COST_FUNCTION_COST_FUNCTION_HH
#define UNOS_COST_FUNCTION_COST_FUNCTION_HH

#include <Eigen/Core>
#include <memory>

namespace unos {
class CostFunction {
 public:
  using Ptr = std::shared_ptr<CostFunction>;
  virtual bool evaluate(double const* const* params, double* residual,
                        double** jacobian) const {};

  int getResidualSize() const { return res_size_; }

  const std::vector<int>& parameterBlockSizes() const {
    return param_block_sizes_;
  }

 protected:
  std::vector<int>* mutableParameterBlockSizes() { return &param_block_sizes_; }
  std::vector<int>  param_block_sizes_;
  int               res_size_;
};
};  // namespace unos

#endif  // UNOS_COST_FUNCTION_COST_FUNCTION_HH
