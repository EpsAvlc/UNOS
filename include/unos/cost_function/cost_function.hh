#ifndef NALIO_COST_FUNCTION_COST_FUNCTION_HH__
#define NALIO_COST_FUNCTION_COST_FUNCTION_HH__

#include <Eigen/Core>
#include <memory>
#include "unos/manifold/manifold.hh"

namespace unos {
class CostFunction {
 public:
  using Ptr = std::shared_ptr<CostFunction>;
  virtual bool evaluate(const double* params, double* residual,
                        double** jacobian) const {};

  int getResidualSize() const { return res_size_; }

 protected:
  int param_size_;
  int res_size_;
};
};  // namespace unos

#endif  // NALIO_COST_FUNCTION_COST_FUNCTION_HH__
