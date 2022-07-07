#ifndef NALIO_COST_FUNCTION_COST_FUNCTION_HH__
#define NALIO_COST_FUNCTION_COST_FUNCTION_HH__

#include <Eigen/Core>
#include <memory>
#include "unos/manifold/manifold.hh"

namespace unos {
class CostFunction {
 public:
  using Ptr = std::shared_ptr<CostFunction>;

 protected:
  void setNumResiduals(size_t num_residuals) {
    num_residuals_ = num_residuals;
  }

  size_t num_residuals_;
};
};  // namespace unos

#endif  // NALIO_COST_FUNCTION_COST_FUNCTION_HH__
