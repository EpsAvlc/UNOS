#ifndef NALIO_COST_FUNCTION_COST_FUNCTION_HH__
#define NALIO_COST_FUNCTION_COST_FUNCTION_HH__

#include <Eigen/Core>
#include <memory>
#include "unos/manifold/manifold.hh"

namespace unos {
template <typename M>
class CostFunction {
 public:
  using Ptr = std::shared_ptr<CostFunction>;
  virtual bool evaluate(const M& m, Eigen::VectorXd* residuals,
                        Eigen::MatrixXd* jacobians) const = 0;
};
};  // namespace unos

#endif  // NALIO_COST_FUNCTION_COST_FUNCTION_HH__
