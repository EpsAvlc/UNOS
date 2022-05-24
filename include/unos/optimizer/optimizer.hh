#ifndef NALIO_OPTIMIZER_OPTIMIZER_HH__
#define NALIO_OPTIMIZER_OPTIMIZER_HH__

#include <Eigen/Core>

#include "unos/cost_function/cost_function.hh"
#include "unos/manifold/manifold.hh"

namespace unos {
class Optimizer {
 public:
  virtual void init(const Manifold& init_val, const std::vector<const CostFunction::Ptr>& functions) = 0;
  virtual Eigen::VectorXd calculateDx() = 0;
};
}  // namespace unos

#endif  // NALIO_OPTIMIZER_OPTIMIZER_HH__
