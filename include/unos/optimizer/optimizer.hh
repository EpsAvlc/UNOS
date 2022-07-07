#ifndef NALIO_OPTIMIZER_OPTIMIZER_HH__
#define NALIO_OPTIMIZER_OPTIMIZER_HH__

#include <Eigen/Core>
#include <memory>

#include "unos/cost_function/cost_function.hh"
#include "unos/manifold/manifold.hh"

namespace unos {
class Optimizer {
 public:
  using Ptr = std::shared_ptr<Optimizer>;
  // virtual void init(
  //     const M& init_val,
  //     const std::vector<typename CostFunction::Ptr>& functions) = 0;
  // virtual M optimize() = 0;
};
}  // namespace unos

#endif  // NALIO_OPTIMIZER_OPTIMIZER_HH__
