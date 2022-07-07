#ifndef UNOS_SIZED_COST_FUNCTION_HH
#define UNOS_SIZED_COST_FUNCTION_HH

#include "unos/cost_function/cost_function.hh"

namespace nalio {
template <typename Manifold, size_t NRes>
class SizedCostFunction : public CostFunction {
 public:
  SizedCostFunction() { setNumResiduals(Nres); }
 protected:
};
};  // namespace nalio

#endif  // UNOS_SIZED_COST_FUNCTION_HH
