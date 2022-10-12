#ifndef UNOS_COST_FUNCTION_SIZED_COST_FUNCTION_HH
#define UNOS_COST_FUNCTION_SIZED_COST_FUNCTION_HH

#include "unos/cost_function/cost_function.hh"
// #include <cstd

namespace unos {
template <int NRes, int... NParams>
class SizedCostFunction : public CostFunction {
 public:
  SizedCostFunction() {
    param_block_sizes_ = std::vector<int>{NParams...};
    res_size_          = NRes;
  }

 protected:
};
};  // namespace unos

#endif  // UNOS_COST_FUNCTION_SIZED_COST_FUNCTION_HH
