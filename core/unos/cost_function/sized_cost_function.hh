#ifndef UNOS_COST_FUNCTION_SIZED_COST_FUNCTION_HH
#define UNOS_COST_FUNCTION_SIZED_COST_FUNCTION_HH

#include "unos/cost_function/cost_function.hh"
// #include <cstd

namespace unos {
template <size_t ParamSize, size_t NRes>
class SizedCostFunction : public CostFunction {
 public:
  SizedCostFunction() {
    param_size_ = ParamSize;
    res_size_ = NRes;
  }

 protected:

};
};  // namespace unos

#endif // UNOS_COST_FUNCTION_SIZED_COST_FUNCTION_HH
