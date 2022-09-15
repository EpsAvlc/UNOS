#ifndef UNOS_ANALYTIC_COST_FUNCTION_HH
#define UNOS_ANALYTIC_COST_FUNCTION_HH

#include "unos/cost_function/sized_cost_function.hh"
namespace unos {
template <size_t param_size, size_t NRes>
class AnalyticCostFunction : public SizedCostFunction<param_size, NRes> {
 public:
 private:
};
}  // namespace unos

#endif  // UNOS_ANALYTIC_COST_FUNCTION_HH
