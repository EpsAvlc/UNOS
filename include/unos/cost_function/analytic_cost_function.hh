#ifndef UNOS_ANALYTIC_COST_FUNCTION_HH
#define UNOS_ANALYTIC_COST_FUNCTION_HH

#include "unos/cost_function/sized_cost_function.hh"
namespace unos {
template <typename M, size_t NRes>
class AnalyticCostFunction : public SizeCostFunction<M, NRes> {
 public:
  virtual void evaluate(const M& manifold, double* residual, double** jacobian){

  };

 private:
};
}  // namespace unos

#endif  // UNOS_ANALYTIC_COST_FUNCTION_HH
