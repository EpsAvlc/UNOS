#ifndef UNOS_OPTIMIZER_PROBLEM_HH__
#define UNOS_OPTIMIZER_PROBLEM_HH__

#include "unos/cost_function/cost_function.hh"
#include "unos/manifold/manifold.hh"
#include "unos/optimizer/optimizer.hh"

namespace unos {
template <typename ManifoldT>
class Problem {
 public:
  using OptimizerT = Optimizer<ManifoldT>;
  using CostFunctionT = CostFunction<ManifoldT>;
  Problem(const typename ManifoldT::Ptr& manifold)
      : iter_num_(1), manifold_(manifold) {}

  void setManifold(typename ManifoldT::Ptr& manifold) {
    manifold_ = manifold;
  }

  void addCostFunctions(const typename CostFunctionT::Ptr& cost_function) {
    cost_functions_.push_back(cost_function);
  }

  void setOptimizer(const typename OptimizerT::Ptr& optimizer) {
    optimizer_ = optimizer;
  }

  void optimize() {
    for (int ii = 0; ii < iter_num_; ++ii) {
      optimizer_->init(*manifold_, cost_functions_);
      *manifold_ = optimizer_->optimize();
    }
  }

 private:
  typename OptimizerT::Ptr optimizer_;
  typename ManifoldT::Ptr manifold_;
  std::vector<typename CostFunctionT::Ptr> cost_functions_;
  uint16_t iter_num_;
};
};  // namespace unos

#endif  // UNOS_OPTIMIZER_OPTIMIZER_HH__
