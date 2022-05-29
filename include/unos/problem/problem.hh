#ifndef UNOS_OPTIMIZER_PROBLEM_HH__
#define UNOS_OPTIMIZER_PROBLEM_HH__

#include "unos/cost_function/cost_function.hh"
#include "unos/manifold/manifold.hh"
#include "unos/optimizer/optimizer.hh"

namespace unos {

class Problem {
 public:
  Problem() : iter_num_(1) {}

  void setManifold(Manifold::Ptr& manifold) { manifold_ = manifold; }

  void addCostFunctions(const CostFunction::Ptr& cost_function) {
    cost_functions_.push_back(cost_function);
  }

  void setOptimizer(const Optimizer::Ptr& optimizer) { optimizer_ = optimizer; }

  void optimize() {
    for (int ii = 0; ii < iter_num_; ++ii) {
      optimizer_->init(*manifold_, cost_functions_);
      *manifold_ = optimizer_->optimize();
    }
  }

 private:
  Optimizer::Ptr optimizer_;
  Manifold::Ptr manifold_;
  std::vector<CostFunction::Ptr> cost_functions_;
  uint16_t iter_num_;
};
};  // namespace unos

#endif  // UNOS_OPTIMIZER_OPTIMIZER_HH__
