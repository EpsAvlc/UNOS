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
  struct Options {
    int max_iteration_num;
  };
  virtual void init(const Options& options) = 0;
  // virtual M optimize() = 0;
  virtual void optimize(){};
};
}  // namespace unos

#endif  // NALIO_OPTIMIZER_OPTIMIZER_HH__
